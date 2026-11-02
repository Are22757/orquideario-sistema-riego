// ============================================================
//  Heltec WiFi LoRa 32 V3  —  TRANSMISOR relay (v5)
//  Recibe datos de los Makerfabs Soil Moisture Sensor (SX1276)
//  y los retransmite al Heltec V3 Receptor con prefijo "HLT:"
//
//  NUEVO en v5: el receptor ahora es un ESP32 + REYAX RYLR998.
//    Cada paquete al receptor lleva el encabezado que exige el RYLR998:
//    [dst_lo, dst_hi, src_lo, src_hi, len] + "HLT:..."  (CRC LoRa ON)
//    Validado el 22/oct/2026: Heltec -> RYLR998 sin pérdidas.
//
//  v4: maestro Modbus RTU (RS485) que lee el sensor
//  ambiental (HR / T) y lo envía al receptor como paquete aparte:
//    "HLT:MB MB=1 ST=1 RH=60.1 TA=21.3 CNT=123*5A"
//    "HLT:MB MB=0 FC=3*41"            (esclavo sin respuesta)
//
//  Escenario:
//    Makerfabs Sensor → (LoRa SF9 CR4/7) → [este nodo]
//    Sensor RS485     → (Modbus RTU)     → [este nodo]
//                     → (LoRa SF9 CR4/5) → ESP32 + RYLR998 (receptor)
//
//  Librerías: RadioLib (jgromes), U8g2 (oliver),
//             ModbusMaster (Doc Walker)
//  Board: "Heltec WiFi LoRa 32(V3)"
// ============================================================

#include <RadioLib.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <ModbusMaster.h>

// ----- Pines SX1262 ----------------------------------------
#define LORA_CS    8
#define LORA_IRQ   14
#define LORA_RST   12
#define LORA_BUSY  13

// ----- Pines OLED ------------------------------------------
#define OLED_SDA   17
#define OLED_SCL   18
#define OLED_RST   21
#define VEXT_PIN   36

// ----- Parámetros LoRa — SENSOR Makerfabs -----------------
#define SENSOR_FREQ   915.0
#define SENSOR_BW     125.0
#define SENSOR_SF     9
#define SENSOR_CR     7
#define SENSOR_SW     RADIOLIB_SX127X_SYNC_WORD
#define SENSOR_PWR    10
#define SENSOR_PRE    8

// ----- Parámetros LoRa — Heltec Receptor ------------------
#define RX_FREQ   915.0
#define RX_BW     125.0
#define RX_SF     9
#define RX_CR     5
#define RX_SW     0x12
#define RX_PWR    22
#define RX_PRE    12

// ----- Prefijo para identificar paquetes del Heltec -------
#define PREFIJO "HLT:"

// ----- Direcciones REYAX (deben coincidir con AT+ADDRESS) --
#define REYAX_DIR_RECEPTOR  2    // AT+ADDRESS del RYLR998 receptor
#define REYAX_DIR_ORIGEN    1    // dirección con la que se identifica este nodo
#define REYAX_MAX_DATOS     240  // máximo de datos que acepta el RYLR998

// ----- Modbus RTU / RS485 (validado 10/oct/2026) -----------
#define RS485_RX      6     // RO  (vía BSS138, HV6 → LV6)
#define RS485_TX      5     // DI  (directo)
#define RS485_DE      4     // DE + RE (directo)
#define MB_BAUD       9600
#define MB_ESCLAVO_ID 1
#define MB_REG_INICIO 0x0000
#define MB_NUM_REGS   4     // 0=HR×10, 1=T×10 (con signo), 2=estado, 3=contador

#define MB_LECTURA_MS 10000UL   // período de lectura del esclavo
#define MB_ENVIO_MS   30000UL   // período de envío por LoRa (≈0.3 s de aire c/u)

// Rangos físicos plausibles (lectura fuera de rango = inválida)
#define HR_MIN   0.0
#define HR_MAX   100.0
#define T_MIN   -40.0
#define T_MAX    85.0

// ============================================================

SX1262 radio = new Module(LORA_CS, LORA_IRQ, LORA_RST, LORA_BUSY);

U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(
  U8G2_R0, OLED_RST, OLED_SCL, OLED_SDA
);

ModbusMaster nodo;

volatile bool paqueteRecibido = false;
volatile bool transmitiendo   = false;

struct Paquete {
  String datos  = "--";
  float  rssi   = 0;
  float  snr    = 0;
  int    cuenta = 0;
} ultimo;

struct Ambiente {
  bool     comOk          = false;  // última lectura Modbus válida
  float    hr             = 0;      // %HR
  float    t              = 0;      // °C
  uint16_t estado         = 0;      // reg 2: 1 = AHT10 OK, 0 = simulado
  uint16_t contador       = 0;      // reg 3: contador de lecturas del esclavo
  uint16_t fallosSeguidos = 0;
  uint32_t intentos       = 0;
  uint32_t okTotal        = 0;
  uint32_t errTotal       = 0;
} amb;

unsigned long tUltLecturaMB = 0;
unsigned long tUltEnvioMB   = 0;

// ============================================================

void IRAM_ATTR alRecibirPaquete() {
  if (!transmitiendo) paqueteRecibido = true;
}

// ------------------------------------------------------------
void iniciarVEXT() {
  pinMode(VEXT_PIN, OUTPUT);
  digitalWrite(VEXT_PIN, LOW);
  delay(100);
}

// ------------------------------------------------------------
// Control de dirección del MAX485 (llamado por ModbusMaster)
void preTransmission()  { digitalWrite(RS485_DE, HIGH); }
void postTransmission() { digitalWrite(RS485_DE, LOW);  }

// ------------------------------------------------------------
void lineaAmbiente(char* buf, size_t n) {
  if (amb.intentos == 0) {
    snprintf(buf, n, "MB: iniciando...");
  } else if (amb.comOk) {
    snprintf(buf, n, "MB %.1f%% %.1fC%s",
             amb.hr, amb.t, (amb.estado == 1) ? "" : " SIM");
  } else {
    snprintf(buf, n, "MB: sin resp (%u)", amb.fallosSeguidos);
  }
}

// ------------------------------------------------------------
void actualizarPantalla() {
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tf);

  display.drawStr(0, 10, "TRANSMISOR relay");
  display.drawHLine(0, 13, 128);

  char bufAmb[22];
  lineaAmbiente(bufAmb, sizeof(bufAmb));

  if (ultimo.cuenta == 0) {
    display.drawStr(5, 32, "Esperando sensor...");
    display.drawStr(0, 50, bufAmb);
  } else {
    String linDatos = ultimo.datos;
    if (linDatos.length() > 21) linDatos = linDatos.substring(0, 21);
    display.drawStr(0, 26, linDatos.c_str());

    char bufSig[22];
    snprintf(bufSig, sizeof(bufSig), "RSSI:%.0fdBm SNR:%.1fdB",
             ultimo.rssi, ultimo.snr);
    display.drawStr(0, 38, bufSig);

    display.drawStr(0, 50, bufAmb);

    char bufCnt[22];
    snprintf(bufCnt, sizeof(bufCnt), "Reenviados: %d", ultimo.cuenta);
    display.drawStr(0, 63, bufCnt);
  }

  display.sendBuffer();
}

// ------------------------------------------------------------
void pantallaError(String msg) {
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tf);
  display.drawStr(0, 12, "!! ERROR !!");
  display.drawHLine(0, 15, 128);
  if (msg.length() > 21) {
    display.drawStr(0, 32, msg.substring(0, 21).c_str());
    display.drawStr(0, 46, msg.substring(21).c_str());
  } else {
    display.drawStr(0, 38, msg.c_str());
  }
  display.drawStr(0, 63, "Reinicia el ESP32");
  display.sendBuffer();
}

// ------------------------------------------------------------
// Reconfigura el radio al modo "escuchar sensores".
// Solo toca los parámetros que realmente difieren del modo receptor
// (Coding Rate, potencia, preámbulo) — evita un radio.begin() completo,
// que reinicializa el chip entero y crea una ventana "ciega" larga
// durante la cual se pierden paquetes de los sensores.
void configurarModoSensor() {
  radio.setCodingRate(SENSOR_CR);
  radio.setOutputPower(SENSOR_PWR);
  radio.setPreambleLength(SENSOR_PRE);
  radio.setCRC(true);   // los paquetes del sensor sí deben validarse con CRC

  radio.setDio1Action(alRecibirPaquete);
  radio.startReceive();
}

// ------------------------------------------------------------
// Reconfigura el radio al modo "hablar con el Receptor".
// Mismo criterio: solo cambia lo que es distinto (CR, potencia, preámbulo).
void configurarModoReceptor() {
  radio.setDio1Action(NULL);
  radio.setCodingRate(RX_CR);
  radio.setOutputPower(RX_PWR);
  radio.setPreambleLength(RX_PRE);
  radio.setCRC(true);   // el RYLR998 recibe con CRC LoRa activado (validado 22/oct)
}

// ------------------------------------------------------------
// Verifica que el paquete tenga la forma esperada de un sensor Makerfabs
// ("IDxxxxxx REPLY : SOIL INEDX:... H:... T:... BAT:...") y que todos sus
// caracteres sean texto imprimible. Sirve para detectar paquetes corruptos
// por colisión entre sensores (dos transmitiendo casi al mismo tiempo),
// que a veces igual pasan la validación CRC del radio pero traen basura.
bool esPaqueteValido(String datos) {
  if (datos.length() < 20 || datos.length() > 150) return false;
  if (!datos.startsWith("ID")) return false;
  if (datos.indexOf("REPLY") < 0)   return false;
  if (datos.indexOf("INEDX:") < 0)  return false;
  if (datos.indexOf("H:") < 0)      return false;
  if (datos.indexOf("T:") < 0)      return false;
  if (datos.indexOf("BAT:") < 0)    return false;

  // Todos los caracteres deben ser imprimibles (ASCII 32-126)
  for (int i = 0; i < (int)datos.length(); i++) {
    char c = datos.charAt(i);
    if (c < 32 || c > 126) return false;
  }

  return true;
}

// ------------------------------------------------------------
void escucharSensor() {
  configurarModoSensor();
}

// ------------------------------------------------------------
void transmitirAlReceptor(String datos) {
  transmitiendo = true;

  configurarModoReceptor();

  // Prefijo para que el receptor identifique el origen
  String payload = String(PREFIJO) + datos;
  size_t n = payload.length();
  if (n > REYAX_MAX_DATOS) n = REYAX_MAX_DATOS;

  // Encabezado que exige el RYLR998: [dst_lo, dst_hi, src_lo, src_hi, len]
  uint8_t buf[5 + REYAX_MAX_DATOS];
  buf[0] = REYAX_DIR_RECEPTOR & 0xFF;
  buf[1] = (REYAX_DIR_RECEPTOR >> 8) & 0xFF;
  buf[2] = REYAX_DIR_ORIGEN & 0xFF;
  buf[3] = (REYAX_DIR_ORIGEN >> 8) & 0xFF;
  buf[4] = (uint8_t)n;
  memcpy(buf + 5, payload.c_str(), n);

  int txEstado = radio.transmit(buf, 5 + n);

  if (txEstado == RADIOLIB_ERR_NONE) {
    Serial.println("[TX] Reenviado al receptor OK: " + payload);
  } else {
    Serial.print("[TX] Error al reenviar: ");
    Serial.println(txEstado);
  }

  transmitiendo = false;
}

// ============================================================
//  MODBUS
// ============================================================

// Checksum XOR estilo NMEA. El enlace A3→A1 va SIN CRC de radio,
// así que el receptor (capa 2a) validará el paquete MB con esto.
uint8_t checksumXOR(const char* s) {
  uint8_t c = 0;
  while (*s) c ^= (uint8_t)(*s++);
  return c;
}

// ------------------------------------------------------------
const char* textoErrorModbus(uint8_t r) {
  switch (r) {
    case 0x01: return "excepcion: funcion ilegal";
    case 0x02: return "excepcion: direccion ilegal";
    case 0x03: return "excepcion: valor ilegal";
    case 0x04: return "excepcion: falla del esclavo";
    case 0xE0: return "ID de esclavo invalido";
    case 0xE1: return "funcion invalida en respuesta";
    case 0xE2: return "timeout (sin respuesta)";
    case 0xE3: return "CRC invalido";
    default:   return "desconocido";
  }
}

// ------------------------------------------------------------
// Lectura bloqueante: ~30-40 ms si el esclavo responde,
// hasta 2 s (timeout fijo de ModbusMaster) si no responde.
// Mientras tanto el SX1262 sigue en RX y guarda el paquete en su buffer;
// el flag paqueteRecibido lo atiende el loop apenas termine la lectura.
void leerModbus() {
  amb.intentos++;
  uint8_t r = nodo.readHoldingRegisters(MB_REG_INICIO, MB_NUM_REGS);

  if (r == nodo.ku8MBSuccess) {
    float    hr  = nodo.getResponseBuffer(0) / 10.0;
    float    t   = (int16_t)nodo.getResponseBuffer(1) / 10.0;
    uint16_t est = nodo.getResponseBuffer(2);
    uint16_t cnt = nodo.getResponseBuffer(3);

    if (hr < HR_MIN || hr > HR_MAX || t < T_MIN || t > T_MAX) {
      amb.comOk = false;
      amb.fallosSeguidos++;
      amb.errTotal++;
      Serial.printf("[MB] Lectura fuera de rango (HR=%.1f T=%.1f) — descartada.\n", hr, t);
      return;
    }

    amb.hr = hr;  amb.t = t;  amb.estado = est;  amb.contador = cnt;
    amb.comOk = true;
    amb.fallosSeguidos = 0;
    amb.okTotal++;

    Serial.printf("[MB] OK  HR=%.1f %%  T=%.1f C  ST=%u  CNT=%u  (ok:%lu err:%lu)\n",
                  amb.hr, amb.t, amb.estado, amb.contador,
                  (unsigned long)amb.okTotal, (unsigned long)amb.errTotal);
  } else {
    amb.comOk = false;
    amb.fallosSeguidos++;
    amb.errTotal++;
    Serial.printf("[MB] ERROR 0x%02X (%s) — fallos seguidos: %u\n",
                  r, textoErrorModbus(r), amb.fallosSeguidos);
  }
}

// ------------------------------------------------------------
void enviarAmbiente() {
  char cuerpo[64];
  if (amb.comOk) {
    snprintf(cuerpo, sizeof(cuerpo), "MB MB=1 ST=%u RH=%.1f TA=%.1f CNT=%u",
             amb.estado, amb.hr, amb.t, amb.contador);
  } else {
    snprintf(cuerpo, sizeof(cuerpo), "MB MB=0 FC=%u", amb.fallosSeguidos);
  }

  char paquete[72];
  snprintf(paquete, sizeof(paquete), "%s*%02X", cuerpo, checksumXOR(cuerpo));

  Serial.println("[TX] Enviando ambiente al receptor...");
  transmitirAlReceptor(String(paquete));
  escucharSensor();
}

// ------------------------------------------------------------
// Tareas Modbus. Se llama solo cuando no hay paquete LoRa pendiente,
// para que los Makerfabs tengan siempre prioridad.
void tareasModbus() {
  if (millis() - tUltLecturaMB >= MB_LECTURA_MS) {
    tUltLecturaMB = millis();
    leerModbus();
    actualizarPantalla();
  }

  // Si llegó un paquete LoRa durante la lectura, se atiende antes de transmitir
  if (paqueteRecibido) return;

  if (amb.intentos > 0 && millis() - tUltEnvioMB >= MB_ENVIO_MS) {
    tUltEnvioMB = millis();
    enviarAmbiente();
  }
}

// ============================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("=== Heltec V3 — TRANSMISOR relay v5 (LoRa + Modbus -> REYAX) ===");

  iniciarVEXT();

  display.begin();
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tf);
  display.drawStr(0, 12, "Heltec V3 Relay v5");
  display.drawStr(0, 28, "Sensor -> Receptor");
  display.drawStr(0, 44, "Iniciando...");
  display.sendBuffer();
  Serial.println("[OLED] Pantalla iniciada.");

  // RS485 / Modbus
  pinMode(RS485_DE, OUTPUT);
  digitalWrite(RS485_DE, LOW);                        // MAX485 en recepción
  Serial1.begin(MB_BAUD, SERIAL_8N1, RS485_RX, RS485_TX);
  nodo.begin(MB_ESCLAVO_ID, Serial1);
  nodo.preTransmission(preTransmission);
  nodo.postTransmission(postTransmission);
  Serial.println("[MB] Maestro Modbus listo | 9600 8N1 | esclavo ID 1");

  int estado = radio.begin(
    SENSOR_FREQ, SENSOR_BW, SENSOR_SF, SENSOR_CR,
    SENSOR_SW, SENSOR_PWR, SENSOR_PRE
  );

  if (estado != RADIOLIB_ERR_NONE) {
    String err = "SX1262 err:" + String(estado);
    Serial.println("[ERROR] " + err);
    pantallaError(err);
    while (true) delay(1000);
  }

  Serial.println("[OK] SX1262 listo.");

  radio.setDio1Action(alRecibirPaquete);
  radio.startReceive();

  // Primera lectura Modbus ~2 s después del arranque
  tUltLecturaMB = millis() - MB_LECTURA_MS + 2000;
  tUltEnvioMB   = millis();

  actualizarPantalla();

  Serial.println("Escuchando sensor | SF9 BW125 CR4/7 Pre8 SW:SX127X");
  Serial.println("-----------------------------------------------------");
}

// ============================================================
void loop() {
  if (!paqueteRecibido) {
    tareasModbus();
  }

  if (paqueteRecibido) {
    paqueteRecibido = false;

    String datos;
    int estado = radio.readData(datos);

    if (estado == RADIOLIB_ERR_NONE) {
      datos.trim();
      if (datos.length() == 0) {
        Serial.println("[RX] Paquete vacío — ignorado.");
        radio.startReceive();
        return;
      }

      ultimo.rssi   = radio.getRSSI();
      ultimo.snr    = radio.getSNR();

      if (!esPaqueteValido(datos)) {
        Serial.println("[RX] Paquete con formato invalido - descartado (posible colision entre sensores).");
        Serial.println("  Datos crudos (no reenviado): " + datos);
        Serial.print  ("  RSSI  : "); Serial.print(ultimo.rssi); Serial.println(" dBm");
        Serial.print  ("  SNR   : "); Serial.print(ultimo.snr);  Serial.println(" dB");
        escucharSensor();
        return;
      }

      ultimo.datos  = datos;
      ultimo.cuenta++;

      Serial.println("\n[RX] Paquete #" + String(ultimo.cuenta) + " del sensor");
      Serial.println("  Datos : " + datos);
      Serial.print  ("  RSSI  : "); Serial.print(ultimo.rssi); Serial.println(" dBm");
      Serial.print  ("  SNR   : "); Serial.print(ultimo.snr);  Serial.println(" dB");

      actualizarPantalla();

      Serial.println("[TX] Reenviando al receptor...");
      transmitirAlReceptor(datos);

      Serial.println("[RX] Volviendo a escuchar sensor...");
      escucharSensor();

    } else if (estado == RADIOLIB_ERR_CRC_MISMATCH) {
      Serial.println("[RX] CRC mismatch — ignorado.");
      radio.startReceive();
    } else {
      Serial.print("[RX] Error: ");
      Serial.println(estado);
      radio.startReceive();
    }
  }
}
