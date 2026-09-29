// ============================================================
//  Heltec WiFi LoRa 32 V3  —  TRANSMISOR relay (v3)
//  Recibe datos del Makerfabs Soil Moisture Sensor (SX1276)
//  y los retransmite al Heltec V3 Receptor con prefijo "HLT:"0
//
//  Escenario:
//    Makerfabs Sensor → (LoRa SF9 CR4/7) → [este nodo]
//                     → (LoRa SF9 CR4/5) → Heltec Receptor
//
//  Librerías: RadioLib (jgromes), U8g2 (oliver)
//  Board: "Heltec WiFi LoRa 32(V3)"
// ============================================================

#include <RadioLib.h>
#include <U8g2lib.h>
#include <Wire.h>

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

// ============================================================

SX1262 radio = new Module(LORA_CS, LORA_IRQ, LORA_RST, LORA_BUSY);

U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(
  U8G2_R0, OLED_RST, OLED_SCL, OLED_SDA
);

volatile bool paqueteRecibido = false;
volatile bool transmitiendo   = false;

struct Paquete {
  String datos  = "--";
  float  rssi   = 0;
  float  snr    = 0;
  int    cuenta = 0;
} ultimo;

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
void actualizarPantalla() {
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tf);

  display.drawStr(0, 10, "TRANSMISOR relay");
  display.drawHLine(0, 13, 128);

  if (ultimo.cuenta == 0) {
    display.drawStr(5, 35, "Esperando sensor...");
    display.drawStr(45, 50, "...");
  } else {
    String linDatos = ultimo.datos;
    if (linDatos.length() > 21) linDatos = linDatos.substring(0, 21);
    display.drawStr(0, 26, linDatos.c_str());

    char bufSig[22];
    snprintf(bufSig, sizeof(bufSig), "RSSI:%.0fdBm SNR:%.1fdB",
             ultimo.rssi, ultimo.snr);
    display.drawStr(0, 38, bufSig);

    int barW = map(constrain((int)ultimo.rssi, -120, -50), -120, -50, 0, 60);
    display.drawStr(0, 50, "Senal:");
    display.drawFrame(40, 42, 62, 9);
    display.drawBox(40, 42, barW, 9);

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
void escucharSensor() {
  radio.begin(SENSOR_FREQ, SENSOR_BW, SENSOR_SF, SENSOR_CR,
              SENSOR_SW, SENSOR_PWR, SENSOR_PRE);
  radio.setDio1Action(alRecibirPaquete);
  radio.startReceive();
}

// ------------------------------------------------------------
void transmitirAlReceptor(String datos) {
  transmitiendo = true;
  radio.setDio1Action(NULL);

  radio.begin(RX_FREQ, RX_BW, RX_SF, RX_CR,
              RX_SW, RX_PWR, RX_PRE);
  radio.setCRC(0);

  // Agrega prefijo para que el receptor identifique el origen
  String payload = String(PREFIJO) + datos;
  int txEstado = radio.transmit(payload);

  if (txEstado == RADIOLIB_ERR_NONE) {
    Serial.println("[TX] Reenviado al receptor OK: " + payload);
  } else {
    Serial.print("[TX] Error al reenviar: ");
    Serial.println(txEstado);
  }

  transmitiendo = false;
}

// ============================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("=== Heltec V3 — TRANSMISOR relay v3 ===");

  iniciarVEXT();

  display.begin();
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tf);
  display.drawStr(0, 12, "Heltec V3 Relay");
  display.drawStr(0, 28, "Sensor -> Receptor");
  display.drawStr(0, 44, "Iniciando...");
  display.sendBuffer();
  Serial.println("[OLED] Pantalla iniciada.");

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

  actualizarPantalla();

  Serial.println("Escuchando sensor | SF9 BW125 CR4/7 Pre8 SW:SX127X");
  Serial.println("-----------------------------------------------------");
}

// ============================================================
void loop() {
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
