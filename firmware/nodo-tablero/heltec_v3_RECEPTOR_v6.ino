// ============================================================
//  Heltec WiFi LoRa 32 V3  —  RECEPTOR v6 (tablero de control)
//
//  Reemplaza al ESP32 + REYAX RYLR998. Misma lógica y MISMOS tópicos
//  MQTT, así que Node-RED no cambia. El transmisor A3 (v5) NO se toca:
//  este receptor entiende el encabezado REYAX que el A3 agrega.
//
//    Heltec A3 (transmisor v5) → LoRa → [Heltec V3 receptor] → WiFi/MQTT → Node-RED
//
//  - Radio igual que el RYLR998: 915 MHz, SF9, BW125, CR4/5, preámbulo 12,
//    sync word 0x12, CRC activado
//  - Quita el encabezado REYAX [dst_lo, dst_hi, src_lo, src_hi, len]
//  - Paquetes "HLT:MB ..."  → orquideario/ambiente (valida checksum)
//  - Paquetes "HLT:ID..."   → orquideario/lora/N   (Makerfabs)
//  - Ignora los paquetes que llegan directo de los sensores (sin "HLT:"),
//    porque el A3 ya los retransmite: así no hay datos duplicados
//  - Relé de riego por MQTT con protecciones locales:
//       1) arranca APAGADO   2) tiempo máximo (Ticker, no depende del loop)
//       3) se apaga si se pierde WiFi/MQTT   4) un ON repetido no alarga el riego
//       5) se apaga al iniciar una actualización OTA
//  - Estado del receptor (online/offline, retain + Last Will)
//  - OTA: tras la primera carga por cable, Tools → Port → "orquideario-receptor"
//
//  Conexiones del relé:
//    Relé VCC → 5V del Heltec   GND → GND   IN → GPIO47
//    (módulo con jumper en H, activo en ALTO)
//
//  Board: "Heltec WiFi LoRa 32(V3)"
//  Librerías: RadioLib (jgromes), U8g2 (oliver), PubSubClient (knolleary).
//             WiFi, Ticker y ArduinoOTA vienen con el core ESP32.
// ============================================================

#include <RadioLib.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Ticker.h>
#include <ArduinoOTA.h>

// ----- Relé de riego ---------------------------------------
#define RELE_PIN    47
#define RELE_ON     HIGH
#define RELE_OFF    LOW
#define TIEMPO_MAX_APERTURA_SEG  300

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

// ----- LoRa (igual que AT+PARAMETER=9,7,1,12 / NETWORKID 18) --
#define FRECUENCIA   915.0
#define BANDWIDTH    125.0
#define SF           9
#define CODING_RATE  5
#define SYNC_WORD    0x12
#define POTENCIA     22
#define PREAMBLE     12

// ----- Encabezado REYAX que agrega el A3 -------------------
#define MI_DIRECCION  2      // el A3 envía a la dirección 2
#define PREFIJO       "HLT:"

// ----- WiFi ------------------------------------------------
const char* WIFI_SSID     = "Lis";         // ← tu red real
const char* WIFI_PASSWORD = "misdatos";  // ← tu contraseña real

// ----- MQTT ------------------------------------------------
const char* MQTT_BROKER   = "172.20.10.2";    // ← IP de la PC con Node-RED
const int   MQTT_PORT     = 1883;
const char* MQTT_CLIENT   = "ReceptorHeltecV3";

#define MQTT_REINTENTO_MS   5000UL
#define WIFI_REINTENTO_MS   30000UL

// ----- OTA -------------------------------------------------
const char* OTA_HOSTNAME = "orquideario-receptor";
const char* OTA_PASSWORD = "orquideario";

// ----- Mapeo de ID de hardware → número de sensor (1 a 6) --
#define NUM_SENSORES 6
const char* SENSOR_IDS[NUM_SENSORES] = {
  "ID012101",  // Sensor 1
  "ID012129",  // Sensor 2
  "ID012097",  // Sensor 3
  "ID012127",  // Sensor 4
  "ID010000",  // Sensor 5 (sensor de pruebas de Lis; el UVG es ID012107)
  "ID012096"   // Sensor 6
};

// ----- Tópicos (idénticos a los receptores anteriores) -----
const char* TOPIC_BASE            = "orquideario/lora/";
const char* TOPIC_RAW             = "orquideario/raw";
const char* TOPIC_AMBIENTE        = "orquideario/ambiente";
const char* TOPIC_RECEPTOR_ESTADO = "orquideario/receptor/estado";
const char* TOPIC_RIEGO_CMD       = "orquideario/riego/cmd";
const char* TOPIC_RIEGO_ESTADO    = "orquideario/riego/estado";

// ============================================================

SX1262 radio = new Module(LORA_CS, LORA_IRQ, LORA_RST, LORA_BUSY);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, OLED_RST, OLED_SCL, OLED_SDA);
WiFiClient   espClient;
PubSubClient mqtt(espClient);

volatile bool paqueteRecibido = false;

// Estado del relé
bool           releEncendido = false;
unsigned long  tInicioRele   = 0;
String         motivoRele    = "arranque";
bool           publicarRiego = true;
Ticker         tickerSeguridad;
volatile bool  cortePorTiempoPendiente = false;

// Para la pantalla
float  ultRssi = 0, ultSnr = 0;
int    pktsTotal = 0;
int    ultSensor = 0;
String ultHum = "--";
String ambTxt = "--";
unsigned long tUltPantalla = 0;

unsigned long tUltIntentoMQTT = 0;
unsigned long tUltIntentoWiFi = 0;
bool otaIniciado = false;

// ============================================================
void IRAM_ATTR alRecibirPaquete() { paqueteRecibido = true; }

void iniciarVEXT() {
  pinMode(VEXT_PIN, OUTPUT);
  digitalWrite(VEXT_PIN, LOW);   // LOW = OLED alimentada
  delay(100);
}

// ============================================================
//  RELÉ
// ============================================================
void cortePorTiempo() {                 // Ticker: independiente del loop
  digitalWrite(RELE_PIN, RELE_OFF);
  cortePorTiempoPendiente = true;
}

void apagarRele(const char* motivo) {
  tickerSeguridad.detach();
  digitalWrite(RELE_PIN, RELE_OFF);
  bool estaba = releEncendido;
  releEncendido = false;
  motivoRele = motivo;
  publicarRiego = true;
  if (estaba) Serial.printf("[RIEGO] Relé APAGADO (%s) tras %lu s\n", motivo, (millis() - tInicioRele) / 1000UL);
  else        Serial.printf("[RIEGO] Relé ya estaba apagado (%s)\n", motivo);
}

void encenderRele(const char* motivo) {
  if (releEncendido) {
    Serial.println("[RIEGO] ON recibido con el relé ya encendido — sin cambios.");
    publicarRiego = true;
    return;
  }
  digitalWrite(RELE_PIN, RELE_ON);
  releEncendido = true;
  tInicioRele   = millis();
  motivoRele    = motivo;
  publicarRiego = true;
  tickerSeguridad.once((float)TIEMPO_MAX_APERTURA_SEG, cortePorTiempo);
  Serial.printf("[RIEGO] Relé ENCENDIDO (%s) — corte de seguridad en %d s\n", motivo, TIEMPO_MAX_APERTURA_SEG);
}

void publicarEstadoRiego() {
  if (!mqtt.connected()) return;
  unsigned long segOn = releEncendido ? (millis() - tInicioRele) / 1000UL : 0;
  String p = "{";
  p += "\"rele\":"     + String(releEncendido ? 1 : 0) + ",";
  p += "\"estado\":\"" + String(releEncendido ? "ON" : "OFF") + "\",";
  p += "\"motivo\":\"" + motivoRele + "\",";
  p += "\"segOn\":"    + String(segOn) + ",";
  p += "\"maxSeg\":"   + String(TIEMPO_MAX_APERTURA_SEG);
  p += "}";
  if (mqtt.publish(TOPIC_RIEGO_ESTADO, p.c_str(), true)) {
    publicarRiego = false;
    Serial.println("[MQTT] Estado riego → " + p);
  }
}

void vigilarConexionRiego() {
  if (!releEncendido) return;
  if (WiFi.status() != WL_CONNECTED) apagarRele("sin_wifi");
  else if (!mqtt.connected())        apagarRele("sin_mqtt");
}

void alRecibirMQTT(char* topic, byte* payload, unsigned int length) {
  String cmd;
  for (unsigned int i = 0; i < length; i++) cmd += (char)payload[i];
  cmd.trim();
  cmd.toUpperCase();
  Serial.println("[MQTT] Orden recibida en " + String(topic) + ": " + cmd);
  if (String(topic) != TOPIC_RIEGO_CMD) return;

  if (cmd == "ON" || cmd == "1" || cmd == "TRUE" || cmd == "ABRIR")          encenderRele("orden");
  else if (cmd == "OFF" || cmd == "0" || cmd == "FALSE" || cmd == "CERRAR")  apagarRele("orden");
  else Serial.println("[RIEGO] Orden no reconocida — ignorada.");
}

// ============================================================
//  RED
// ============================================================
void conectarWiFi() {
  Serial.print("[WiFi] Conectando a ");
  Serial.print(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setSleep(false);            // evita desconexiones por ahorro de energía
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  tUltIntentoWiFi = millis();

  int intentos = 0;
  while (WiFi.status() != WL_CONNECTED && intentos < 40) {
    delay(500);
    Serial.print(".");
    intentos++;
  }
  if (WiFi.status() == WL_CONNECTED)
    Serial.println("\n[WiFi] Conectado. IP: " + WiFi.localIP().toString());
  else
    Serial.println("\n[WiFi] No se pudo conectar. Se reintenta en segundo plano.");
}

void mantenerWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  if (millis() - tUltIntentoWiFi < WIFI_REINTENTO_MS) return;
  tUltIntentoWiFi = millis();
  Serial.println("[WiFi] Sin conexión — relanzando WiFi.begin()...");
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void mantenerMQTT() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (mqtt.connected()) return;
  if (millis() - tUltIntentoMQTT < MQTT_REINTENTO_MS) return;
  tUltIntentoMQTT = millis();

  Serial.print("[MQTT] Conectando al broker...");
  if (mqtt.connect(MQTT_CLIENT, TOPIC_RECEPTOR_ESTADO, 1, true, "offline")) {
    Serial.println(" conectado.");
    mqtt.publish(TOPIC_RECEPTOR_ESTADO, "online", true);
    mqtt.subscribe(TOPIC_RIEGO_CMD);
    Serial.println("[MQTT] Suscrito a " + String(TOPIC_RIEGO_CMD));
    publicarRiego = true;
  } else {
    Serial.printf(" fallo, rc=%d — reintento en 5 s.\n", mqtt.state());
  }
}

void mantenerOTA() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (!otaIniciado) {
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.onStart([]() {
      apagarRele("ota");
      Serial.println("[OTA] Actualización iniciada...");
      display.clearBuffer();
      display.setFont(u8g2_font_6x10_tf);
      display.drawStr(0, 20, "Actualizando OTA...");
      display.drawStr(0, 36, "No desconectar");
      display.sendBuffer();
    });
    ArduinoOTA.onEnd([]() { Serial.println("\n[OTA] Listo. Reiniciando."); });
    ArduinoOTA.onProgress([](unsigned int prog, unsigned int total) {
      Serial.printf("[OTA] %u%%\r", (prog * 100) / total);
    });
    ArduinoOTA.onError([](ota_error_t e) { Serial.printf("[OTA] Error %u\n", e); });
    ArduinoOTA.begin();
    otaIniciado = true;
    Serial.printf("[OTA] Listo: %s.local (%s)\n", OTA_HOSTNAME, WiFi.localIP().toString().c_str());
  }
  ArduinoOTA.handle();
}

bool publicar(const char* topic, const String& payload, bool retain = false) {
  if (!mqtt.connected()) {
    Serial.printf("[MQTT] Sin conexión — no se publicó %s\n", topic);
    return false;
  }
  bool ok = mqtt.publish(topic, payload.c_str(), retain);
  if (!ok) Serial.printf("[MQTT] Error al publicar en %s\n", topic);
  return ok;
}

// ============================================================
//  PARSEO (igual que los receptores anteriores)
// ============================================================
String extraerCampo(const String& datos, const String& etiqueta) {
  int idx = datos.indexOf(etiqueta);
  if (idx < 0) return "";
  idx += etiqueta.length();
  int fin = datos.indexOf(' ', idx);
  if (fin < 0) fin = datos.length();
  return datos.substring(idx, fin);
}

String extraerID(const String& datos) {
  int fin = datos.indexOf(' ');
  return (fin < 0) ? datos : datos.substring(0, fin);
}

int buscarNumeroSensor(const String& id) {
  for (int i = 0; i < NUM_SENSORES; i++) if (id == SENSOR_IDS[i]) return i + 1;
  return 0;
}

void procesarSuelo(const String& datos) {
  String id = extraerID(datos);
  int numSensor = buscarNumeroSensor(id);
  if (numSensor == 0) {
    Serial.println("[MQTT] ID desconocido (" + id + ") — no está en SENSOR_IDS, no se publica.");
    return;
  }
  String soil = extraerCampo(datos, "ADC:");
  String hum  = extraerCampo(datos, "H:");
  String temp = extraerCampo(datos, "T:");
  String bat  = extraerCampo(datos, "BAT:");

  String payload = "{";
  payload += "\"id\":\"" + id + "\",";
  payload += "\"soil\":" + (soil.length() > 0 ? soil : String("null")) + ",";
  payload += "\"hum\":"  + (hum.length()  > 0 ? hum  : String("null")) + ",";
  payload += "\"temp\":" + (temp.length() > 0 ? temp : String("null")) + ",";
  payload += "\"bat\":"  + (bat.length()  > 0 ? bat  : String("null")) + ",";
  payload += "\"rssi\":" + String(ultRssi, 1) + ",";
  payload += "\"snr\":"  + String(ultSnr, 1);
  payload += "}";

  ultSensor = numSensor;
  ultHum = soil.length() ? soil : String("--");

  String topic = String(TOPIC_BASE) + String(numSensor);
  if (publicar(topic.c_str(), payload)) {
    Serial.println("[MQTT] Publicado en " + topic + ":");
    Serial.println("  " + payload);
  }
}

bool checksumValido(const String& datos, String& cuerpo) {
  int ast = datos.lastIndexOf('*');
  if (ast < 0 || ast + 3 != (int)datos.length()) return false;
  if (!isxdigit(datos.charAt(ast + 1)) || !isxdigit(datos.charAt(ast + 2))) return false;
  cuerpo = datos.substring(0, ast);
  uint8_t calc = 0;
  for (int i = 0; i < (int)cuerpo.length(); i++) calc ^= (uint8_t)cuerpo.charAt(i);
  uint8_t recibido = (uint8_t)strtol(datos.substring(ast + 1).c_str(), NULL, 16);
  return calc == recibido;
}

void procesarAmbiente(const String& datos) {
  String cuerpo;
  if (!checksumValido(datos, cuerpo)) {
    Serial.println("[MB] Checksum inválido — paquete descartado.");
    return;
  }
  String mb = extraerCampo(cuerpo, "MB=");
  String payload;

  if (mb == "1") {
    String st  = extraerCampo(cuerpo, "ST=");
    String rh  = extraerCampo(cuerpo, "RH=");
    String ta  = extraerCampo(cuerpo, "TA=");
    String cnt = extraerCampo(cuerpo, "CNT=");
    if (st.length() == 0 || rh.length() == 0 || ta.length() == 0 || cnt.length() == 0) {
      Serial.println("[MB] Faltan campos — paquete descartado.");
      return;
    }
    payload  = "{\"mb\":1";
    payload += ",\"st\":"   + st;
    payload += ",\"hr\":"   + rh;
    payload += ",\"temp\":" + ta;
    payload += ",\"cnt\":"  + cnt;
    ambTxt = rh + "% " + ta + "C";
  } else if (mb == "0") {
    String fc = extraerCampo(cuerpo, "FC=");
    if (fc.length() == 0) fc = "0";
    payload  = "{\"mb\":0";
    payload += ",\"fc\":" + fc;
    ambTxt = "sin resp (" + fc + ")";
  } else {
    Serial.println("[MB] Campo MB inválido — paquete descartado.");
    return;
  }
  payload += ",\"rssi\":" + String(ultRssi, 1);
  payload += ",\"snr\":"  + String(ultSnr, 1);
  payload += "}";

  if (publicar(TOPIC_AMBIENTE, payload)) {
    Serial.println("[MQTT] Publicado en " + String(TOPIC_AMBIENTE) + ":");
    Serial.println("  " + payload);
  }
}

// ============================================================
//  LORA
// ============================================================
void leerPaquete() {
  size_t n = radio.getPacketLength();
  uint8_t buf[256];
  if (n == 0 || n > sizeof(buf)) { radio.startReceive(); return; }

  int estado = radio.readData(buf, n);
  ultRssi = radio.getRSSI();
  ultSnr  = radio.getSNR();
  radio.startReceive();

  if (estado == RADIOLIB_ERR_CRC_MISMATCH) { Serial.println("[RX] CRC inválido — ignorado."); return; }
  if (estado != RADIOLIB_ERR_NONE)         { Serial.printf("[RX] Error %d\n", estado); return; }

  // Encabezado REYAX: [dst_lo, dst_hi, src_lo, src_hi, len] + datos
  size_t ini = 0;
  if (n >= 6 && buf[4] == n - 5) {
    uint16_t dst = buf[0] | (buf[1] << 8);
    if (dst != MI_DIRECCION) {
      Serial.printf("[RX] Paquete para la dirección %u — ignorado.\n", dst);
      return;
    }
    ini = 5;
  }

  String datos;
  for (size_t i = ini; i < n; i++) {
    char c = (char)buf[i];
    if (c < 32 || c > 126) { Serial.println("[RX] Caracteres no válidos — ignorado."); return; }
    datos += c;
  }
  datos.trim();

  if (!datos.startsWith(PREFIJO)) {
    // Llegó directo de un sensor Makerfabs: el A3 lo retransmite, así que se ignora
    return;
  }
  datos = datos.substring(strlen(PREFIJO));
  pktsTotal++;

  Serial.printf("\n[RX] Paquete #%d\n", pktsTotal);
  Serial.println("  Datos : " + datos);
  Serial.printf("  RSSI  : %.0f dBm | SNR: %.1f dB\n", ultRssi, ultSnr);

  publicar(TOPIC_RAW, datos);
  if (datos.startsWith("MB ")) procesarAmbiente(datos);
  else                         procesarSuelo(datos);
}

// ============================================================
//  PANTALLA
// ============================================================
void actualizarPantalla() {
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tf);
  display.drawStr(0, 10, "RECEPTOR v6 915MHz");
  display.drawHLine(0, 13, 128);

  char l1[24], l2[24], l3[24], l4[24];
  if (ultSensor > 0) snprintf(l1, sizeof(l1), "S%d ADC:%s", ultSensor, ultHum.c_str());
  else               snprintf(l1, sizeof(l1), "Esperando paquetes");
  snprintf(l2, sizeof(l2), "Amb: %s", ambTxt.c_str());
  snprintf(l3, sizeof(l3), "RSSI:%.0f SNR:%.1f", ultRssi, ultSnr);

  const char* red = (WiFi.status() != WL_CONNECTED) ? "Sin WiFi" : (mqtt.connected() ? "MQTT OK" : "Sin MQTT");
  snprintf(l4, sizeof(l4), "%s B:%s P:%d", red, releEncendido ? "ON" : "OFF", pktsTotal);

  display.drawStr(0, 25, l1);
  display.drawStr(0, 37, l2);
  display.drawStr(0, 49, l3);
  display.drawStr(0, 63, l4);
  display.sendBuffer();
}

void pantallaError(const String& msg) {
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tf);
  display.drawStr(0, 12, "!! ERROR !!");
  display.drawHLine(0, 15, 128);
  display.drawStr(0, 38, msg.c_str());
  display.drawStr(0, 63, "Reinicia la placa");
  display.sendBuffer();
}

// ============================================================
void setup() {
  // Protección 1: relé apagado ANTES que cualquier otra cosa
  pinMode(RELE_PIN, OUTPUT);
  digitalWrite(RELE_PIN, RELE_OFF);

  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== Heltec V3 — RECEPTOR v6 + MQTT + RIEGO ===");
  Serial.printf("[RIEGO] Relé en GPIO%d, apagado. Tiempo máx: %d s\n", RELE_PIN, TIEMPO_MAX_APERTURA_SEG);

  iniciarVEXT();
  display.begin();
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tf);
  display.drawStr(0, 12, "Heltec V3 Receptor v6");
  display.drawStr(0, 28, "Conectando WiFi...");
  display.sendBuffer();

  conectarWiFi();

  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(alRecibirMQTT);
  mqtt.setSocketTimeout(5);
  tUltIntentoMQTT = millis() - MQTT_REINTENTO_MS;
  mantenerMQTT();

  int estado = radio.begin(FRECUENCIA, BANDWIDTH, SF, CODING_RATE, SYNC_WORD, POTENCIA, PREAMBLE);
  if (estado != RADIOLIB_ERR_NONE) {
    String err = "SX1262 err:" + String(estado);
    Serial.println("[ERROR] " + err);
    pantallaError(err);
    while (true) delay(1000);   // el relé ya quedó apagado
  }
  radio.setCRC(true);           // el A3 transmite con CRC activado
  radio.setDio1Action(alRecibirPaquete);
  radio.startReceive();

  Serial.println("[OK] SX1262 listo | SF9 BW125 CR4/5 Pre12 SW:0x12 CRC ON");
  Serial.println("--------------------------------------------");
  actualizarPantalla();
}

void loop() {
  // Riego: protecciones primero
  if (cortePorTiempoPendiente) {
    cortePorTiempoPendiente = false;
    apagarRele("tiempo_max");
  }
  vigilarConexionRiego();

  // Red
  mantenerWiFi();
  mantenerOTA();
  mantenerMQTT();
  mqtt.loop();
  if (publicarRiego) publicarEstadoRiego();

  // LoRa
  if (paqueteRecibido) {
    paqueteRecibido = false;
    leerPaquete();
  }

  // Pantalla cada 1 s
  if (millis() - tUltPantalla >= 1000) {
    tUltPantalla = millis();
    actualizarPantalla();
  }
}
