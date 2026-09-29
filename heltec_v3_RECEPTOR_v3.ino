// ============================================================
//  Heltec WiFi LoRa 32 V3  —  RECEPTOR v3
//  Recibe datos del Heltec V3 Transmisor relay
//  Filtra por prefijo "HLT:" para ignorar el sensor directo
//  Envía datos a Node-RED por MQTT via WiFi
//
//  Escenario:
//    Makerfabs Sensor → Heltec Transmisor → [este nodo] → MQTT → Node-RED
//
//  Librerías: RadioLib (jgromes), U8g2 (oliver), PubSubClient (knolleary)
//  Board: "Heltec WiFi LoRa 32(V3)"
// ============================================================

#include <RadioLib.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>

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

// ----- Parámetros LoRa — deben coincidir con TRANSMISOR ----
#define FRECUENCIA   915.0
#define BANDWIDTH    125.0
#define SF           9
#define CODING_RATE  5
#define SYNC_WORD    0x12
#define POTENCIA     22
#define PREAMBLE     12

// ----- Prefijo identificador del Heltec transmisor ---------
#define PREFIJO "HLT:"

// ----- WiFi ------------------------------------------------
const char* WIFI_SSID     = "Lis";
const char* WIFI_PASSWORD = "misdatos";

// ----- MQTT ------------------------------------------------
const char* MQTT_BROKER   = "172.20.10.2";  // IP de tu PC con Node-RED
const int   MQTT_PORT     = 1883;
const char* MQTT_CLIENT   = "HeltecReceptor";

// Tópicos MQTT — un tópico por variable del sensor
const char* TOPIC_SOIL    = "invernadero/humedad_suelo";
const char* TOPIC_HUMAIR  = "invernadero/humedad_aire";
const char* TOPIC_TEMP    = "invernadero/temperatura";
const char* TOPIC_BAT     = "invernadero/bateria";
const char* TOPIC_RAW     = "invernadero/raw";  // string completo sin parsear

// ============================================================

SX1262 radio = new Module(LORA_CS, LORA_IRQ, LORA_RST, LORA_BUSY);

U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(
  U8G2_R0, OLED_RST, OLED_SCL, OLED_SDA
);

WiFiClient   espClient;
PubSubClient mqtt(espClient);

volatile bool paqueteRecibido = false;

struct Paquete {
  String datos  = "--";
  float  rssi   = 0;
  float  snr    = 0;
  int    cuenta = 0;
} ultimo;

// ============================================================

void IRAM_ATTR alRecibirPaquete() {
  paqueteRecibido = true;
}

// ------------------------------------------------------------
void iniciarVEXT() {
  pinMode(VEXT_PIN, OUTPUT);
  digitalWrite(VEXT_PIN, LOW);
  delay(100);
}

// ------------------------------------------------------------
void conectarWiFi() {
  Serial.print("[WiFi] Conectando a ");
  Serial.print(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  // Muestra en pantalla mientras conecta
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tf);
  display.drawStr(0, 12, "Conectando WiFi...");
  display.drawStr(0, 28, WIFI_SSID);
  display.sendBuffer();

  int intentos = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    intentos++;
    if (intentos > 40) {
      // Tras 20 segundos sin conectar, continúa sin WiFi
      Serial.println("\n[WiFi] No se pudo conectar. Continuando sin WiFi.");
      display.clearBuffer();
      display.setFont(u8g2_font_6x10_tf);
      display.drawStr(0, 20, "WiFi: sin conexion");
      display.drawStr(0, 36, "Solo LoRa activo");
      display.sendBuffer();
      delay(2000);
      return;
    }
  }

  Serial.println("\n[WiFi] Conectado. IP: " + WiFi.localIP().toString());

  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tf);
  display.drawStr(0, 12, "WiFi conectado!");
  String ip = WiFi.localIP().toString();
  display.drawStr(0, 28, ip.c_str());
  display.sendBuffer();
  delay(1500);
}

// ------------------------------------------------------------
void conectarMQTT() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (mqtt.connected()) return;

  Serial.print("[MQTT] Conectando al broker...");
  if (mqtt.connect(MQTT_CLIENT)) {
    Serial.println(" conectado.");
  } else {
    Serial.print(" fallo, rc=");
    Serial.println(mqtt.state());
  }
}

// ------------------------------------------------------------
// Extrae un campo del string del sensor por su etiqueta
// Ejemplo: extraerCampo("H:61.78 T:22.48", "H:") → "61.78"
String extraerCampo(String datos, String etiqueta) {
  int idx = datos.indexOf(etiqueta);
  if (idx < 0) return "";
  idx += etiqueta.length();
  int fin = datos.indexOf(' ', idx);
  if (fin < 0) fin = datos.length();
  return datos.substring(idx, fin);
}

// ------------------------------------------------------------
// Publica los datos parseados en tópicos MQTT individuales
// Formato del sensor: "ID010000 REPLY : SOIL INEDX:181 H:61.78 T:22.48 ADC:879 BAT:927"
void publicarMQTT(String datos) {
  if (!mqtt.connected()) {
    conectarMQTT();
    if (!mqtt.connected()) {
      Serial.println("[MQTT] Sin conexión — datos no enviados.");
      return;
    }
  }

  // Publica el string raw completo
  mqtt.publish(TOPIC_RAW, datos.c_str());

  // Parsea y publica cada variable por separado
  String soil = extraerCampo(datos, "INEDX:");   // humedad suelo
  String hum  = extraerCampo(datos, "H:");        // humedad aire
  String temp = extraerCampo(datos, "T:");        // temperatura
  String bat  = extraerCampo(datos, "BAT:");      // batería

  if (soil.length() > 0) mqtt.publish(TOPIC_SOIL,   soil.c_str());
  if (hum.length()  > 0) mqtt.publish(TOPIC_HUMAIR, hum.c_str());
  if (temp.length() > 0) mqtt.publish(TOPIC_TEMP,   temp.c_str());
  if (bat.length()  > 0) mqtt.publish(TOPIC_BAT,    bat.c_str());

  Serial.println("[MQTT] Publicado:");
  Serial.println("  " + String(TOPIC_SOIL)   + " → " + soil);
  Serial.println("  " + String(TOPIC_HUMAIR) + " → " + hum);
  Serial.println("  " + String(TOPIC_TEMP)   + " → " + temp);
  Serial.println("  " + String(TOPIC_BAT)    + " → " + bat);
}

// ------------------------------------------------------------
void actualizarPantalla() {
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tf);

  display.drawStr(0, 10, "RECEPTOR 915MHz");
  display.drawHLine(0, 13, 128);

  if (ultimo.cuenta == 0) {
    display.drawStr(10, 35, "Esperando paquetes");
    display.drawStr(45, 50, "...");
  } else {
    // Muestra humedad suelo y temperatura en líneas separadas
    String soil = extraerCampo(ultimo.datos, "INEDX:");
    String hum  = extraerCampo(ultimo.datos, "H:");
    String temp = extraerCampo(ultimo.datos, "T:");

    char linSoil[22]; snprintf(linSoil, sizeof(linSoil), "Suelo: %s%%", soil.c_str());
    char linHum[22];  snprintf(linHum,  sizeof(linHum),  "Hum: %s%%  T:%sC", hum.c_str(), temp.c_str());

    display.drawStr(0, 24, linSoil);
    display.drawStr(0, 36, linHum);

    char bufSig[22];
    snprintf(bufSig, sizeof(bufSig), "RSSI:%.0fdBm SNR:%.1fdB",
             ultimo.rssi, ultimo.snr);
    display.drawStr(0, 48, bufSig);

    // WiFi y contador
    String wifiStr = (WiFi.status() == WL_CONNECTED) ? "WiFi OK" : "Sin WiFi";
    char bufCnt[22];
    snprintf(bufCnt, sizeof(bufCnt), "%s  Pkts:%d",
             wifiStr.c_str(), ultimo.cuenta);
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

// ============================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("=== Heltec V3 — RECEPTOR v3 + MQTT ===");

  iniciarVEXT();

  display.begin();
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tf);
  display.drawStr(0, 12, "Heltec V3 Receptor");
  display.drawStr(0, 28, "Iniciando...");
  display.sendBuffer();
  Serial.println("[OLED] Pantalla iniciada.");

  // WiFi
  conectarWiFi();

  // MQTT
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  conectarMQTT();

  // SX1262
  int estado = radio.begin(
    FRECUENCIA, BANDWIDTH, SF, CODING_RATE,
    SYNC_WORD, POTENCIA, PREAMBLE
  );

  if (estado != RADIOLIB_ERR_NONE) {
    String err = "SX1262 err:" + String(estado);
    Serial.println("[ERROR] " + err);
    pantallaError(err);
    while (true) delay(1000);
  }

  radio.setCRC(0);
  Serial.println("[OK] SX1262 listo.");

  radio.setDio1Action(alRecibirPaquete);
  radio.startReceive();

  actualizarPantalla();

  Serial.println("Escuchando | SF9 BW125 CR4/5 Pre12 SW:0x12");
  Serial.println("--------------------------------------------");
}

// ============================================================
void loop() {
  // Mantiene conexión MQTT activa
  if (WiFi.status() == WL_CONNECTED && !mqtt.connected()) {
    conectarMQTT();
  }
  mqtt.loop();

  if (paqueteRecibido) {
    paqueteRecibido = false;

    String datos;
    int estado = radio.readData(datos);

    if (estado == RADIOLIB_ERR_NONE) {
      datos.trim();

      // Ignora paquetes vacíos
      if (datos.length() == 0) {
        Serial.println("[RX] Paquete vacío — ignorado.");
        radio.startReceive();
        return;
      }

      // Ignora paquetes que no vengan del Heltec transmisor
      if (!datos.startsWith(PREFIJO)) {
        Serial.println("[RX] Sin prefijo HLT: — ignorado (sensor directo).");
        radio.startReceive();
        return;
      }

      // Quita el prefijo "HLT:" antes de procesar
      datos = datos.substring(strlen(PREFIJO));

      ultimo.rssi   = radio.getRSSI();
      ultimo.snr    = radio.getSNR();
      ultimo.datos  = datos;
      ultimo.cuenta++;

      Serial.println("\n[RX] Paquete #" + String(ultimo.cuenta));
      Serial.println("  Datos : " + datos);
      Serial.print  ("  RSSI  : "); Serial.print(ultimo.rssi); Serial.println(" dBm");
      Serial.print  ("  SNR   : "); Serial.print(ultimo.snr);  Serial.println(" dB");

      // Publica en MQTT
      publicarMQTT(datos);

      actualizarPantalla();

    } else if (estado == RADIOLIB_ERR_CRC_MISMATCH) {
      Serial.println("[RX] CRC mismatch — ignorado.");
    } else {
      Serial.print("[RX] Error: ");
      Serial.println(estado);
    }

    radio.startReceive();
  }
}
