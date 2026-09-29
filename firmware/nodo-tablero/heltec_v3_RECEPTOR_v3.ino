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
const char* MQTT_BROKER   = "172.20.10.2";  // IP de tu PC con Node-RED (hotspot personal)
const int   MQTT_PORT     = 1883;
const char* MQTT_CLIENT   = "HeltecReceptor";

// ----- Mapeo de ID de hardware → número de sensor (1 a 6) --
// El ID llega en el paquete crudo, ej: "ID010000 REPLY : ..."
// Completar aquí los 6 IDs reales según se vayan confirmando
// en el Monitor Serial (ver campo "Datos" al recibir cada paquete).
#define NUM_SENSORES 6
const char* SENSOR_IDS[NUM_SENSORES] = {
  "ID010000",  // Sensor 1
  "ID020000",  // Sensor 2
  "ID030000",  // Sensor 3
  "ID040000",  // Sensor 4
  "ID050000",  // Sensor 5
  "ID060000"   // Sensor 6
};

// Tópico base — se arma dinámicamente como invernadero/sensor/N
// (el "/" antes de N es necesario para que Node-RED pueda suscribirse
// a los 6 con un solo comodín: invernadero/sensor/+)
const char* TOPIC_BASE    = "invernadero/sensor/";
const char* TOPIC_RAW     = "invernadero/raw";  // string completo sin parsear (debug)

// ============================================================

SX1262 radio = new Module(LORA_CS, LORA_IRQ, LORA_RST, LORA_BUSY);

U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(
  U8G2_R0, OLED_RST, OLED_SCL, OLED_SDA
);

WiFiClient   espClient;
PubSubClient mqtt(espClient);

volatile bool paqueteRecibido = false;

struct Paquete {
  String datos     = "--";
  float  rssi      = 0;
  float  snr       = 0;
  int    cuenta    = 0;
  int    numSensor = 0;   // 1..6, o 0 si el ID no está mapeado
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
// Extrae el ID de hardware, que va al inicio del paquete
// Ejemplo: "ID010000 REPLY : SOIL..." → "ID010000"
String extraerID(String datos) {
  int fin = datos.indexOf(' ');
  if (fin < 0) return datos;
  return datos.substring(0, fin);
}

// ------------------------------------------------------------
// Busca a qué número de sensor (1..NUM_SENSORES) corresponde un ID
// Devuelve 0 si el ID no está en la tabla SENSOR_IDS
int buscarNumeroSensor(String id) {
  for (int i = 0; i < NUM_SENSORES; i++) {
    if (id == SENSOR_IDS[i]) return i + 1;
  }
  return 0;
}

// ------------------------------------------------------------
// Publica los datos parseados en un tópico JSON individual por sensor
// Formato del sensor: "ID010000 REPLY : SOIL INEDX:181 H:61.78 T:22.48 ADC:879 BAT:927"
// Tópico resultante: invernadero/sensor/1, invernadero/sensor/2, ...
// Payload: {"id":"ID010000","soil":181,"hum":61.78,"temp":22.48,"bat":927,"rssi":-42,"snr":9.5}
void publicarMQTT(String datos) {
  if (!mqtt.connected()) {
    conectarMQTT();
    if (!mqtt.connected()) {
      Serial.println("[MQTT] Sin conexión — datos no enviados.");
      return;
    }
  }

  // Publica el string raw completo (útil para depurar en Node-RED)
  mqtt.publish(TOPIC_RAW, datos.c_str());

  // Identifica de cuál de los 6 sensores viene el paquete
  String id = extraerID(datos);
  int numSensor = buscarNumeroSensor(id);

  if (numSensor == 0) {
    Serial.println("[MQTT] ID desconocido (" + id + ") — no está en SENSOR_IDS, no se publica.");
    return;
  }

  // Parsea las variables del sensor
  String soil = extraerCampo(datos, "INEDX:");   // humedad suelo
  String hum  = extraerCampo(datos, "H:");        // humedad aire
  String temp = extraerCampo(datos, "T:");        // temperatura
  String bat  = extraerCampo(datos, "BAT:");      // batería

  // Arma el JSON a mano (evita depender de la librería ArduinoJson)
  String payload = "{";
  payload += "\"id\":\"" + id + "\",";
  payload += "\"soil\":" + (soil.length() > 0 ? soil : "null") + ",";
  payload += "\"hum\":"  + (hum.length()  > 0 ? hum  : "null") + ",";
  payload += "\"temp\":" + (temp.length() > 0 ? temp : "null") + ",";
  payload += "\"bat\":"  + (bat.length()  > 0 ? bat  : "null") + ",";
  payload += "\"rssi\":" + String(ultimo.rssi, 1) + ",";
  payload += "\"snr\":"  + String(ultimo.snr, 1);
  payload += "}";

  String topic = String(TOPIC_BASE) + String(numSensor);
  mqtt.publish(topic.c_str(), payload.c_str());

  Serial.println("[MQTT] Publicado en " + topic + ":");
  Serial.println("  " + payload);
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

    char linSoil[22];
    if (ultimo.numSensor > 0) {
      snprintf(linSoil, sizeof(linSoil), "S%d Suelo: %s%%", ultimo.numSensor, soil.c_str());
    } else {
      snprintf(linSoil, sizeof(linSoil), "ID desconocido!");
    }
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

      ultimo.rssi      = radio.getRSSI();
      ultimo.snr       = radio.getSNR();
      ultimo.datos     = datos;
      ultimo.numSensor = buscarNumeroSensor(extraerID(datos));
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
