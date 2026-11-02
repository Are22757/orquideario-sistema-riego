/*
 * Esclavo Modbus RTU — Transmisor T/HR (Arduino Nano + AHT10 + MAX485)
 * Proyecto Orquideario UVG
 *
 * Conexiones:
 *   MAX485: VCC→5V  GND→GND  RO→D10  DI→D11  DE+RE→D2
 *   AHT10:  VIN→5V  GND→GND  SDA→A4  SCL→A5
 *
 * Funciones soportadas: 0x03 (holding) y 0x04 (input), mismo mapa.
 * Mapa de registros:
 *   0 : Humedad relativa x10   (ej. 653 = 65.3 %HR)
 *   1 : Temperatura x10        (con signo, ej. 241 = 24.1 °C)
 *   2 : Estado del sensor      (1 = AHT10 OK, 0 = sin sensor / simulado)
 *   3 : Contador de lecturas
 */
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>

// ===== Configuración =====
#define SLAVE_ID    1
#define BAUD        9600
#define PIN_RX      10      // ← RO del MAX485
#define PIN_TX      11      // → DI del MAX485
#define PIN_DE      2       // → DE + RE
#define T_LECTURA   2000    // ms entre lecturas del sensor
#define T_SILENCIO  4000UL  // µs: > 3.5 caracteres a 9600 bps = fin de trama
#define DEBUG       1

SoftwareSerial rs485(PIN_RX, PIN_TX);
Adafruit_AHTX0 aht;
bool sensorOK = false;

const uint8_t N_REGS = 4;
uint16_t regs[N_REGS] = {0, 0, 0, 0};

uint8_t frame[64];
uint8_t len = 0;
unsigned long tUltimoByte = 0;
unsigned long tLectura = 0;

// ---------- CRC16 Modbus ----------
uint16_t crc16(const uint8_t *buf, uint8_t n) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < n; i++) {
    crc ^= buf[i];
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
  }
  return crc;
}

// ---------- Envío con control de dirección ----------
void enviar(uint8_t *buf, uint8_t n) {
  uint16_t crc = crc16(buf, n);
  buf[n++] = crc & 0xFF;          // CRC: byte bajo primero
  buf[n++] = crc >> 8;
  digitalWrite(PIN_DE, HIGH);     // modo transmisión
  delayMicroseconds(100);
  rs485.write(buf, n);            // bloqueante: termina tras el último bit
  delayMicroseconds(100);
  digitalWrite(PIN_DE, LOW);      // vuelve a recepción
  while (rs485.available()) rs485.read();   // descarta eco/ruido
}

void excepcion(uint8_t fn, uint8_t codigo) {
  uint8_t r[5] = {SLAVE_ID, (uint8_t)(fn | 0x80), codigo};
  enviar(r, 3);
#if DEBUG
  Serial.print(F("  -> Excepcion ")); Serial.println(codigo);
#endif
}

// ---------- Procesamiento de la trama recibida ----------
void procesarTrama() {
  if (len < 8) return;                          // muy corta
  if (frame[0] != SLAVE_ID) return;             // no es para este esclavo
  uint16_t crcRx = frame[len - 2] | (frame[len - 1] << 8);
  if (crc16(frame, len - 2) != crcRx) {         // trama corrupta: se ignora
#if DEBUG
    Serial.println(F("  CRC invalido, trama ignorada"));
#endif
    return;
  }
  uint8_t  fn   = frame[1];
  uint16_t dir  = (frame[2] << 8) | frame[3];
  uint16_t cant = (frame[4] << 8) | frame[5];

  if (fn != 0x03 && fn != 0x04) { excepcion(fn, 0x01); return; }       // función ilegal
  if (cant == 0 || cant > N_REGS || (uint32_t)dir + cant > N_REGS) {
    excepcion(fn, 0x02); return;                                        // dirección ilegal
  }

  uint8_t r[5 + 2 * N_REGS];
  r[0] = SLAVE_ID; r[1] = fn; r[2] = cant * 2;
  for (uint16_t i = 0; i < cant; i++) {
    r[3 + 2 * i] = regs[dir + i] >> 8;
    r[4 + 2 * i] = regs[dir + i] & 0xFF;
  }
  enviar(r, 3 + cant * 2);
#if DEBUG
  Serial.print(F("  Respondida fn 0x0")); Serial.print(fn, HEX);
  Serial.print(F(", regs ")); Serial.print(dir); Serial.print(F("-"));
  Serial.println(dir + cant - 1);
#endif
}

// ---------- Lectura del sensor ----------
void leerSensor() {
  float h, t;
  sensors_event_t eh, et;
  if (!sensorOK) sensorOK = aht.begin();        // reintenta si no estaba
  if (sensorOK && aht.getEvent(&eh, &et)) {
    h = eh.relative_humidity;
    t = et.temperature;
    regs[2] = 1;
  } else {                                      // modo prueba sin sensor
    sensorOK = false;
    h = 60.0 + (regs[3] % 20);
    t = 24.0 + (regs[3] % 10) / 10.0;
    regs[2] = 0;
  }
  regs[0] = (uint16_t)lround(h * 10);
  regs[1] = (uint16_t)(int16_t)lround(t * 10);
  regs[3]++;
#if DEBUG
  Serial.print(regs[2] ? F("AHT10  ") : F("SIMUL  "));
  Serial.print(F("HR=")); Serial.print(h, 1);
  Serial.print(F("%  T=")); Serial.print(t, 1);
  Serial.println(F("C"));
#endif
}

void setup() {
  pinMode(PIN_DE, OUTPUT);
  digitalWrite(PIN_DE, LOW);      // arranca escuchando
  Serial.begin(115200);
  rs485.begin(BAUD);
  Wire.begin();
  sensorOK = aht.begin();
  Serial.println(F("Esclavo Modbus RTU listo"));
  Serial.print(F("ID=")); Serial.print(SLAVE_ID);
  Serial.print(F("  ")); Serial.print(BAUD); Serial.println(F(" bps 8N1"));
  Serial.println(sensorOK ? F("AHT10 detectado") : F("AHT10 NO detectado -> valores simulados"));
  leerSensor();
}

void loop() {
  // 1) Recepción byte a byte
  while (rs485.available()) {
    uint8_t b = rs485.read();
    if (len < sizeof(frame)) frame[len++] = b;
    tUltimoByte = micros();
  }
  // 2) Fin de trama por silencio en el bus
  if (len > 0 && (micros() - tUltimoByte) > T_SILENCIO) {
    procesarTrama();
    len = 0;
  }
  // 3) Lectura periódica del sensor (solo si no hay trama a medias)
  if (len == 0 && millis() - tLectura >= T_LECTURA) {
    tLectura = millis();
    leerSensor();
  }
}
