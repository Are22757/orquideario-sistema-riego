# Sistema de riego automatizado - Orquideario UVG

Sistema de control de riego automatizado para el Orquideario del Jardín Botánico de la Universidad del Valle de Guatemala. Proyecto de graduación de Ingeniería Electrónica, implementado como prototipo a escala de laboratorio.

## Descripción

El sistema mide la humedad del sustrato con seis sensores inalámbricos LoRa y activa el riego cuando la humedad promedio baja del límite configurado. También mide la temperatura y la humedad del ambiente, que solo generan alertas y no activan el riego.

Funcionamiento general:

1. Los sensores de humedad del sustrato envían sus lecturas por LoRa al nodo de adquisición.
2. El nodo de adquisición lee el sensor ambiental por Modbus RTU sobre RS485 y reenvía todos los datos por LoRa al tablero de automatización.
3. El tablero de automatización publica los datos por MQTT hacia Node-RED y acciona el relé de riego según las órdenes que recibe.
4. Node-RED ejecuta la lógica de riego, muestra el dashboard, envía alertas por Telegram y genera reportes en PDF y Excel por correo.

El sistema tiene modo automático y modo manual. El tablero incluye un temporizador de seguridad local que cierra la válvula si permanece abierta demasiado tiempo, aunque se pierda la comunicación con Node-RED.

## Estructura del repositorio

- firmware/nodo-interno: firmware del nodo de adquisición (Heltec V3)
  - transmisor_relay_v5.ino: versión final
  - heltec_v3_TRANSMISOR_v3.ino: versión anterior
- firmware/nodo-tablero: firmware del tablero de automatización (Heltec V3)
  - heltec_v3_RECEPTOR_v6.ino: versión final
  - heltec_v3_RECEPTOR_v3.ino: versión anterior
- firmware/sensor-ambiental: firmware del sensor ambiental (Arduino Nano)
  - EsclavoNano_AHT10Sensor.ino: esclavo Modbus RTU con sensor AHT10 y módulo MAX485
- node-red-flows: flujo de Node-RED
  - flujo-orquideario_full.json: lógica de riego, dashboard, alertas y reportes

## Hardware

- 6 sensores de humedad del sustrato LoRa (Makerfabs)
- 2 placas Heltec WiFi LoRa 32 V3 (nodo de adquisición y tablero de automatización)
- Arduino Nano con sensor AHT10 y módulos MAX485 (sensor ambiental por RS485)
- Módulo de relé para el accionamiento del riego

En el diseño final el relé controla una electroválvula. En el prototipo de laboratorio se usa una bomba en su lugar.

## Librerías de Arduino

- RadioLib
- U8g2
- ModbusMaster
- PubSubClient
- Adafruit AHTX0

## Node-RED

Importar el archivo node-red-flows/flujo-orquideario_full.json.

Módulos necesarios:

- node-red-dashboard
- node-red-node-ui-table
- node-red-contrib-telegrambot
- node-red-node-email
- pdfkit y exceljs (usados en el nodo que genera los reportes)

## Configuración

Antes de cargar el firmware del tablero de automatización, completar en heltec_v3_RECEPTOR_v6.ino:

- WIFI_SSID: nombre de la red WiFi
- WIFI_PASSWORD: contraseña de la red
- MQTT_BROKER: dirección IP de la computadora donde corre Node-RED

En Node-RED se deben configurar el token del bot y el chat ID de Telegram, y las credenciales del correo en el nodo de envío. Estas credenciales no se incluyen en el repositorio.

## Autora

Lisbeth Arévalo
Ingeniería Electrónica, Universidad del Valle de Guatemala, 2026
