// Inclusao das bibliotecas
#include <Wire.h> // I2C
#include <Adafruit_Sensor.h> // Sensores da Adafruit
#include <Adafruit_TSL2561_U.h> // Sensor de Luminosidade
#include <EEPROM.h> // Biblioteca para manipular memoria interna do ESP8266
#include <DHT.h> // Sensor de Umidade e Temperatura
#include <ESP8266WiFi.h> // Biblioteca para Comunicacao Network via Wifi
#include <PubSubClient.h> // Biblioteca do Procotolo MQTT

// Constantes
#define DHTTYPE DHT11 // Tipo do sensor de umidade e temperatura, nesse caso é o DHT11
#define LIGHT_ON_TIME 10000 // Tempo em milisegundos que a luz permanecera ligada apos ultima deteccao de movimento
#define HUMIDITY_THRESHOLD 80.0f // Limiar para ligar Umidificador, abaixo desse valor ele sera ligado
#define LUX_THRESHOLD 20.0f // Limiar para Ligar a luz, abaixo desse valor o LED/Lampada sera ligada
#define MQTT_PUBLISH_DATA_INTERVAL 2000 // Intervalor para publicacao dos dados dos sensores para MQTT
#define DEBUG 1 // Habilita mensagens de debug na serial

// GPIOs
#define BUILTIN_LED_PIN 2
#define HUMIDIFIER_PIN 12
#define LIGHT_PIN 15
#define MOTION_SENSOR_PIN 13
#define DHTPIN 14

// Network
char wifiSsid[32] = "";
char wifiPassword[32] = "";
const char* mqtt_broker = "mqtt.eclipseprojects.io";
WiFiClient wifiClient;
PubSubClient client(wifiClient);

// Sensores I2C
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
DHT dht(DHTPIN, DHTTYPE); 

//Variaveis para guardar os valores dos sensores
int luxValue;
int lightStateChanges;
float humidityValue = 0.0;
float tempValue = 0.0;
boolean motionSensorStarted = false;

// Tempo: Variaveis auxiliares de tempo
unsigned long currentMillis = millis();
unsigned long mqttPublishPreviousMillis = 0; // Tempo da ultima publicacao de dados no MQTT
unsigned long motionSensorLastTrigger = 0; // Tempo da ultima deteccao de movimento

void setup(void) 
{
  // GPIOs
  pinMode(BUILTIN_LED_PIN, OUTPUT);
  pinMode(HUMIDIFIER_PIN, OUTPUT);
  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(MOTION_SENSOR_PIN, INPUT_PULLUP);

  // GPIOs inicializacao
  digitalWrite(HUMIDIFIER_PIN, HIGH);
  digitalWrite(LIGHT_PIN, LOW);
  
  // Serial
  Serial.begin(9600);

  // Interrupcao
  attachInterrupt(digitalPinToInterrupt(MOTION_SENSOR_PIN), detectsMovement, RISING);
 
  // Sensores
  if(!tsl.begin())
  {
    Serial.print("Sensor TSL2561 não detectado");
    while(1);
  }
  tsl.enableAutoRange(true);
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);
  
  dht.begin();

  // Hello World
  Serial.println("\n\nUFABC: Sistemas Microprocessados 2022.1\n\n\n");
  delay(500);
  
  // Wifi
  wifi_connect();
  client.setServer(mqtt_broker,1883);

}

// Interrupcao de deteccao de movimento
ICACHE_RAM_ATTR void detectsMovement() {
  if (DEBUG) Serial.println("Movimento Detectado!");

  if(luxValue < LUX_THRESHOLD) {
    digitalWrite(LIGHT_PIN, HIGH);
    motionSensorStarted = true;
    motionSensorLastTrigger = millis();
  }
}

void checkLightConditionsTask(unsigned long currentMillis) {
  if (DEBUG) Serial.print("[TSL_2661] Lendo valor do sensor...  ");
  sensors_event_t event;
  tsl.getEvent(&event);
  if (DEBUG) Serial.print(event.light);
  if (DEBUG) Serial.println(" lux");
  luxValue = event.light;
  
  int lightDigitalValue = digitalRead(LIGHT_PIN);
  
  if(motionSensorStarted && (currentMillis - motionSensorLastTrigger > (LIGHT_ON_TIME))) {
    if (DEBUG) Serial.println("Desligando Luz...");
    digitalWrite(LIGHT_PIN, LOW);
    motionSensorStarted = false;
  }
}

void checkHumidityConditionsTask(unsigned long currentMillis) {
  humidityValue = dht.readHumidity();
  if (DEBUG) Serial.print("[DHT11] Humidade: ");
  if (DEBUG) Serial.println(humidityValue);
  if(humidityValue < HUMIDITY_THRESHOLD) {
    digitalWrite(HUMIDIFIER_PIN, LOW);
  } else {
    digitalWrite(HUMIDIFIER_PIN, HIGH);
  }
}

void checkTemperatureConditionsTask(unsigned long currentMillis) {
  tempValue = dht.readTemperature();
  if (DEBUG) Serial.print("[DHT11] Temperatura: ");
  if (DEBUG) Serial.println(tempValue);
}

void publishMQTTData(unsigned long currentMillis){
  
  if((currentMillis - mqttPublishPreviousMillis) >= MQTT_PUBLISH_DATA_INTERVAL) {
    mqttPublishPreviousMillis = currentMillis;

    int lightState = digitalRead(LIGHT_PIN);
    int humidifierState = digitalRead(HUMIDIFIER_PIN);
    
    // Informacoes sobre iluminacao
    client.publish("ufabc_sustentable_house/lux_value", String(luxValue).c_str(), false);
    client.publish("ufabc_sustentable_house/light_state", String(lightState).c_str(), false);
  
    // Umidade e Temperatura
    client.publish("ufabc_sustentable_house/humidity_value", String(humidityValue).c_str(), false);
    client.publish("ufabc_sustentable_house/humidifier_state", String(humidifierState).c_str(), false);
    client.publish("ufabc_sustentable_house/temp_value", String(tempValue).c_str(), false);
  
    // Piscar led interno indicando envio de dados via MQTT
    digitalWrite(BUILTIN_LED_PIN, HIGH);
    delay(50);
    digitalWrite(BUILTIN_LED_PIN, LOW);
    delay(50);
    digitalWrite(BUILTIN_LED_PIN, HIGH);
    delay(50);
    digitalWrite(BUILTIN_LED_PIN, LOW);
  }
}

void loop(void) 
{
  // Garante que conexao MQTT sempre esta estabelecida
  if (!client.connected()) {
    reconnect_MQTT();
  }

  currentMillis = millis();
  
  // Tasks
  checkLightConditionsTask(currentMillis);
  checkHumidityConditionsTask(currentMillis);
  checkTemperatureConditionsTask(currentMillis);

  // Send data to MQTT
  publishMQTTData(currentMillis);

  delay(1000);
}
