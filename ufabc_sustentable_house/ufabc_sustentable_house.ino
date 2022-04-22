#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <EEPROM.h>

#define lightOnTimeSeconds 10

//Constantes
const bool DEBUG = false;
const float LUX_THRESHOLD = 20.0;
const long MQTT_PUBLISH_DATA_INTERVAL = 2000;

// GPIOs
const int ESP_LED_BUILTIN = 2;
const int HUMIDIFIER_PIN = 12;
const int LIGHT_PIN = 15;
const int MOTION_SENSOR_PIN = 13;

// Network
char wifiSsid[32] = "";
char wifiPassword[32] = "";
const char* mqtt_broker = "mqtt.eclipseprojects.io";
WiFiClient wifiClient;
PubSubClient client(wifiClient);

// Sensores I2C
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

//Variaveis para valores dos sensores
int luxValue;
int lightStateChanges;
int humidityValue = 59;
int tempValue = 24;

// Tempo: Variaveis auxiliares de tempo
unsigned long mqttPublishPreviousMillis = 0;
unsigned long currentMillis = millis();
unsigned long motionSensorLastTrigger = 0;
boolean motionSensorStartTimer = false;


void setup(void) 
{
  // GPIOs
  pinMode(ESP_LED_BUILTIN, OUTPUT);
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

  // Hello World
  Serial.println("\n\nUFABC: Sistemas Microprocessados 2022.1\n\n\n");
  delay(500);
  
  // Wifi
  wifi_connect();
  client.setServer(mqtt_broker,1883);

}

void readWifiConnectionData(void) {
  EEPROM.begin(512);
  EEPROM.get(0, wifiSsid);
  EEPROM.get(0+sizeof(wifiSsid), wifiPassword);
  char ok[2+1];
  EEPROM.get(0+sizeof(wifiSsid)+sizeof(wifiPassword), ok);
  EEPROM.end();
  if (String(ok) != String("OK")) {
    wifiSsid[0] = 0;
    wifiPassword[0] = 0;
  }
  
  if(strlen(wifiPassword)==0) {
    Serial.println("\n\nDados de Wifi nao encontrados em memoria\nPor favor digite nome e senha do seu Wifi\n\n");
    Serial.print("Digite o nome do Wifi: ");
    while(Serial.available() == 0) {}
    String ssid = Serial.readString();
    ssid.trim();
    ssid.toCharArray(wifiSsid, 32);
    Serial.println();
    Serial.print("Digite a senha do Wifi: ");
    while(Serial.available() == 0) {}
    String password = Serial.readString();
    password.trim();
    password.toCharArray(wifiPassword, 32);
    Serial.println();

    EEPROM.begin(512);
    EEPROM.put(0, wifiSsid);
    EEPROM.put(0+sizeof(wifiSsid), wifiPassword);
    char ok[2+1] = "OK";
    EEPROM.put(0+sizeof(wifiSsid)+sizeof(wifiPassword), ok);
    EEPROM.commit();
    EEPROM.end();
  } else {
    Serial.println();
    Serial.println("Credenciais de Wifi Encontradas:");
    Serial.println(wifiSsid);
    Serial.println("********");
  }
}

// estabelece conexao com rede wifi 2.4Ghz
void wifi_connect(){
  readWifiConnectionData();
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSsid, wifiPassword);
  Serial.print("Conectando ao Wifi");
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(ESP_LED_BUILTIN, LOW);
    delay(200);
    digitalWrite(ESP_LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(ESP_LED_BUILTIN, LOW);
    delay(200);
    digitalWrite(ESP_LED_BUILTIN, HIGH);
    Serial.print(".");
  }
  Serial.println("\n\n***************************************\n\n");
  Serial.print("Connected to Wifi: ");
  Serial.println(wifiSsid);
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  digitalWrite(ESP_LED_BUILTIN, LOW);
  Serial.println("***************************************\n\n");
}

// Restabelece conexão com broker mqtt
void reconnect_MQTT(){
  while (!client.connected()) {
    delay(500);
    client.connect("UFABC_bsUWuxIuMqCa");
    Serial.println("Reconnecting MQTT...");
  }
}

// Interrupcao de deteccao de movimento
ICACHE_RAM_ATTR void detectsMovement() {
  if (DEBUG) Serial.println("Movimento Detectado!");

  if(luxValue < LUX_THRESHOLD) {
    digitalWrite(LIGHT_PIN, HIGH);
    motionSensorStartTimer = true;
    motionSensorLastTrigger = millis();
  }
 
}

int getLuxValueFromTSL2561(void) {
  if (DEBUG) Serial.print("[TSL_2661] Lendo valor do sensor...  ");
  sensors_event_t event;
  tsl.getEvent(&event);
  if (DEBUG) Serial.print(event.light);
  if (DEBUG) Serial.println(" lux");
  return event.light;
}

void checkLightConditions(unsigned long currentMillis) {
  luxValue = getLuxValueFromTSL2561();
  int lightDigitalValue = digitalRead(LIGHT_PIN);
  
  if(motionSensorStartTimer && (currentMillis - motionSensorLastTrigger > (lightOnTimeSeconds*1000))) {
    if (DEBUG) Serial.println("Desligando Luz...");
    digitalWrite(LIGHT_PIN, LOW);
    motionSensorStartTimer = false;
  }
}

void checkHumidityConditions(unsigned long currentMillis) {
  if(luxValue < 20) {
    digitalWrite(HUMIDIFIER_PIN, LOW);
  } else {
    digitalWrite(HUMIDIFIER_PIN, HIGH);
  }
}

void checkTemperatureConditions(unsigned long currentMillis) {
  // TODO
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
    digitalWrite(ESP_LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(ESP_LED_BUILTIN, LOW);
    delay(50);
    digitalWrite(ESP_LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(ESP_LED_BUILTIN, LOW);
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
  checkLightConditions(currentMillis);
  checkHumidityConditions(currentMillis);
  checkTemperatureConditions(currentMillis);

  // Send data to MQTT
  publishMQTTData(currentMillis);

  delay(100);
}
