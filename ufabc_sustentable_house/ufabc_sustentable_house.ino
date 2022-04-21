#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#define lightOnTimeSeconds 10

//Constantes
const bool DEBUG = false;
const float LUX_THRESHOLD = 20.0;

// GPIOs
const int HUMIDIFIER_PIN = 12;
const int LIGHT_PIN = 15;
const int ESP_LED_BUILTIN = 2;
const int MOTION_SENSOR_PIN = 13;

// Mudanças no estado de dispositivos Digitais
// -1 Representa que não houve mudança no estado
//  1 Representa que o dispositivo estava desligado e foi ligado
//  0 Representa que o dispositivo estava ligado e foi desligado
const int STATE_NO_CHANGES = -1;
const int STATE_TURNED_ON = 1;
const int STATE_TURNED_OFF = 0;


// Conexecoes
const char* ssid = "ssid";
const char* password = "password";
const char* mqtt_broker = "mqtt.eclipseprojects.io";
WiFiClient wifiClient;
PubSubClient client(wifiClient);

// Sensores I2C
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

//Variaveis para valores dos sensores
int luxValue;
int lightStateChanges;
int humidityValue = 60;
int tempValue = 24;

// Tempos
unsigned long mqttPublishPreviousMillis = 0;

// Intervalos de Tasks
const long MQTT_PUBLISH_DATA_INTERVAL = 2000;


// Timer: Auxiliary variables
unsigned long now = millis();
unsigned long motionSensorLastTrigger = 0;
boolean motionSensorStartTimer = false;

// Interrupcao de deteccao de movimento
ICACHE_RAM_ATTR void detectsMovement() {
  Serial.println("MOTION DETECTED!!!");

  if(luxValue < LUX_THRESHOLD) {
    digitalWrite(LIGHT_PIN, HIGH);
    motionSensorStartTimer = true;
    motionSensorLastTrigger = millis();
  }

  
}

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

  // Wifi
  wifi_connect();
  client.setServer(mqtt_broker,1883);

  // Hello World
  Serial.println("UFABC: Sistemas Microprocessados 2022.1");
}

// estabelece conexao com rede wifi 2.4Ghz
void wifi_connect(){
  delay(10);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
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
  Serial.println("");
  Serial.println("WiFi connected");
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

int getLuxValueFromTSL2561(void) {
  if (DEBUG) Serial.print("[TSL_2661] Lendo valor do sensor...  ");
  sensors_event_t event;
  tsl.getEvent(&event);
  if (DEBUG) Serial.print(event.light);
  if (DEBUG) Serial.println(" lux");
  return event.light;
}

void checkLightConditions(void) {
  luxValue = getLuxValueFromTSL2561();
  int lightDigitalValue = digitalRead(LIGHT_PIN);
  
  unsigned long currentMillis = millis();
  if(motionSensorStartTimer && (currentMillis - motionSensorLastTrigger > (lightOnTimeSeconds*1000))) {
    Serial.println("Motion stopped...");
    digitalWrite(LIGHT_PIN, LOW);
    motionSensorStartTimer = false;
  }
}

void checkHumidityConditions(void) {
  // TODO
}

void checkTemperatureConditions(void) {
  // TODO
}

void publishMQTTData(){

  unsigned long currentMillis = millis();
  
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

  // Tasks
  checkLightConditions();
  checkHumidityConditions();
  checkTemperatureConditions();

  // Send data to MQTT
  publishMQTTData();

  delay(100);
}
