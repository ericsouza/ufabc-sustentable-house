#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

//Constantes
const bool DEBUG = false;
const float LUX_THRESHOLD = 20.0;

// GPIOs
const int HUMIDIFER_PIN = 13;
const int ESP_LED_BUILTIN = 2;

// Mudanças no estado de dispositivos Digitais
// -1 Representa que não houve mudança no estado
//  1 Representa que o dispositivo estava desligado e foi ligado
//  0 Representa que o dispositivo estava ligado e foi desligado
const int STATE_NO_CHANGES = -1;
const int STATE_TURNED_ON = 1;
const int STATE_TURNED_OFF = 0;


// Conexecoes
const char* ssid = "ssid do wifi";
const char* password = "senha do wifi";
const char* mqtt_broker = "mqtt.eclipseprojects.io";
WiFiClient wifiClient;
PubSubClient client(wifiClient);

// Sensores I2C
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

//Variaveis para valores dos sensores
int luxValue;
int lightStateChanges;

// Tempos
unsigned long mqttPublishPreviousMillis = 0;

// Intervalos de Tasks
const long MQTT_PUBLISH_DATA_INTERVAL = 2000;


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
// Restabelece conexão com broker mqtt ao inicar ou caso tenha caido
void reconnect_MQTT(){
  while (!client.connected()) {
    delay(500);
    client.connect("UFABC_bsUWuxIuMqCa");
    Serial.println("Reconnecting MQTT...");
  }
}

void setup(void) 
{
  // GPIOs
  pinMode(ESP_LED_BUILTIN, OUTPUT);
  pinMode(HUMIDIFER_PIN, OUTPUT);

  // GPIOs inicializacao
  digitalWrite(HUMIDIFER_PIN, HIGH);
  
  // Serial
  Serial.begin(9600);
 
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


int getLuxValueFromTSL2561(void) {
  if (DEBUG) Serial.print("[TSL_2661] Lendo valor do sensor...  ");
  sensors_event_t event;
  tsl.getEvent(&event);
  if (DEBUG) Serial.print(event.light);
  if (DEBUG) Serial.println(" lux");
  return event.light;
}

void sendLightConditionData(int lux, int state_changes) {
  // int lux, estado antigo, estado atual
}

void checkLightConditions(void) {
  lightStateChanges = STATE_NO_CHANGES;
  luxValue = getLuxValueFromTSL2561();
  int lightDigitalValue = digitalRead(HUMIDIFER_PIN);

  if (luxValue < LUX_THRESHOLD) {
    if(lightDigitalValue != LOW) {
      Serial.println("turning ON light");
      digitalWrite(HUMIDIFER_PIN, LOW);
      lightStateChanges = STATE_TURNED_ON;
    }
  } else {
    if(lightDigitalValue != HIGH) {
      Serial.println("turning OFF light");
      digitalWrite(HUMIDIFER_PIN, HIGH);
      lightStateChanges = STATE_TURNED_OFF;
    }
  }
}

void checkHumidityConditions(void) {
  
}

void publishMQTTData(){

  unsigned long currentMillis = millis();
  
  if((currentMillis - mqttPublishPreviousMillis) >= MQTT_PUBLISH_DATA_INTERVAL) {
    mqttPublishPreviousMillis = currentMillis;
    // The true parameter persist information in topic
    client.publish("ufabc_sustentable_house/lux", String(luxValue).c_str(), false);
  
    // Umidade
    client.publish("ufabc_sustentable_house/humidifier_state", String(digitalRead(HUMIDIFER_PIN)).c_str(), false);
  
    // Shows publication success in the MQTT topic blinking internal led
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

  // Send data to MQTT
  publishMQTTData();

  delay(100);
}
 
