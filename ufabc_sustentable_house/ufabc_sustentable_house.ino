#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
 

//Constantes
const bool DEBUG = false;
const float LUX_THRESHOLD = 20.0;
const int LIGHT_PIN = 2;
const int STATE_NO_CHANGES = -1;
const int STATE_TURNED_ON = 1;
const int STATE_TURNED_OFF = 0;

Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

void setup(void) 
{
  pinMode(LIGHT_PIN, OUTPUT);
  Serial.begin(9600);
  Serial.println("Light Sensor Test"); 
  Serial.println("");
 
  /* Inicializacao do sensor de Luminosidade */
  if(!tsl.begin())
  {
    Serial.print("Sensor TSL2561 não detectado");
    while(1);
  }
 
  tsl.enableAutoRange(true);
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);
 
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
  int state_changes = STATE_NO_CHANGES;
  int luxValue = getLuxValueFromTSL2561();
  int lightDigitalValue = digitalRead(LIGHT_PIN);

  if (luxValue < LUX_THRESHOLD) {
    if(lightDigitalValue != LOW) {
      Serial.println("turning ON light");
      digitalWrite(LIGHT_PIN, LOW);
      state_changes = STATE_TURNED_ON;
    }
  } else {
    if(lightDigitalValue != HIGH) {
      Serial.println("turning OFF light");
      digitalWrite(LIGHT_PIN, HIGH);
      state_changes = STATE_TURNED_OFF;
    }
  }
  sendLightConditionData(luxValue, state_changes);
}

void loop(void) 
{      
  checkLightConditions();

  delay(500);
}
 
