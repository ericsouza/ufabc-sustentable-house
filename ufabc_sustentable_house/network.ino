// estabelece conexao com rede wifi 2.4Ghz
void wifiConnect(){
  readWifiConnectionData();
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSsid, wifiPassword);
  Serial.print("Conectando ao Wifi");
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(BUILTIN_LED_PIN, LOW);
    delay(200);
    digitalWrite(BUILTIN_LED_PIN, HIGH);
    delay(200);
    digitalWrite(BUILTIN_LED_PIN, LOW);
    delay(200);
    digitalWrite(BUILTIN_LED_PIN, HIGH);
    Serial.print(".");
  }
  Serial.println("\n\n***************************************");
  Serial.print("Conectado ao Wifi: ");
  Serial.println(wifiSsid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  digitalWrite(BUILTIN_LED_PIN, LOW);
  Serial.println("***************************************\n");
}

// Restabelece conexão com broker mqtt
void reconnectMQTT(){
  while (!client.connected()) {
    delay(500);
    client.connect("UFABC_bsUWuxIuMqCa");
    Serial.println("Reconectando ao broker MQTT...");
  }
}
