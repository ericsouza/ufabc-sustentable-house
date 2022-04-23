// estabelece conexao com rede wifi 2.4Ghz
void wifi_connect(){
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
  Serial.println("\n\n***************************************\n\n");
  Serial.print("Connected to Wifi: ");
  Serial.println(wifiSsid);
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  digitalWrite(BUILTIN_LED_PIN, LOW);
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
