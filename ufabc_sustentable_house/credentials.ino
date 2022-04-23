
// Funcao que faz a leitura da memoria procurando dados credenciais da rede wifi
// Caso nao seja encontrado, sera requisitado o preenchimento dos dados via serial
// Apos preenchido esses dados serao gravados na memoria interna, para numa futura reinicizalicao do microcontrolador possa ser lida diretamente da memoria.
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
