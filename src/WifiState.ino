
//----------------------------------------------------------------------
// Configure wifi as access point to allow client config
void setupWifiAp()
{
  WiFi.mode(WIFI_AP);
  WiFi.disconnect();
  WiFi.softAP(ApSsid);
}

//----------------------------------------------------------------------
void setupWifiStation()
{
  WiFi.disconnect();
  delay(20);
  if (_settings._wifiSsid.length() == 0)
  {
    Serial1.println(F("No client SSID set, switching to AP"));
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ApSsid);
  }
  else
  {
    Serial1.print(F("Connecting to "));
    Serial1.print(_settings._wifiSsid);
    Serial1.print(":");
    Serial1.println(_settings._wifiPass);
    WiFi.mode(WIFI_STA);
    if (_settings._deviceName.length() > 0)
      WiFi.hostname("ESP-" + _settings._deviceName);
    WiFi.begin(_settings._wifiSsid.c_str(), _settings._wifiPass.c_str());
  }
}

//----------------------------------------------------------------------
void serviceWifiMode()
{
  if (clientReconnect)
  {
    WiFi.disconnect();
    delay(10);
    clientReconnect = false;
    currentApMode = CLIENT_NOTCONNECTED;
  }
  
  if (currentApMode != requestApMode)
  {
    if (requestApMode == WIFI_AP)
    {
      Serial1.println("Access Point Mode");
      setupWifiAp();             
      currentApMode = WIFI_AP;
    }

    if (requestApMode == WIFI_STA)
    {
      Serial1.println("Station Mode");
      setupWifiStation();             
      currentApMode = WIFI_STA;
      clientConnectionState = CLIENT_CONNECTING;
    }
  }  

  if (clientConnectionState == CLIENT_CONNECTING)
  {    
    Serial1.print(F("c:"));
    Serial1.println(WIFI_COUNT);
    WIFI_COUNT++;
    delay(40); // Else it will restart way to quickly.
    if (WIFI_COUNT > 500) { 
      WIFI_COUNT=0;
      ESP.restart();
    }
    if (WiFi.status() == WL_CONNECTED)
    {
      clientConnectionState = CLIENT_CONNECTED;
      WIFI_COUNT = 0;
      Serial1.print(F("IP address: "));
      Serial1.println(WiFi.localIP());
    }
  }
}
