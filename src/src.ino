
/*************************************************************************************
/* ALl credits for some bits from https://github.com/scottwday/InverterOfThings
/* And i have trashed alot of his code, rewritten some. So thanks to him and credits to him. 
/*
/* Changes done by Daniel aka Daromer aka DIY Tech & Repairs 2020
/* https://github.com/daromer2/InverterOfThings
/* https://www.youtube.com/channel/UCI6ASwT150rendNc5ytYYrQ?
/*************************************************************************************/


//TODO:
// Clean up webpages
// Fix update timer?
// Rewrite send to MQTT part so it sends json perhaps?

// Add some code so we can set stuff on the inverters.
// MPI should have the feedToGridCorrection so it can be set via mqtt


#include <Wire.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <espconn.h>
#include <EEPROM.h>
#include <PubSubClient.h>
#include "uptime_formatter.h"
#include <ArduinoJson.h>

#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>


#include "main.h"
#include "TickCounter.h"
#include "Settings.h"
#include "inverter.h"

const char ApSsid[] = "SetSolar";


//--------------------------------- Wifi State
#define CLIENT_NOTCONNECTED 0
#define CLIENT_RECONNECT 1
#define CLIENT_CONNECTING 2
#define CLIENT_PRECONNECTED 3
#define CLIENT_CONNECTED 4
byte currentApMode = 0;
byte requestApMode = WIFI_STA;
byte clientConnectionState = CLIENT_NOTCONNECTED;
bool clientReconnect = false;

//--------------------------------- IP connection
WiFiServer server(80);
WiFiClient client;
Settings _settings;
TickCounter _tickCounter;
int WIFI_COUNT = 0;

extern QpigsMessage _qpigsMessage;
extern P003GSMessage _P003GSMessage;
extern P003PSMessage _P003PSMessage;
extern P006FPADJMessage _P006FPADJMessage;
extern String _nextCommandNeeded;
extern String _setCommand;
extern String _otherBuffer;

//---------------------- MQTT
PubSubClient mqttclient(client);

// Interface types that can be used. 
const byte MPI = 1;
const byte PCM = 0;
const byte PIP = 2;
byte crc = 0; // Default CRC is off. 
byte inverterType = MPI; //And defaults in case...
String topic = "solar/";  //Default first part of topic. We will add device ID in setup
String st = "";

unsigned long mqtttimer = 0;
extern bool _allMessagesUpdated;
extern bool _otherMessagesUpdated;
//---------- LEDS  
int Led_Red = 5;  //D1
int Led_Green = 4;  //D2

StaticJsonDocument<300> doc;  
//----------------------------------------------------------------------
void setup() 
{
 
  initHardware();
  _settings.load();
  serviceWifiMode();
  delay(2500);
  
  server.begin();
  delay(50);

  if (String(_settings._deviceType) == "MPI") {
    inverterType = MPI; }
  else if (String(_settings._deviceType) == "PIP"){
    inverterType = PIP;  
    crc = 1; // Enable CRC
  }
  else {
     inverterType = PCM; 
     crc = 1;  //Enable CRC
  }

 
  
  //dev = _settings._deviceName.c_str();
  topic = topic + String(_settings._deviceName.c_str());
  
  mqttclient.setServer(_settings._mqttServer.c_str(), _settings._mqttPort);
  mqttclient.setCallback(callback);
  
  pinMode(Led_Red, OUTPUT); 
  pinMode(Led_Green, OUTPUT); 
  digitalWrite(Led_Red, HIGH); 
  digitalWrite(Led_Green, LOW); 
  
  /*if (_settings._deviceName.length() > 0) 
    ArduinoOTA.setHostname(String("ESP-" + _settings._deviceName).c_str());
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial1.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial1.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial1.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial1.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial1.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial1.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial1.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial1.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial1.println("End Failed");
  });
  ArduinoOTA.begin();*/
}




//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------
void loop() 
{
  delay(10);

  // Make sure wifi is in the right mode
  serviceWifiMode();
  if (WiFi.status() == WL_CONNECTED) { //No use going to next step unless WIFI is up and running. 
    
    
    //ArduinoOTA.handle();  //Handle any OTA requests   DISABLED DUE TO BUG
  
    //If we have pending data to send send it first!
    sendRaw();
  
    // Comms with inverter
    serviceInverter();  // Check if we recieved data or should send data
    sendtoMQTT();  // Update data to MQTT server if we should
   
      // Check if we have something to read from MQTT 
    mqttclient.loop();
  }
  // Check if a client towards port 80 has connected
  WiFiClient client = server.available();
  if (!client) {
    return;
  }
  
  // Read the first line of the request
  String req = client.readStringUntil('\r');
  Serial1.println(req);
  client.flush();

  if (req.indexOf("/wifi") != -1) {
    serveWifiSetupPage(client);    
  } else if (req.indexOf("/aplist") != -1) {
    serveWifiApList(client);
  } else if (req.indexOf("/setap") != -1) {
    serveWifiSetAp(client, req);
  } else if (req.indexOf("/mqtt") != -1) {
    serveMqtt(client, req);
  } else if (req.indexOf("/setmqtt") != -1) {
    serveSetMqtt(client, req);
  } else if (req.indexOf("/reboot") != -1) {
    serve404(client);
    delay(100);
    ESP.restart();
  } else {
    servePage(client, req);
    //serve404(client);
  }
  client.flush();
}

void initHardware()
{
 
  delay(100);
  Wire.begin(4, 5);
  
  Serial1.begin(115200); // Debugging towards UART1
  Serial.begin(2400); // Using UART0 for comm with inverter. IE cant be connected during flashing

}

int WifiGetRssiAsQuality(int rssi)  // THis part borrowed from Tasmota code
{
  int quality = 0;

  if (rssi <= -100) {
    quality = 0;
  } else if (rssi >= -50) {
    quality = 100;
  } else {
    quality = 2 * (rssi + 100);
  }
  return quality;
}


bool sendtoMQTT() {
    if (millis() < (mqtttimer + 3000)) { 
    return false;
  }
  mqtttimer = millis();
  if (!mqttclient.connected()) {
    if (mqttclient.connect((String("ESP-" +_settings._deviceName)).c_str(), _settings._mqttUser.c_str(), _settings._mqttPassword.c_str() )) {
    
        Serial1.println(F("Reconnected to MQTT SERVER"));
        mqttclient.publish((topic + String("/Info")).c_str(), ("{\"Status\":\"Im alive!\", \"DeviceType\": \"" + _settings._deviceType + "\",\"IP \":\"" + WiFi.localIP().toString() + "\"}" ).c_str());
        mqttclient.subscribe((topic + String("/code")).c_str());
        mqttclient.subscribe((topic + String("/code")).c_str());
      } else {
        Serial1.println(F("CANT CONNECT TO MQTT"));
        Serial1.println(_settings._mqttUser.c_str());
        digitalWrite(Led_Green, LOW); 
        //delay(50);
        return false; // Exit if we couldnt connect to MQTT brooker
      }
  } 
    
    mqttclient.publish((topic + String("/uptime")).c_str(), String("{\"human\":\"" + String(uptime_formatter::getUptime()) + "\", \"seconds\":" + String(millis()/1000) + "}").c_str()    );
    mqttclient.publish((topic + String("/wifi")).c_str()  , (String("{ \"FreeRam\": ") + String(ESP.getFreeHeap()) + String(", \"rssi\": ") + String(WiFi.RSSI()) + String(", \"dbm\": ") + String(WifiGetRssiAsQuality(WiFi.RSSI())) + String("}")).c_str());  


    
    Serial1.print(F("Data sent to MQTT SERver"));
    Serial1.print(F(" - up: "));
    Serial1.println(uptime_formatter::getUptime());
    digitalWrite(Led_Green, HIGH);
    

  
  if (!_allMessagesUpdated) return false;
  
  _allMessagesUpdated = false; // Lets reset messages and process them
  
  if (inverterType == PCM) { //PCM
     mqttclient.publish((String(topic) + String("/battv")).c_str(), String(_qpigsMessage.battV).c_str());
     mqttclient.publish((String(topic) + String("/solarv")).c_str(), String(_qpigsMessage.solarV).c_str());
     mqttclient.publish((String(topic) + String("/batta")).c_str(), String(_qpigsMessage.battChargeA).c_str());
     mqttclient.publish((String(topic) + String("/wattage")).c_str(), String(_qpigsMessage.wattage).c_str());
     mqttclient.publish((String(topic) + String("/solara")).c_str(), String(_qpigsMessage.solarA).c_str());

    doc.clear();
    doc["battv"] =  _qpigsMessage.battV;
    doc["solarv"] = _qpigsMessage.solarV;
    doc["batta"] =  _qpigsMessage.battChargeA;
    doc["wattage"] =_qpigsMessage.wattage;
    doc["solara"] = _qpigsMessage.solarA;
    st = "";
    serializeJson(doc,st);
    mqttclient.publish((String(topic) + String("/status")).c_str(), st.c_str() );

     
  }
   if (inverterType == PIP) { //PIP
   
     mqttclient.publish((String(topic) + String("/GridV")).c_str(), String(_qpigsMessage.GridV).c_str());
     mqttclient.publish((String(topic) + String("/GridF")).c_str(), String(_qpigsMessage.GridF).c_str());
     mqttclient.publish((String(topic) + String("/AcV")).c_str(), String(_qpigsMessage.AcV).c_str());
     mqttclient.publish((String(topic) + String("/AcF")).c_str(), String(_qpigsMessage.AcF).c_str());
     mqttclient.publish((String(topic) + String("/AcVA")).c_str(), String(_qpigsMessage.AcVA).c_str());
     mqttclient.publish((String(topic) + String("/AcW")).c_str(), String(_qpigsMessage.AcW).c_str());
     mqttclient.publish((String(topic) + String("/Load")).c_str(), String(_qpigsMessage.Load).c_str());
     //mqttclient.publish((String(topic) + String("/BusV")).c_str(), String(_qpigsMessage.BusV).c_str());
     mqttclient.publish((String(topic) + String("/BattV")).c_str(), String(_qpigsMessage.BattV).c_str());
     mqttclient.publish((String(topic) + String("/ChgeA")).c_str(), String(_qpigsMessage.ChgeA).c_str());
     mqttclient.publish((String(topic) + String("/BattC")).c_str(), String(_qpigsMessage.BattC).c_str());
     mqttclient.publish((String(topic) + String("/Temp")).c_str(), String(_qpigsMessage.Temp).c_str());
     mqttclient.publish((String(topic) + String("/PVbattA")).c_str(), String(_qpigsMessage.PVbattA).c_str());
     mqttclient.publish((String(topic) + String("/PVV")).c_str(), String(_qpigsMessage.PVV).c_str());
     //mqttclient.publish((String(topic) + String("/BattSCC")).c_str(), String(_qpigsMessage.BattSCC).c_str());
     mqttclient.publish((String(topic) + String("/BattDisA")).c_str(), String(_qpigsMessage.BattDisA).c_str());
     mqttclient.publish((String(topic) + String("/Stat")).c_str(), String(_qpigsMessage.Stat).c_str());
     //mqttclient.publish((String(topic) + String("/BattOffs")).c_str(), String(_qpigsMessage.BattOffs).c_str());
     //mqttclient.publish((String(topic) + String("/Eeprom")).c_str(), String(_qpigsMessage.Eeprom).c_str());
     mqttclient.publish((String(topic) + String("/PVchW")).c_str(), String(_qpigsMessage.PVchW).c_str());
     //mqttclient.publish((String(topic) + String("/b10")).c_str(), String(_qpigsMessage.b10).c_str());
     //mqttclient.publish((String(topic) + String("/b9")).c_str(), String(_qpigsMessage.b9).c_str());
     //mqttclient.publish((String(topic) + String("/b8")).c_str(), String(_qpigsMessage.b8).c_str());
     //mqttclient.publish((String(topic) + String("/reserved")).c_str(), String(_qpigsMessage.reserved).c_str());

    doc.clear();
    doc["GridV"] =  _qpigsMessage.GridV;
    doc["GridF"] = _qpigsMessage.GridF;
    doc["AcV"] =  _qpigsMessage.AcV;
    doc["AcF"] =_qpigsMessage.AcF;
    doc["AcVA"] = _qpigsMessage.AcVA;
    doc["AcW"] = _qpigsMessage.AcW;
    doc["Load"] = _qpigsMessage.Load;
    //doc["BusV"] = _qpigsMessage.BusV;
    doc["BattV"] = _qpigsMessage.BattV;
    doc["ChgeA"] = _qpigsMessage.ChgeA;
    doc["BattC"] = _qpigsMessage.BattC;
    doc["Temp"] = _qpigsMessage.Temp;
    doc["PVbattA"] = _qpigsMessage.PVbattA;
    doc["PVV"] = _qpigsMessage.PVV;
   //doc["BattSCC"] = _qpigsMessage.BattSCC;
    doc["BattDisA"] = _qpigsMessage.BattDisA;
    doc["Stat"] = _qpigsMessage.Stat;
    //doc["BattOffs"] = _qpigsMessage.BattOffs;
    //doc["Eeprom"] = _qpigsMessage.Eeprom;
    doc["PVchW"] = _qpigsMessage.PVchW;
    //doc["b10"] = _qpigsMessage.b10;
    //doc["b9"] = _qpigsMessage.b9;
    //doc["b8"] = _qpigsMessage.b8;
    //doc["reserved"] = _qpigsMessage.reserved;
    st = "";
    serializeJson(doc,st);
    mqttclient.publish((String(topic) + String("/status")).c_str(), st.c_str() );
  }
  
  if (inverterType == MPI) { //IF MPI
    mqttclient.publish((String(topic) + String("/solar1w")).c_str(), String(_P003GSMessage.solarInputV1*_P003GSMessage.solarInputA1).c_str());
    mqttclient.publish((String(topic) + String("/solar2w")).c_str(), String(_P003GSMessage.solarInputV2*_P003GSMessage.solarInputA2).c_str());
    mqttclient.publish((String(topic) + String("/solarInputV1")).c_str(), String(_P003GSMessage.solarInputV1).c_str());
    mqttclient.publish((String(topic) + String("/solarInputV2")).c_str(), String(_P003GSMessage.solarInputV2).c_str());
    mqttclient.publish((String(topic) + String("/solarInputA1")).c_str(), String(_P003GSMessage.solarInputA1).c_str());
    mqttclient.publish((String(topic) + String("/solarInputA2")).c_str(), String(_P003GSMessage.solarInputA2).c_str()); 
    mqttclient.publish((String(topic) + String("/battV")).c_str(), String(_P003GSMessage.battV).c_str());
    mqttclient.publish((String(topic) + String("/battA")).c_str(), String(_P003GSMessage.battA).c_str());

    mqttclient.publish((String(topic) + String("/acInputVoltageR")).c_str(), String(_P003GSMessage.acInputVoltageR).c_str());   
    mqttclient.publish((String(topic) + String("/acInputVoltageS")).c_str(), String(_P003GSMessage.acInputVoltageS).c_str());
    mqttclient.publish((String(topic) + String("/acInputVoltageT")).c_str(), String(_P003GSMessage.acInputVoltageT).c_str());

    mqttclient.publish((String(topic) + String("/acInputCurrentR")).c_str(), String(_P003GSMessage.acInputCurrentR).c_str());   
    mqttclient.publish((String(topic) + String("/acInputCurrentS")).c_str(), String(_P003GSMessage.acInputCurrentS).c_str());
    mqttclient.publish((String(topic) + String("/acInputCurrentT")).c_str(), String(_P003GSMessage.acInputCurrentT).c_str());

    mqttclient.publish((String(topic) + String("/acOutputCurrentR")).c_str(), String(_P003GSMessage.acOutputCurrentR).c_str());   
    mqttclient.publish((String(topic) + String("/acOutputCurrentS")).c_str(), String(_P003GSMessage.acOutputCurrentS).c_str());
    mqttclient.publish((String(topic) + String("/acOutputCurrentT")).c_str(), String(_P003GSMessage.acOutputCurrentT).c_str());

    mqttclient.publish((String(topic) + String("/acWattageR")).c_str(), String(_P003PSMessage.w_r).c_str());
    mqttclient.publish((String(topic) + String("/acWattageS")).c_str(), String(_P003PSMessage.w_s).c_str());
    mqttclient.publish((String(topic) + String("/acWattageT")).c_str(), String(_P003PSMessage.w_t).c_str());
    mqttclient.publish((String(topic) + String("/acWattageTotal")).c_str(), String(_P003PSMessage.w_total).c_str());
    mqttclient.publish((String(topic) + String("/ac_output_procent")).c_str(), String(_P003PSMessage.ac_output_procent).c_str());

    mqttclient.publish((String(topic) + String("/feedingGridDirectionR")).c_str(), String(_P006FPADJMessage.feedingGridDirectionR).c_str());
    mqttclient.publish((String(topic) + String("/calibrationWattR")).c_str(), String(_P006FPADJMessage.calibrationWattR).c_str());
    mqttclient.publish((String(topic) + String("/feedingGridDirectionS")).c_str(), String(_P006FPADJMessage.feedingGridDirectionS).c_str());
    mqttclient.publish((String(topic) + String("/calibrationWattS")).c_str(), String(_P006FPADJMessage.calibrationWattS).c_str());
    mqttclient.publish((String(topic) + String("/feedingGridDirectionT")).c_str(), String(_P006FPADJMessage.feedingGridDirectionT).c_str());
    mqttclient.publish((String(topic) + String("/calibrationWattT")).c_str(), String(_P006FPADJMessage.calibrationWattT).c_str());
    
    doc.clear();
    doc["solar1w"] =  _P003GSMessage.solarInputV1*_P003GSMessage.solarInputA1;
    doc["solar2w"] =  _P003GSMessage.solarInputV2*_P003GSMessage.solarInputA2;
    doc["solarInputV1"] =    _P003GSMessage.solarInputV1;
    doc["solarInputV2"] =    _P003GSMessage.solarInputV2;
    doc["solarInputA1"] =    _P003GSMessage.solarInputA1;
    doc["solarInputA2"] =    _P003GSMessage.solarInputA2;
    doc["battV"] = _P003GSMessage.battV;
    doc["battA"] = _P003GSMessage.battA;
    doc["acInputVoltageR"] = _P003GSMessage.acInputVoltageR;
    doc["acInputVoltageS"] = _P003GSMessage.acInputVoltageS;
    doc["acInputVoltageT"] = _P003GSMessage.acInputVoltageT;
    doc["acInputCurrentR"] = _P003GSMessage.acInputCurrentR;
    doc["acInputCurrentS"] = _P003GSMessage.acInputCurrentS;
    doc["acInputCurrentT"] = _P003GSMessage.acInputCurrentT;
    doc["acOutputCurrentR"] = _P003GSMessage.acOutputCurrentR;
    doc["acOutputCurrentS"] = _P003GSMessage.acOutputCurrentS;
    doc["acOutputCurrentT"] = _P003GSMessage.acOutputCurrentT;
    doc["acWattageR"] = _P003PSMessage.w_r;
    doc["acWattageS"] = _P003PSMessage.w_s;
    doc["acWattageT"] = _P003PSMessage.w_t;
    doc["acWattageTotal"] = _P003PSMessage.w_total;
    doc["acOutputProcentage"] = _P003PSMessage.ac_output_procent;
    doc["feedingGridDirectionR"] = _P006FPADJMessage.feedingGridDirectionR;
    doc["feedingGridDirectionS"] = _P006FPADJMessage.feedingGridDirectionS;
    doc["feedingGridDirectionT"] = _P006FPADJMessage.feedingGridDirectionT;
    doc["calibrationWattR"] = _P006FPADJMessage.calibrationWattR;
    doc["calibrationWattS"] = _P006FPADJMessage.calibrationWattS;
    doc["calibrationWattT"] = _P006FPADJMessage.calibrationWattT;
    st = "";
    serializeJson(doc,st);
    mqttclient.publish((String(topic) + String("/status")).c_str(), st.c_str() );
  }

  return true;
}


// Check if we have pending raw messages to send to MQTT. Then send it. 
void sendRaw() {
  if (_otherMessagesUpdated) {
    _otherMessagesUpdated = false;
    Serial1.print("Sending other data to mqtt: ");
    Serial1.println(_otherBuffer);
    mqttclient.publish((String(topic) + String("/debug/recieved")).c_str(), String(_otherBuffer).c_str() );
  }
}
/// TESTING MQTT SEND

void callback(char* top, byte* payload, unsigned int length) {
  Serial1.println(F("Callback done"));
  if (strcmp(top,"pir1Status")==0){
    // whatever you want for this topic
  }
 
  String st ="";
  for (int i = 0; i < length; i++) {
    st += String((char)payload[i]);
  }
  
  mqttclient.publish((String(topic) + String("/debug/sent")).c_str(), String("top: " + topic + " data: " + st).c_str() );
  Serial1.print(F("Current command: "));
  Serial1.print(_nextCommandNeeded);
  Serial1.print(F(" Setting next command to : "));
  Serial1.println(st);
  _setCommand = st;
  // Add code to put the call into the queue but verify it firstly. Then send the result back to debug/new window?
}
