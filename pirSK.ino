


/*
 this sketch is in parts a tool chest for sensors

  105   wifiscan schaltbar
  106  : secrets aus include, lorien
  108  : secrets : fangorn
  109 : bme aus und AM2320 eingebaut. Letzterer ist scheinbar ein selbst-heizer
  110 : kein REST mehr
*/

#include <mqtt_kranich.h>
#include <wifi_orgon.h>

#include <Wire.h>
#include <BH1750.h>

//#include <Adafruit_Sensor.h>
//#include <Adafruit_BME280.h>

#include "AM2320.h"

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>

#include <Streaming.h>
#include <Arduino.h>

#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <i2cdetect.h>




//#define DHTPIN D4          // what digital DHT uses
//#define DHTTYPE DHT22     // DHT 22  (AM2302), AM2321



// timed loop

#define INTERVAL_1 200
#define INTERVAL_2 10000
#define INTERVAL_3 30000
#define INTERVAL_4 60000
 
unsigned long time_1 = 0;
unsigned long time_2 = 0;
unsigned long time_3 = 0;
unsigned long time_4 = 0;

const char* version = "1.11";

/************ MQTT Information  ******************/


const char* state_topic = "sensor/sk/state";
const char* set_topic   = "sensor/sk/set";
const char* dbg_topic   = "sensor/sk/dbg";
const char* scan_topic  =  "wifiscan/sensorSK";

/************ WIFI Information  ******************/

const char* host = "smarthome.intern";
const int httpPort = 8080;

const char* HOSTNAME = "paok21";



/************ REST item Information  ******************/


String luxId="3";
String itemId="SK";


String luxItem="sensor_lux_"+luxId;
String getItem="rush_lux_"+luxId;

String pirItem="PirState_"+itemId;
String pir2Item="PirState_"+itemId+"2";
String tocItem="TocState_"+itemId;

String tempItem="sensor_temperature_"+itemId;
String humItem="sensor_humidity_"+itemId;
String errItem="sensor_error_"+itemId;
String rssiItem="sensor_rssi_"+itemId;

  
String restUrl="http://smarthome.intern:8080/rest/items/";
String getUrl=restUrl+getItem+"/state";

String pubString;
String restItem;


/************ Flags & States  ******************/

bool rushLux=false;
bool showPir=false;
bool showErr=false;
bool scanne=false;

int errStatus = D0;// Digital pin D5
int pirStatus = D1;  // Digital pin D6

int pirSensor = D7;  // Digital pin D7
int pirSensor2= D6;
int tocSensor = D5;

long pirState;
long prevState;

int activated;

bool flagToc  = false;
bool flagPir  = false;
bool flagPir2 = false;

String errBme="ok";

// measurement results
long rssi =0;
float humidity, temperature, pressure, rPressure,gas,altitude_measured;
float altitude = 56; //54,06 +2m schrank
uint16_t lux =0;

/**************************** wifiscan **************************************************/

int scanId=0;

/**************************** FOR OTA **************************************************/
#define SENSORNAME "sensorSK"


/****************************************FOR JSON***************************************/
const int BUFFER_SIZE = JSON_OBJECT_SIZE(25);


// debug messages
const int numMsg=20;
int msgCount=0;
String msg="";
String arrMessages[numMsg];

WiFiClient espClient;
PubSubClient mqClient(espClient);

BH1750 lightMeter;
AM2320 th(&Wire);

//Adafruit_BME280 bme;
//Adafruit_BME680 bme;
//Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
//DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor.


void setup(){
 
  
  Serial.begin(115200);
 
  Serial << F("setup wire / I2C bus") << "\n";
  // Initialize the I2C bus (BH1750 library doesn't do this automatically)
  // On esp8266 you can select SCL and SDA pins using Wire.begin(D4, D3);
  Wire.begin(D4, D3);

  i2cdetect();
    
  

  //Serial << F("setup BH1750") << "\n";
  lightMeter.begin();


  setupWifi();
  //setupBme();
  //setup_BME680;
  setupMq();
  setupOta();


  // pinMode
  pinMode(pirSensor,  INPUT);   // declare sensor as input
  pinMode(pirSensor2, INPUT);   // declare sensor as input
  pinMode(tocSensor,  INPUT);   // declare sensor as input

  
  // interrupt für toc
  attachInterrupt(digitalPinToInterrupt(tocSensor), handleToc, RISING);

  // interrupt für pir
  attachInterrupt(digitalPinToInterrupt(pirSensor), handlePir, RISING);

  // interrupt für pir2
  attachInterrupt(digitalPinToInterrupt(pirSensor2), handlePir2, RISING);
  
  debug( String(SENSORNAME)+" "+version,true);
  
}


void loop() {
  mqClient.loop();
  ArduinoOTA.handle();
 
  timed_loop();
}


// dies als loop ruft Funktionen in definierten intervallen
void timed_loop() {
  if(millis() > time_1 + INTERVAL_1){
      time_1 = millis();

      if (!mqClient.connected()) {
        mqConnect();
      }  
      checkDebug();   
          
    }
   
    if(millis() > time_2 + INTERVAL_2){
        time_2 = millis();
       // debug("time2",true);
    }
   
    if(millis() > time_3 + INTERVAL_3){
        time_3 = millis(); 
        //debug("time3-1",true);
        //measureBme(); 
        
        measureAM2320();
        checkLight();    

        sendState();
        //debug("time3-2",true);
    }

    if(millis() > time_4 + INTERVAL_4){
        time_4 = millis();
        checkWifi();
        if (scanne) {
          scanWifi();
        }   


    }
   
 //checkPir();
 checkFlags();
}

// measure signal strength
void checkWifi(){
  rssi = WiFi.RSSI();
  //Serial << F("RSSI: ") << rssi << "\n";

}

//
//void checkPir(void)
//{
//  pirState = digitalRead(pirSensor);
//
//  if (pirState != prevState) {
//    // change detected
//    prevState=pirState;
//    if(pirState == HIGH) {
//      if (showPir){
//        digitalWrite (pirStatus, HIGH);
//      }
//      // pirState posten...
//      postRestItem(pirItem,"ON");
//
//    }
//    else {
//      digitalWrite (pirStatus, LOW);
//    }
//  }
//}

void handleToc(){
  Serial << F("Touch felt\n");
  flagToc = true;
}

void handlePir(){
  Serial << F("Pir felt\n");
  flagPir = true;
}

void handlePir2(){
  Serial << F("Pir2 felt\n");
  flagPir2 = true;
}

void checkFlags(){
  
  if (flagToc){
    //Serial << F("Toc taken\n");
    sendState();
    flagToc=false;
    //postRestItem(tocItem,"ON");
    }
  if (flagPir){
    //Serial << F("Pir taken\n");
    sendState();
    flagPir=false;
    //postRestItem(pirItem,"ON");
    }
  if (flagPir2){
    //Serial << F("Pir2 taken\n");
    sendState();
    flagPir2=false;
    //postRestItem(pir2Item,"ON");
    }
    
}

//void measureBmp(){
//  sensors_event_t event;
//  bmp.getEvent(&event);
//
//  bmp.getTemperature(&temperature);
//  pressure=event.pressure;
//  
//  float altitude = 55;
//  rPressure=(bmp.seaLevelForAltitude(altitude,event.pressure));
//
//
//}

//void measureBme(void)
//{
//  bme.takeForcedMeasurement();
//  humidity    = bme.readHumidity();
//  temperature = bme.readTemperature();
//  pressure    = bme.readPressure()/100.0F;
//  rPressure   = bme.seaLevelForAltitude(altitude, pressure);
//
//  Serial << (humidity) << F(" humidity \n");
//}

void measureAM2320(){
  switch(th.Read()) {
    case 2:
      debug(F("AM2320  CRC failed"),1);
      break;
    case 1:
      debug(F("AM2320  Sensor offline"),1);
      break;
    case 0:
      humidity    = th.Humidity;
      temperature = th.cTemp;

      break;
  }  
}

void checkLight(void)
{
  lux = lightMeter.readLightLevel();
  //Serial << (lux) << F(" lx \n");
  
}


//void setup_BME680(){
//    if (!bme.begin(0x76)) 
//  {
//    Serial.println("Could not find a valid BME680 sensor, check wiring!");
//    errBme=F("Could not find a valid BME680 sensor, check wiring!");
//    while (1);
//  }
// 
//  // Set up oversampling and filter initialization
//  bme.setTemperatureOversampling(BME680_OS_8X);
//  bme.setHumidityOversampling(BME680_OS_2X);
//  bme.setPressureOversampling(BME680_OS_4X);
//  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
//  bme.setGasHeater(320, 200); // 320*C for 150 ms
//}


//void read_BME680() 
//{
////  if (! bme.performReading()) 
////  {
////    errBme = F("Failed to perform reading :(\n");
////    Serial << errBme;
////   // return;
////  }
////  
////  if (! bme.performReading()) 
////  {
////    errBme = F("Failed to perform reading :(\n");
////    Serial << errBme;
////   // return;
////  }
////  
////  if (! bme.performReading()) 
////  {
////    errBme = F("Failed to perform reading :(\n");
////    Serial << errBme;
////   // return;
////  }
////  
////  if (! bme.performReading()) 
////  {
////    errBme = F("Failed to perform reading :(\n");
////    Serial << errBme;
////   // return;
////  }
////  
////  if (! bme.performReading()) 
////  {
////    errBme = F("Failed to perform reading :(\n");
////    Serial << errBme;
////   // return;
////  }
//
//  float rslt;
//
//  rslt=bme.readTemperature();
//  Serial << "temp: "<< rslt<<"\n";
//
//  rslt=bme.readPressure();
//  Serial << "pres: "<< rslt<<"\n";
//
//  rslt=bme.readHumidity();
//  Serial << "hum:  "<< rslt<<"\n";
//
//  rslt=bme.readGas();
//  Serial << "gas:  "<< rslt<<"\n";
////
////  rslt=bme.readAltitude();
////  Serial << "alt:  "<< rslt<<"\n";
//
//
//  
//  
//  
////  
////  
////  
////  temperature = (bme.temperature);
////  pressure    = (bme.pressure / 100.0);
////  rPressure   = seaLevelForAltitude(altitude,(bme.pressure / 100.0) );
////  humidity    = (bme.humidity);
////  gas         = (bme.gas_resistance / 1000.0);
////  altitude_measured = (bme.readAltitude(rPressure));
////  
//}

float seaLevelForAltitude(float altitude, float atmospheric)
{
    // Equation taken from BMP180 datasheet (page 17):
    //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

    // Note that using the equation from wikipedia can give bad results
    // at high altitude. See this thread for more information:
    //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

    return atmospheric / pow(1.0 - (altitude/44330.0), 5.255);
}

//
//
//void postRestItem(String itemName,String msg){
//   WiFiClient client;
//  
//  if (!client.connect(host, httpPort)) {
//    //Serial.println("connection for post on "+itemName+" failed");
//    return;
//  }
//  
//  String pubStringLength = String(msg.length(), DEC);
//  
//  // We now create a URI for the request
//  
//  String req="POST /rest/items/"+itemName+" HTTP/1.1";
//  String hst="Host: "+String(host);
//  
//  //Serial.println("Requesting POST: "+itemName+" "+msg);
//  // Send request to the server:
//  client.println(req);
//  client.println(hst);
//  client.println("Content-Type: text/plain");
//  client.println("Accept: application/json");
//  client.println("Connection: close");
//  client.print("Content-Length: ");
//  client.println(pubStringLength);
//  client.println();
//  client.print(msg);
//  client.println();
//  delay(20);
//  
//  // Read all the lines of the reply from server and print them to Serial
//  while (client.available()) {
//    String line = client.readStringUntil('\r');
//    //Serial.print(line);
//  }
//}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




/********************************** START CALLBACK*****************************************/
void callback(char* topic, byte* payload, unsigned int length) {

  StaticJsonDocument<256> root;
  deserializeJson(root, payload, length);

  const char* on_cmd = "ON";
  const char* off_cmd = "OFF";
  

 
  if (root.containsKey("scanWifi")) {
    if (strcmp(root["scanWifi"], on_cmd) == 0) {
      scanne = true;
    }
    else if (strcmp(root["scanWifi"], off_cmd) == 0) {
      scanne = false;
    }
  }
  sendState();
}


/********************************** START SEND STATE*****************************************/
void sendState() {
  //Serial.println("about to send state");
 StaticJsonDocument<BUFFER_SIZE> root;


  const char* on_cmd = "ON";
  const char* off_cmd = "OFF";


  root["pir1"] = (flagPir) ? on_cmd : off_cmd;
  root["pir2"] = (flagPir2) ? on_cmd : off_cmd;
  root["toc"]  = (flagToc) ? on_cmd : off_cmd;


  root["temp"] = temperature;
  root["hum"] = humidity;
//  root["press"] = pressure;
//  root["rPress"] = rPressure;
  root["lux"] = lux;

  root["rssi"] = rssi;  

  root["vers"] = version;
 

  char buffer[512];
  serializeJson(root, buffer);

  mqClient.publish(state_topic, buffer, true);
}



///////////////////////////////////////////////////////////////////////////////


// send a message to mq
void sendDbg(String msg){
  StaticJsonDocument<BUFFER_SIZE> doc;
 
  doc["dbg"]=msg;
  

  char buffer[512];
  size_t n = serializeJson(doc, buffer);

  mqClient.publish(dbg_topic, buffer, n);
}


// called out of timed_loop async
void checkDebug(){
  if (msgCount>0){
    
    String message = arrMessages[0];

     for (int i = 0; i < numMsg-1; i++) {
      arrMessages[i]=arrMessages[i+1];
    }
    arrMessages[numMsg-1]="";
    msgCount--;
    sendDbg(message);
  }
  
  
}

// stuff the line into an array. Another function will send it to mq later
void debug(String dbgMsg, boolean withSerial){
  Serial << "dbgMsg: " << dbgMsg <<  "\n";
  
  if (withSerial) {
    Serial.println( dbgMsg );
  }
  if (msgCount<numMsg){
    arrMessages[msgCount]=dbgMsg;
    msgCount++;
  }
  
}


/////////////////////////////////////////////////////////////////


void scanWifi(){
  WiFi.disconnect();
  delay(100);

  /*
   * pubsub mag es nicht, die ganze Liste auf einen rutsch zu posten
   * Deshalb mal ein post je netz
   * umzuordnen zu können, scanId und reportId angeben
   */
   
  scanId++;
   
  // WiFi.scanNetworks will return the number of networks found
  int netCnt = WiFi.scanNetworks();
  
  setupWifi();
  delay(50);
  if (!mqClient.connected()) {
        mqConnect();
  }  
  delay(50);    
  debug(String(netCnt)+" SSIDS erkannt",false);
  
  if (netCnt == 0) {
    Serial << F("no networks found\n");
  } else {
    //Serial << (netCnt) << F(" networks found\n");

   
    for (int i = 0; i < netCnt; ++i) {
      StaticJsonDocument<256> doc;
    
      doc["scanner"] = String(SENSORNAME); 
      doc["netCnt"] = netCnt;
      doc["scanId"] = scanId;
    
      JsonArray nets = doc.createNestedArray("nets");

      JsonObject obj = nets.createNestedObject();
      
      obj["bssid"] = WiFi.BSSIDstr(i);
      obj["ssid"] = WiFi.SSID(i);
      obj["rssi"] = WiFi.RSSI(i);
      obj["chnl"] = WiFi.channel(i);
      //obj["isHidden"] = WiFi.isHidden(i);
      
      doc["dtlId"] = i;

      char buffer[256];
      serializeJson(doc, buffer);
      mqClient.publish(scan_topic, buffer);
      
      //Serial << WiFi.SSID(i) << F(" | ") << WiFi.RSSI(i) << F(" | ") << WiFi.BSSIDstr(i) << F(" | ") << WiFi.channel(i) << F(" | ") << WiFi.isHidden(i)  << F("\n");
      
      delay(10);

    }
  }
}


/////////////////////////////////////////////////////////////////
//
//void setupBme(){
//
//  //Initialize BME280
//  bme.begin(); 
//
//  // weather monitoring
//    bme.setSampling(Adafruit_BME280::MODE_FORCED,
//                    Adafruit_BME280::SAMPLING_X1, // temperature
//                    Adafruit_BME280::SAMPLING_X1, // pressure
//                    Adafruit_BME280::SAMPLING_X1, // humidity
//                    Adafruit_BME280::FILTER_OFF   );
//                      
//    // suggested rate is 1/60Hz (1m)
//    //delayTime = 60000; // in milliseconds
//
//}

void setupWifi(){
  // Connect to WiFi

  // make sure there is no default AP set up unintended
  WiFi.mode(WIFI_STA);
  //WiFi.hostname(HOSTNAME);
  
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  msg = "WiFi connected, local IP: "+WiFi.localIP().toString();
  debug(msg,true);
  
}


void setupMq(){
  // pubsub setup
  mqClient.setServer(mqtt_server, mqtt_port);
  mqClient.setCallback(callback);
  mqConnect();  
}


void setupOta(){

  //OTA SETUP
  ArduinoOTA.setPort(OTAport);
  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(SENSORNAME);

  // No authentication by default
  ArduinoOTA.setPassword((const char *)OTApassword);

  ArduinoOTA.onStart([]() {
    debug("Starting OTA",false);
  });
  ArduinoOTA.onEnd([]() {
    debug("End OTA",false);
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    //Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) debug("OTA Auth Failed",false);
    else if (error == OTA_BEGIN_ERROR) debug("OTA Begin Failed",false);
    else if (error == OTA_CONNECT_ERROR) debug("OTA Connect Failed",false);
    else if (error == OTA_RECEIVE_ERROR) debug("OTA Receive Failed",false);
    else if (error == OTA_END_ERROR) debug("OTA End Failed",false);
  });
  ArduinoOTA.begin();
  
}




/********************************** START mosquitto *****************************************/

void mqConnect() {
  // Loop until we're reconnected
  while (!mqClient.connected()) {

    // Attempt to connect
    if (mqClient.connect(SENSORNAME, mqtt_username, mqtt_password)) {
      
      mqClient.subscribe(set_topic);      
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }

}
