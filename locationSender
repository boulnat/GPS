
//LilyGo
//DOIT ESP32 DEVKIT V1
//ESP32 TTGO1
//Node MCU ESP12-E
#include <Arduino.h>
#include <ESP8266WiFi.h>
//#include <WiFi.h>
#include <WifiLocation.h>
#include <PubSubClient.h>
#include <ArduinoMqttClient.h>

///////////////////////////////////
//           To set              //
///////////////////////////////////

const char* mqttBroker = "broker.hivemq.com";
//const char broker[] = "test.mosquitto.org";
const char topic[]  = "spectacle";
//const char ssid[] = "telenet-6F8E53A";
//const char passwd[] = "rehTmts8ehnc";
const char ssid[] = "Mathieu (2)";
const char passwd[] = "mathieuu";
const float motorLon = 0.00;
const float motorLat = 0.00;

///////////////////////////////////

//Server to publish the informaiton
int        port     = 1883;
const char* googleApiKey = "AIzaSyBQQxS1vNoqATuEfB1aSvIQjlgTbU0BNxk";

WiFiClient espClient;
PubSubClient client(espClient);
MqttClient mqttClient(espClient);
WifiLocation location (googleApiKey);

// function to calculate angle 
float bearing(float lat,float lon,float lat2,float lon2){

    float teta1 = radians(lat);
    float teta2 = radians(lat2);
    float delta1 = radians(lat2-lat);
    float delta2 = radians(lon2-lon);

    //==================Heading Formula Calculation================//

    float y = sin(delta2) * cos(teta2);
    float x = cos(teta1)*sin(teta2) - sin(teta1)*cos(teta2)*cos(delta2);
    float brng = atan2(y,x);
    brng = degrees(brng);// radians to degrees
    brng = ( ((int)brng + 360) % 360 ); 

    Serial.print("Heading GPS: ");
    Serial.println(brng);

    return brng;
}

void setup() {
    Serial.begin(115200);
    // Connect to WPA/WPA2 network
    //WiFi.mode(WIFI_STA);
    while (WiFi.begin(ssid, passwd) != WL_CONNECTED) {
        Serial.print("Attempting to connect to WPA SSID: ");
        Serial.println(ssid);
        delay(5000);
    }
    Serial.println ("Connected");


    if(!mqttClient.connect(mqttBroker,port)){
      Serial.println(mqttClient.connectError());
      while(1);
    }

}

void loop() {
  //if (!client.connected()) {
    //reconnect();
  //}
  //location_t loc = location.getGeoFromWiFi();
  
  mqttClient.poll();
  mqttClient.beginMessage(topic);
  //mqttClient.println("Location: " + String (loc.lat, 7) + "," + String (loc.lon, 7));
  mqttClient.println(bearing(40.76, -73.984, 38.89, -77.032)); //Test result should be around 233.00 wich is W-S
  mqttClient.endMessage();

  delay(10000);
}
