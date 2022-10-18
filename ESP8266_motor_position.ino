//// to check how to implement the rotation value with the of the TOPIC

//// ESP32 TTGO1
//// Node MCU ESP12-E
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoMqttClient.h>
//include <SPI.h>
#include <TFT_eSPI.h> // Hardware-specific library

#define TFT_GREY          0x5AEB  // New colour
#define PINFEEDBACKYELLOW 32      // encoder pulse A
#define PINFEEDBACKGREEN  33      // encoder pulse B
#define IN2               13      // H-bridge: run motor forward
#define IN1               15      // H-bridge: run motor backward

//// Motor definition 
int encoder_r   = 0;
int encoder_f   = 0;
int position_pv = 0;
int position_sv = 0;
int m_direction = 0;
int rotation_value;

//// TFT screen definition
TFT_eSPI tft = TFT_eSPI();            // Invoke custom library
TFT_eSprite spr = TFT_eSprite(&tft);  // Sprite object

//// MQTT definition
const char* mqttBroker = "broker.hivemq.com"; // public broker
int         port        = 1883;               // port of the borker
const char* TOPIC       = "spectacle";        // Topic to define in sender and receiver

WiFiClient espClient;
PubSubClient client(espClient);
MqttClient mqttClient(espClient);

//const char* googleApiKey = "AIzaSyBQQxS1vNoqATuEfB1aSvIQjlgTbU0BNxk";
char* ssid = "Oreilledeveau";
const char* passwd = "GEFVTALT";


// receive MQTT message
void onMqttMessage(int messageSize) {
  Serial.println("Received a message with topic '");   // we received a message, print out the topic and contents
  Serial.print(mqttClient.messageTopic());
  Serial.print("', length ");
  Serial.print(messageSize);
  Serial.println(" bytes:");

  // use the Stream interface to print the contents
  while (mqttClient.available()) {
    Serial.print((char)mqttClient.read());
  }
  Serial.println();
  rotation_value = mqttClient.read();
  position_sv = (1024 - rotation_value)/10; //unit: % (e.g. 0% ~ 0dg; 100% ~ 360dg)
  position_pv = ((encoder_r+encoder_f)/2); //unit: %; encoder pulse per revolution: 200ppr
  
  if(position_pv-position_sv < 0){
      runMotor();
    }
    else{
      if(position_pv-position_sv > 0){
        // run motor in opposite direction
      }
      else{
        //stop motor if position between +0 ~ -0
        stopMotor();
      }
    }
}

int stateMotor;
int lastStateMotor;
int counter = 0;

void setup() {
    Serial.begin(115200);
    
    //// declaration for position control
    pinMode(PINFEEDBACKYELLOW, INPUT_PULLUP);
    pinMode(PINFEEDBACKGREEN, INPUT_PULLUP);
    pinMode(IN2,OUTPUT); //IN2
    pinMode(IN1,OUTPUT); //IN1
    attachInterrupt(digitalPinToInterrupt(PINFEEDBACKYELLOW), detect_a_r, RISING);
    //lastStateMotor = digitalRead(PINFEEDBACKYELLOW);
    
    stopMotor();

    // Connect to WPA/WPA2 network
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

    mqttClient.onMessage(onMqttMessage);
    mqttClient.subscribe(TOPIC);
    
    tft.init();
    tft.setRotation(1); // landscape
   
    int xw = tft.width()/2;   // xw, yh is middle of screen
    int yh = tft.height()/2;
    tft.setPivot(xw+50, yh);     // Set pivot to middle of TFT screen
    // Create the Sprite
   
    spr.setPivot(xw, yh);      // Set pivot relative to top left corner of Sprite
   
    // Swap the colour byte order when rendering
    tft.setSwapBytes(true);
}
/*
void initMotor(){
  while(!digitalRead(PINFEEDBACKYELLOW)){
    analogWrite(PINFORWARDMOTOR, 125); 
    Serial.println("OK"); 
  }  
} 
*/
void runMotor(){
  digitalWrite(IN2, HIGH); 
  digitalWrite(IN1, LOW); 
}

void stopMotor(){
  digitalWrite(IN2, LOW); 
  digitalWrite(IN1, LOW); 
}

void detect_a_r() {
  m_direction = digitalRead(PINFEEDBACKGREEN); //read direction of motor
  if(!m_direction){
    encoder_r += 1;   //increasing encoder at forward run
  }
  else{
    encoder_r += -1;  //decreasing encoder at backward run
  }
  attachInterrupt(digitalPinToInterrupt(PINFEEDBACKYELLOW), detect_a_f, FALLING); //change interrupt to Falling edge
}

void detect_a_f() {
  m_direction = digitalRead(pin_b); //read direction of motor
  if(m_direction){
    encoder_f += 1; //increasing encoder at forward run
  }
  else{
    encoder_f += -1; //decreasing encoder at backward run
  }
  attachInterrupt(digitalPinToInterrupt(PINFEEDBACKGREEN), detect_a_r, RISING);  //change interrupt to Rising edge
}

void loop() {
  //delay(1000);
  mqttClient.poll();
  client.loop();
}
