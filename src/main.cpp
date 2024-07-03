#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "certificate.h"

#define CERT mqtt_broker_cert
#define MSG_BUFFER_SIZE (50)
#define RELE1 D7
#define RELE2 D6
#define RELE3 D5
#define RELE4 D3
#define MIKRIK1 D1
#define MIKRIK2 D2
#define sub1 "open"
#define sub2 "close"
#define mqttTopicOut "Garden/Gates/State"
#define mqttTopicIn "Garden/Gates/Control"
//--------------------------------------
// config (edit here before compiling)
//--------------------------------------
//#define MQTT_TLS // uncomment this define to enable TLS transport
//#define MQTT_TLS_VERIFY // uncomment this define to enable broker certificate verification
const char* ssid = "QWww";
const char* password = "9498498499";
const char* mqtt_server = "192.168.0.138"; // eg. your-demo.cedalo.cloud or 192.168.1.11
const uint16_t mqtt_server_port = 1883; // or 8883 most common for tls transport
const char* mqttUser = "qaplok";
const char* mqttPassword = "qar2tip3";



String State = "Unknow";
String State_now = "Unknow";
bool StateM1, StateM2;
static unsigned long lastStateRead = 0;
// Топик LWT, одновременно это топик статуса устройства:
// - в рабочем состоянии в этом топике будет сообщение "online"
// - когда устройство по какой-либо причине отключится от сервера, сервер примерно через 60 секунд опубликует LWT "offline"
const char* mqttTopicDeviceStatus    = "Garden/Gates/Status";
const char* mqttDeviceStatusOn       = "online";    
const char* mqttDeviceStatusOff      = "offline";
const int   mqttDeviceStatusQos      = 1;
const bool  mqttDeviceStatusRetained = true; 


// Текстовое отображение для состояния входов
const char* mqttInputStatusClosed       = "Closed";
const char* mqttInputStatusOpened      = "Opened";
const char* mqttInputStatusHalfOpened      = "Half_Opened";
const bool  mqttInputStatusRetained  = false;

//--------------------------------------
// globals
//--------------------------------------
#ifdef MQTT_TLS
  WiFiClientSecure wifiClient;
#else
  WiFiClient wifiClient;
#endif
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
PubSubClient mqttClient(wifiClient);

//--------------------------------------
// function setup_wifi called once
//--------------------------------------
void setup_wifi() {
  digitalWrite(RELE1, HIGH);
  digitalWrite(RELE2, HIGH);
  digitalWrite(RELE3, HIGH);
  digitalWrite(RELE4, HIGH);
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  timeClient.begin();

#ifdef MQTT_TLS
  #ifdef MQTT_TLS_VERIFY
    X509List *cert = new X509List(CERT);
    wifiClient.setTrustAnchors(cert);
  #else
    wifiClient.setInsecure();
  #endif
#endif

  Serial.println("WiFi connected");
}

void open_gates(){
  if ((State == "Closed") || (State = "Half_Opened")){
    digitalWrite(RELE1, LOW);
    digitalWrite(RELE3, LOW);
    digitalWrite(RELE2, HIGH);
    digitalWrite(RELE4, HIGH);
  }
  State_now = "Open_now";
}

void close_gates(){
  if ((State == "Opened") || (State = "Half_Opened")){
    digitalWrite(RELE2, LOW);
    digitalWrite(RELE4, LOW);
    digitalWrite(RELE1, HIGH);
    digitalWrite(RELE3, HIGH);
  }
  State_now = "Close_now";
}

void read_state(){
  if (State == "Opened"){
    mqttClient.publish(mqttTopicOut, mqttInputStatusClosed, mqttInputStatusRetained);
  }
  if (State == "Closed"){
    mqttClient.publish(mqttTopicOut, mqttInputStatusOpened, mqttInputStatusRetained);
  }
  if (State == "Half_Opened"){
    mqttClient.publish(mqttTopicOut, mqttInputStatusHalfOpened, mqttInputStatusRetained);
  }
}
//--------------------------------------
// function callback called everytime 
// if a mqtt message arrives from the broker
//--------------------------------------
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived on topic: '");
  Serial.print(topic);
  Serial.print("' with payload: ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  String _payload;
  for (unsigned int i = 0; i < length; i++) {
    _payload += String((char)payload[i]);
  };
  _payload.toLowerCase();
  _payload.trim();

  if (_payload.equals(sub1))
  {
    open_gates();
  }
  if (_payload.equals(sub2))
  {
    close_gates();
  }
  
}

//--------------------------------------
// function connect called to (re)connect
// to the broker
//--------------------------------------
void connect() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    String mqttClientId = "";
    if (mqttClient.connect(mqttClientId.c_str(), mqttUser, mqttPassword,
        mqttTopicDeviceStatus, mqttDeviceStatusQos, mqttDeviceStatusRetained, mqttDeviceStatusOff)) {
      Serial.println("connected");
      mqttClient.publish(mqttTopicDeviceStatus, mqttDeviceStatusOn, mqttDeviceStatusRetained);
      mqttClient.subscribe(mqttTopicIn);
      
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" will try again in 5 seconds");
      digitalWrite(RELE1, HIGH);
      digitalWrite(RELE2, HIGH);
      digitalWrite(RELE3, HIGH);
      digitalWrite(RELE4, HIGH);
      delay(5000);
    }
  }
}

//--------------------------------------
// main arduino setup fuction called once
//--------------------------------------
void setup() {
  pinMode(MIKRIK1, INPUT_PULLUP); 
  pinMode(MIKRIK2, INPUT_PULLUP);
  pinMode(RELE1, OUTPUT);
  pinMode(RELE2, OUTPUT);
  pinMode(RELE3, OUTPUT);
  pinMode(RELE4, OUTPUT);
  digitalWrite(RELE1, HIGH);
  digitalWrite(RELE2, HIGH);
  digitalWrite(RELE3, HIGH);
  digitalWrite(RELE4, HIGH);

  Serial.begin(115200);
  setup_wifi();
  mqttClient.setServer(mqtt_server, mqtt_server_port);
  mqttClient.setCallback(callback);
  StateM1 = digitalRead(MIKRIK1);
  Serial.println("Mikrik1");
  Serial.println(StateM1);
  StateM2 = digitalRead(MIKRIK2);
  Serial.println("Mikrik2");
  Serial.println(StateM2);
  if (StateM1 == 0) {State = "Closed";}
  if (StateM2 == 0) {State = "Opened";}
  if ((StateM2 == 1) & (StateM1 == 1)) {State = "Half_Opened";}
  if ((StateM2 == 0) & (StateM1 == 0)) {State = "Error_mikrik";}
  Serial.println(State);
}

//--------------------------------------
// main arduino loop fuction called periodically
//--------------------------------------
void loop() {
  if (!mqttClient.connected()) {
    connect();
  }
  //Serial.println(lastStateRead);
  mqttClient.loop();
  timeClient.update();
  
  if ((millis() - lastStateRead) >= 2000) {
    lastStateRead = millis();
    StateM1 = digitalRead(MIKRIK1);
    StateM2 = digitalRead(MIKRIK2);
    if (StateM1 == 0) {State = "Closed";}
    if (StateM2 == 0) {State = "Opened";}
    if ((StateM2 == 0) & (StateM1 == 0)) {State = "Error_mikrik";}
    if ((StateM2 == 1) & (StateM1 == 1)) {State = "Half_Opened";}
    read_state();
  }
  
  if (State_now == "Break"){
    digitalWrite(RELE2, LOW);
    digitalWrite(RELE1, LOW);
    digitalWrite(RELE3, HIGH);
    digitalWrite(RELE4, HIGH);
    Serial.println("Break");
    delay(3000);
    digitalWrite(RELE2, HIGH);
    digitalWrite(RELE1, HIGH);
    State_now = "Wait";
    Serial.println("Wait");
    State_now = "Wait";
  }
  
  if (State_now == "Open_now"){
    Serial.println(State_now);
    StateM1 = digitalRead(MIKRIK1); 
    if (StateM1 == 0){
      State_now = "Break";
      //State = "Opened";
      digitalWrite(RELE1, HIGH);
      digitalWrite(RELE3, HIGH);
    }
  }

  if (State_now == "Close_now"){
    Serial.println(State_now);
    StateM1 = digitalRead(MIKRIK2); 
    if (StateM2 == 0){
      State_now = "Break";
      //State = "Closed";
      digitalWrite(RELE2, HIGH);
      digitalWrite(RELE4, HIGH);
    }
  }

}
