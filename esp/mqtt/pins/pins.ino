#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <FS.h>
#include <PubSubClient.h>

#include "dht11.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

void append(String& s, char* from, int len)
{
  while(len-- > 0)
  {
    s += *(from++);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// WIFI

String WiFiSSID;
String WiFiPassword;
String WiFiHostname;

WiFiClient wifiClient;

void wifiConnect()
{
  WiFi.begin(WiFiSSID.c_str(), WiFiPassword.c_str());
  if( WiFiHostname != "" )
  {
    WiFi.hostname(WiFiHostname.c_str());
  }
  
  Serial.print("Connecting to WIFI");
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_BUILTIN,0);
    delay(500);
    digitalWrite(LED_BUILTIN,1);
    Serial.print(".");
  }
  Serial.println("OK!");
  Serial.print("IP ");
  Serial.print(WiFi.localIP());
  Serial.print(" HOST ");
  Serial.print(WiFi.hostname());
  Serial.println();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// MQTT

String MqttServer{};
int    MqttPort = 1883; 
String MqttUser;
String MqttPassword;

PubSubClient mqttClient(wifiClient);

void mqttPublish(const char* subTopic, const char* msg)
{
  String t("esp/");
  t += WiFi.macAddress();
  t += '/';
  t += subTopic;
  mqttClient.publish(t.c_str(), msg);
}

void mqttPublish(const char* subTopic, const String& msg)
{
  mqttPublish(subTopic, msg.c_str());
}

void mqttSubscribeCallback(const char* topic, byte* payload, unsigned int length)
{
  Serial.print(topic);
  Serial.print(" PLen=");
  Serial.println(length);
  String msg;
  append(msg,(char*)payload,length);
  Serial.println(msg);
  
  if( msg == "restart" )
  {
    Serial.println("restarting!");
    ESP.restart();
  }
  
  if(length > 1 && (msg[0]=='d' || msg[0]=='a')) // [a|d]<n>=<v> or [a|d]<n>
  {
    auto i = 1;
    auto pin = 0;
    while(i < length && isdigit(msg[i]))
    {
      pin *= 10;
      pin += msg[i] -'0';
      i++;
    }
    if(i < length && msg[i]=='=')
    {
      ++i;      
      auto val = 0;
      while(i < length && isdigit(msg[i]))
      {
        val *= 10;
        val += msg[i] -'0';
        i++;
      }
      Serial.println(String(msg[0])+String(pin,DEC)+" <= "+String(val,DEC));
      pinMode(pin,OUTPUT);
      if(msg[0]=='d') digitalWrite(pin, val); else analogWrite(pin, val);
    }
    else
    {
      pinMode(pin,INPUT);
    }
    String ret(msg[0]);
    ret += String(pin,DEC);
    ret += "=" + String((msg[0]=='d' ? digitalRead : analogRead)(pin),DEC);
    Serial.println(ret);
    mqttPublish("ret", ret);
  }
}


void mqttConnect()
{
  mqttClient.setServer(MqttServer.c_str(), MqttPort);
  mqttClient.setCallback(mqttSubscribeCallback);
  
  while(!mqttClient.connected()) 
  {
    Serial.print("Connecting to MQTT...");
    if (mqttClient.connect("ESP8266Client", MqttUser.c_str(), MqttPassword.c_str() )) 
    {
      Serial.println("OK!");
    } 
    else 
    {
      Serial.print("failed, state=");
      Serial.println(mqttClient.state());
      delay(500);
    }
  }
  // publish to "esp/new" the MAC address (which is static and unique) to check in
  mqttClient.publish("esp/new", WiFi.macAddress().c_str());
  // subscribe to /esp/mymacaddress/cmd
  mqttClient.subscribe((String("esp/")+WiFi.macAddress()+"/cmd").c_str(), 1);
}


void jsonAddEntry(String& buf, const char* name, const char* value)
{
  buf += buf == "" ? '{' : ',';
  buf += '"';
  buf += name;
  buf += "\":";
  buf += value;
}

void jsonAddEntry(String& buf, const char* name, const String& value)
{
  jsonAddEntry(buf,name,value.c_str());
}

template<class T>
void jsonAddEntry(String& buf, const char* name, T value)
{
  jsonAddEntry(buf, name, String(value).c_str());
}

template<int D>
class X
{
  public:
  struct R { int rh; int t; };
  static R read() { return R{0,0};}
};

// LoLin ESP8266
#define D0 16
#define D1 5
#define D2 4
#define D3 0
#define D4 2
#define D5 14
#define D6 12
#define D7 13
#define D8 15
#define DRX 3
#define DTX 1

void mqttPublishState()
{
  String msg;
  jsonAddEntry(msg, "A0", analogRead(A0));
  jsonAddEntry(msg, "MILLIS", millis());
  auto dht = DHT<D0>::read();
  jsonAddEntry(msg, "RH", dht.rh);
  jsonAddEntry(msg, "TEMP", dht.temp);
  auto dht2 = DHT<D2>::read();
  jsonAddEntry(msg, "RH2", dht2.rh);
  jsonAddEntry(msg, "TEMP2", dht2.temp);
  
  msg += '}';
  mqttPublish("state",msg);
}


////////////////////////////////////////////////////////////////////////////////////////////////////

void initializeConfiguration()
{
  const auto CONFIG_FILENAME = "/config.ini";
  
  if( auto cf = SPIFFS.open(CONFIG_FILENAME, "r") )
  {
    Serial.println("using config file for initialization");
  }
  else
  {
    Serial.println("config file not found, using defaults");
    WiFiSSID = "w2";
    WiFiPassword = "stoinastacjilokomotywa";
    MqttServer = "rpihub";
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,1);
  Serial.begin(115200);
  Serial.println();
  Serial.println("ESP MQTThing");
  Serial.println("MAC " + WiFi.macAddress());
  SPIFFS.begin();
  //SPIFFS.check();
  //SPIFFS.gc();
  FSInfo fs_info;
  SPIFFS.info(fs_info);
  Serial.println("FS "+ String(fs_info.usedBytes,DEC) + "/" + String(fs_info.totalBytes,DEC));
  initializeConfiguration();
  digitalWrite(LED_BUILTIN,0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() 
{
  // monitor connections in a loop in case they fail along the way
  
  if(WiFi.status() != WL_CONNECTED) 
    wifiConnect();    
    
  if(!mqttClient.connected()) 
    mqttConnect();

  // do the work
  {
    static auto lastUpdateTime = millis();
    auto now = millis();  
    if( now - lastUpdateTime > 10000 )
    {
      lastUpdateTime = now;
      digitalWrite(LED_BUILTIN,0);
      mqttPublishState();
      digitalWrite(LED_BUILTIN,1);
    }
  }

  mqttClient.loop();
}
