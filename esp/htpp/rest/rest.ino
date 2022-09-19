#include <Arduino.h>
#include <Esp.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
#include <time.h>

#include <Scheduler.h>
#include <Task.h>
#include <LeanTask.h>

#include <memory>
#include <sys/time.h>
#include <time.h>


String getLocalTimeString() {
  struct tm ti;
  time_t now;
  time(&now);
  localtime_r(&now, &ti);
  return stringPrintf("%04d%02d%02d-%02d%02d%02d", 1900+ti.tm_year, ti.tm_mon+1, ti.tm_mday, ti.tm_hour, ti.tm_min, ti.tm_sec);
}

std::shared_ptr<char> spPrintf(char* format, ...) {
  va_list args;
  va_start(args, format);
  auto len = vsnprintf(nullptr, 0, format, args) + 1;
  std::shared_ptr<char> buf((char*)malloc(len), free);
  vsnprintf(buf.get(), len, format, args);
  va_end(args);
  return buf;
}

String stringPrintf(char* format, ...) {
  va_list args;
  va_start(args, format);
  auto len = vsnprintf(nullptr, 0, format, args) + 1;
  auto buf = static_cast<char*>(malloc(len));
  vsnprintf(buf, len, format, args);
  va_end(args);
  String ret(buf);  // this adds a strlen and a copy; wish there was static String::printf() or String::appendPrintf()
  free(buf);
  return ret;
}

ESP8266WebServer server(80);
ESP8266HTTPUpdateServer httpUpdater;

void page_index() {
  String page;
  page.reserve(1000);
  page += F("<html><body>");
  page += F("<h1>Da Web Server</h1>");
  page += stringPrintf("<h2>%s</h2>", getLocalTimeString().c_str());
  page += F("<br>System time: ");
  page += system_get_time();
  page += F("<br>ResetReason: ");
  page += ESP.getResetReason();
  page += F("<br>ChipId: ");
  page += ESP.getChipId();
  page += F("<br>Vcc: ");
  page += ESP.getVcc();
  page += F("<br>FreeHeap: ");
  page += ESP.getFreeHeap();
  page += stringPrintf("<br>FlashChip: %02X:%08X %dkB", ESP.getFlashChipVendorId(), ESP.getFlashChipId(), ESP.getFlashChipSize() / 1024);
  page += stringPrintf("<br>SketchSize: %dkB; free: %dkB", (ESP.getSketchSize() + 512) / 1024, (ESP.getFreeSketchSpace() + 512) / 1024);
  page += F("<br>");
  page += F("<br>ADC: ");
  page += analogRead(0);
  page += F("</body></html>");
  server.send(200, F("text/html"), page);
}


void rest_systemtime() {
  server.send(200, "text/json", stringPrintf("{\"systemtime\": %d}", system_get_time()));
}

// read/writes to GPIO
void rest_pin() {
  if (server.hasArg("p")) {
    auto pin = server.arg("p").toInt();
    auto val = 0;
    if (server.hasArg("v")) {
      val = server.arg("v").toInt();
      pinMode(pin, OUTPUT);
      digitalWrite(pin, val);
    } else {
      pinMode(pin, INPUT);
      val = digitalRead(pin);
    }
    server.send(200, "text/json", stringPrintf("{\"pin: %d\", \"value\": %d}", pin, val));
  }
}

// crude A2D converter to infer resistance or voltage by change/discharge time 
// assumes a capacitor it connected to a pin to ground
// toggles the pin to charge cap and then measures time it takes it to discharge
// returns this time in us
int measureCapDecayOnPin(int pin, int startPinState = 1) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, startPinState);
  delay(10);  // wait to preset the capacitor (here 10ms)
  auto start = micros();
  pinMode(pin, INPUT);
  while (digitalRead(pin) != startPinState && micros() - start < 100000 /* limit to 0.1s */) {
  }
  return micros() - start;
}

void rest_decay() {
  auto pin = server.argName(0).toInt();
  auto us = measureCapDecayOnPin(pin);
  server.send(200, "text/json", stringPrintf("{\"pin: %d\", \"us\": %d}", pin, us));
}


void handlePageNotFound() {
  String message(F("File Not Found\n\nURI: "));
  message += server.uri();
  message += F("\nMethod: ");
  message += server.method() == HTTP_GET ? F("GET") : F("POST");
  message += F("\nArguments: ");
  message += server.args();
  message += "\n";
  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
}


class HttpServerTask : public Task {
protected:

  void setup() {
    // Set server routing
    server.on(F("/"), HTTP_GET, page_index);
    server.on(F("/systemtime"), HTTP_GET, rest_systemtime);
    server.on(F("/restart"), HTTP_GET, []() {
      ESP.restart();
    });
    server.on(F("/reset"), HTTP_GET, []() {
      ESP.reset();
    });
    server.on(F("/pin"), HTTP_GET, rest_pin);
    server.on(F("/decay"), HTTP_GET, rest_decay);
    
    server.onNotFound(handlePageNotFound);

    httpUpdater.setup(&server);

    // Start server
    server.begin();
    Serial.println("HTTP server started");
  }

  void loop() {
    server.handleClient();
    yield();
  }

} httpServerTask;


const char* ssid = "w2";
const char* password = "stoinastacjilokomotywa";

class WifiTask : public Task {
public:  
  WifiTask() {}
  
protected:
  void loop() {
    // Already connected, nothing more to do.
    if (WiFi.status() == WL_CONNECTED) {
      yield();
      return;
    }

    auto connectingStart = millis();
    Serial.println("WiFi is connecting...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      // if we cannot connect for 5min - try rebooting
      if( millis() - connectingStart > 5*60*1000 ) {
        Serial.println("Rebooting due to no WiFi")
        ESP.reset();
      } 
      yield();
      Serial.print("~");
    }
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    // Activate mDNS this is used to be able to connect to the server
    // with local DNS hostmane esp8266.local
    if (MDNS.begin("esp8266")) {
      Serial.println("MDNS responder started");
    }

    Serial.println("Configuring date and time via NTP...");
    
    configTime(0, 0, "pool.ntp.org");
    setenv( "TZ", "PST8PDT", 1 );
    
    auto start = millis();
    while(millis() - start < 1000) {
      time_t now;
      time(&now);
      if(now > 1451606400 /*20160101 UTC*/) {        
        break;
      }
      yield();
    }      
    Serial.println(getLocalTimeString());
  }
} wifiTask;


void setup(void) {
  Serial.begin(115200);
  Serial.println(F("SETUP"));

  ESP.wdtEnable(1);
  Serial.println("Watchdog started...");

  Scheduler.start(&wifiTask);
  Scheduler.start(&httpServerTask);
  Scheduler.begin();
  Serial.println("Scheduler started...");
}

void loop() {
    // reboot every 13 hours
    if( millis() > 13 * 60 * 60 * 1000 ) {
      Serial.println(getLocalTimeString() + " - scheduled reboot!");
      ESP.reset();
    }
}
