#include <ESP8266WiFi.h>

#define DHTPIN 5
#include <dht11.h>

// Set web server port number to 80
WiFiServer server(80);

void setup() {
  Serial.begin(115200);
  Serial.println("Lean&Mean ESP start");
  Serial.print("Connecting to WiFi");
  WiFi.begin("w2","stoinastacjilokomotywa");
  //WiFi.begin("wg","morewind");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println();  
  Serial.print("Connected, IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("signal strength (RSSI):");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");  
  
  server.begin();
}


void loop() {
  WiFiClient client = server.available();   // Listen for incoming clients
  // Variable to store the HTTP request
  if (client) {                             // If a new client connects,
    Serial.println("connected");
    String line;
    String url;
    char prev = 0;
    while (client.connected()) {
      if (client.available()) { // read request into lines
        char c = client.read();
        //Serial.write(c);
        if(c=='\r') continue; // skip \r, watch for \n
        line += c;
        // check for GET or POST request at the end of line
        if (c == '\n') { 
          // the request line is [method] SP [url] SP [protocol]
          if(line.startsWith("GET ")||line.startsWith("POST ")) {
            // if GET or POST then store the URL
            auto n = line.indexOf(' ',3);
            auto m = line.indexOf(' ',n+1);
            url=line.substring(n+1,m);
            Serial.print(line);
          }
          // if the byte is a newline character
          // and prev byte is newline, we got two newline characters in a row,
          // that's the end of request
          if(prev == '\n') {
            Serial.print("replying to URL=");
            Serial.println(url);
            if(url=="/") {
              // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
              // and a content-type so the client knows what's coming, then a blank line.  
              client.println(
                "HTTP/1.1 200 OK\n"
                "Content-type:text/html\n"
                "Connection: close\n"
                "\n"
                "Hello! DHT11: ");
              auto ht = readDHT(); // (humidity << 8) + temperature
              client.print(String(ht >> 8));
              client.print("RH ");
              client.print(String(ht & 0xFF));
              client.print("C ");
              client.println();
            }
            else
            {
              // that's the end of the client HTTP request, so send a response:
              client.println("HTTP/1.1 404 Not Found\n");
            }
            client.stop();
            Serial.println("closed");
            break;
          }
          line="";
        }
        prev = c;
      }
    } // while
  }
}
