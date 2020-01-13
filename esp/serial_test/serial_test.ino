#include <ESP8266WiFi.h>

void setup()
{
  Serial.begin(115200);
  Serial.println();

  WiFi.begin("w2", "stoinastacjilokomotywa");

  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.print("Connected, IP address: ");
  Serial.println(WiFi.localIP());
}

void loop() 
{
  delay(1000);
  Serial.print("WIFI ");Serial.println(WiFi.status());
  Serial.print("A0   ");Serial.println(analogRead(A0));
}
