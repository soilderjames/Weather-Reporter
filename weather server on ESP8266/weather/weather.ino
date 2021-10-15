#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>

void weather()
{
  HTTPClient http;
  WiFiClient client;
  http.begin(client,"http://api.openweathermap.org/data/2.5/weather?id=(CITY ID)&units=metric&APPID=(API SECRET)");
  int httpCode = http.GET();
  if (httpCode > 0)
{
String payload = http.getString();
Serial.printf(payload.c_str());
/*DynamicJsonDocument doc(1024);
DeserializationError error = deserializeJson(doc, payload);
if (error) {
  Serial.print(F("deserializeJson() failed: "));
  Serial.println(error.f_str());
  return;
}
const char* city = doc["name"];
float temp = (float)(doc["main"]["temp"]);
JsonObject weather_0 = doc["weather"][0];
int weather_0_id = weather_0["id"];
const char* weather_0_main = weather_0["main"];
      Serial.printf("%s\n", city);
      Serial.printf("%.2f°C\r\n", temp);      
      Serial.printf("%s\n", weather_0_main);*/
}
}

void setup() {
 Serial.begin(115200);
 WiFi.begin("ssid", "password");
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) 
  {
    if(Serial.find("GET WEATHER"))
  {
    weather();
  while(Serial.read() >= 0){}
}
  }
}
