#include "config.h"
// config file needs to include WIFI_AP, WIFI_AP, TOKEN, HOSTIP
#include <WiFiEspClient.h>
#include <WiFiEsp.h>
#include <WiFiEspUdp.h>
#include "SoftwareSerial.h"
#include <ThingsBoard.h>
#include "seeed_bme680.h"



#define IIC_ADDR  uint8_t(0x76)

char thingsboardServer[] = HOSTIP;

// Initialize the Ethernet client object
WiFiEspClient espClient;

ThingsBoard tb(espClient);

SoftwareSerial soft(10, 11); // RX, TX

int status = WL_IDLE_STATUS;
unsigned long lastSend;

Seeed_BME680 bme680(IIC_ADDR); /* IIC PROTOCOL */

void setup() {
  // initialize serial for debugging
  Serial.begin(9600);
  // sensors.begin();
 
  while (!bme680.init()) {
    Serial.println(F("bme680 init failed ! can't find device!"));
    delay(10000);
  }


  InitWiFi();
  lastSend = 0;
}

void loop() {
  status = WiFi.status();
  if ( status != WL_CONNECTED) {
    while ( status != WL_CONNECTED) {
      Serial.print(F("Attempting to connect to WPA SSID: "));
      Serial.println(WIFI_AP);
      // Connect to WPA/WPA2 network
      status = WiFi.begin(WIFI_AP, WIFI_PASSWORD);
      delay(500);
    }
    Serial.println(F("Connected to AP"));
  }

  if ( !tb.connected() ) {
    reconnect();
  }

  if ( millis() - lastSend > 1000 ) { // Update and send only after 1 seconds
    getAndSendBME680Data();
    lastSend = millis();
  }

  tb.loop();
}

void getAndSendBME680Data()
{
  // check to see if the bme680 is working
  if (bme680.read_sensor_data()) {
    Serial.println(F("Failed to perform reading :("));
    return;
  }

// assigning readings to variables
  float temp = bme680.sensor_result_value.temperature;

  float pressure = bme680.sensor_result_value.pressure / 1000.0;

  float humidity = bme680.sensor_result_value.humidity;

  float gas = bme680.sensor_result_value.gas / 1000.0;

// serial prints for confirmation that data looks correct
  Serial.println(F("Sending data to ThingsBoard:"));

  tb.sendTelemetryFloat("Temperature", temp);
  tb.sendTelemetryFloat("Pressure", pressure);
  tb.sendTelemetryFloat("Humidity", humidity);
  tb.sendTelemetryFloat("Gas", gas);

  delay(1000);
}

void InitWiFi()
{
  // initialize serial for ESP module
  soft.begin(9600);
  // initialize ESP module
  WiFi.init(&soft);
  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println(F("WiFi shield not present"));
    // don't continue
    while (true);
  }

  Serial.println(F("Connecting to AP ..."));
  // attempt to connect to WiFi network
  while ( status != WL_CONNECTED) {
    Serial.print(F("Attempting to connect to WPA SSID: "));
    Serial.println(WIFI_AP);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(WIFI_AP, WIFI_PASSWORD);
    delay(500);
  }
  Serial.println(F("Connected to AP"));
}

void reconnect() {
  // Loop until we're reconnected
  while (!tb.connected()) {
    Serial.print(F("Connecting to ThingsBoard node ..."));
    // Attempt to connect (clientId, username, password)
    if ( tb.connect(thingsboardServer, TOKEN) ) {
      Serial.println(F( "[DONE]" ));
    } else {
      Serial.print(F( "[FAILED]" ));
      Serial.println(F( " : retrying in 5 seconds" ));
      // Wait 5 seconds before retrying
      delay( 5000 );
    }
  }
}
