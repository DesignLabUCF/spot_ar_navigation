#include "config.h"
// config file needs to include WIFI_AP, WIFI_AP, TOKEN, HOSTIP
#include <WiFiEspClient.h>
#include <WiFiEsp.h>
#include <WiFiEspUdp.h>
#include "SoftwareSerial.h"
#include <ThingsBoard.h>
#include "SCD30.h"
#include "seeed_bme680.h"
#include <Multichannel_Gas_GMXXX.h>
#include <Wire.h>


// BME680
#define IIC_ADDR  uint8_t(0x76)
Seeed_BME680 bme680(IIC_ADDR); /* IIC PROTOCOL */
//SCD30 sensor

//gas sensor
GAS_GMXXX<TwoWire> gas;
static uint8_t recv_cmd[8] = {};

char thingsboardServer[] = HOSTIP;

// Initialize the Ethernet client object
WiFiEspClient espClient;

ThingsBoard tb(espClient);

SoftwareSerial soft(10, 11); // RX, TX

int status = WL_IDLE_STATUS;
unsigned long lastSend;


void setup() {
  // initialize serial for debugging
  Wire.begin();
  Serial.begin(9600);
 
  //SCD30 sensor
  scd30.initialize(); 
  //gas sensor
  gas.begin(Wire, 0x08); 

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
    getAndSendData();
    lastSend = millis();
  }

  tb.loop();
}

void getAndSendData()
{
  // scd30
  float result[3] = {0};

  if (scd30.isAvailable()) {
    scd30.getCarbonDioxideConcentration(result);
    tb.sendTelemetryFloat("Carbon Dioxide", result[0]);
    tb.sendTelemetryFloat("Temperature", result[1]);
    tb.sendTelemetryFloat("Humidity", result[2]);
    
    Serial.print(F("Data sent to ThingsBoard... "));
    Serial.println(result[0]);
  }


  // check to see if the bme680 is working
  if (bme680.read_sensor_data()) {
    Serial.println(F("Failed to perform reading :("));
    return;
  }

  // assigning readings to variables
  float temp = bme680.sensor_result_value.temperature;

  float pressure = bme680.sensor_result_value.pressure / 1000.0;

  float humidity = bme680.sensor_result_value.humidity;

  float gas680 = bme680.sensor_result_value.gas / 1000.0;

  // serial prints for confirmation that data looks correct
  Serial.println(F("Sending data to ThingsBoard:"));

  // duplicate value
  tb.sendTelemetryFloat("Temperature", temp);
  tb.sendTelemetryFloat("Pressure", pressure);
  // duplicate value
  tb.sendTelemetryFloat("Humidity", humidity);
  tb.sendTelemetryFloat("Gas", gas680);


  //multichannel gas
  uint32_t val = 0;

  // assigning readings to variables and then send it in order to save variable space
  val = gas.getGM102B(); 
  tb.sendTelemetryFloat("NO2", gas.calcVol(val));

  val = gas.getGM302B();
  tb.sendTelemetryFloat("C2H5OH", gas.calcVol(val));

  val = gas.getGM502B();  
  tb.sendTelemetryFloat("VOC", gas.calcVol(val));

  val = gas.getGM702B();
  tb.sendTelemetryFloat("CO", gas.calcVol(val));

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
