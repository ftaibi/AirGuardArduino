#include <time.h>
#include <ESP8266WiFi.h>
#include <AzureIoTHub.h>
#include "iothubtransporthttp.h"

#include <AzureIoTProtocol_HTTP.h>

// Times before 2010 (1970 + 40 years) are invalid
#define MIN_EPOCH 40 * 365 * 24 * 3600

// TODO: set these values before uploading the sketch
//
#define IOT_CONFIG_WIFI_SSID            "AP"
#define IOT_CONFIG_WIFI_PASSWORD        ""

#define IOT_CONFIG_CONNECTION_STRING    ""

char msg[700];
char camera_buffer[400];

IOTHUB_CLIENT_LL_HANDLE iotHubClientHandle = nullptr;


void initWifi() {
  // Attempt to connect to Wifi network:
  Serial.print("\r\n\r\nAttempting to connect to SSID: ");
  Serial.println(IOT_CONFIG_WIFI_SSID);

  // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
  WiFi.begin(IOT_CONFIG_WIFI_SSID, IOT_CONFIG_WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\r\nConnected to wifi");
}

static void initTime() {
  time_t epochTime;

  configTime(0, 0, "pool.ntp.org", "time.nist.gov");

  while (true) {
    epochTime = time(NULL);

    if (epochTime < MIN_EPOCH) {
      //Serial.println("Fetching NTP epoch time failed! Waiting 2 seconds to retry.");
      delay(2000);
    } else {
      Serial.print("Fetched NTP epoch time is: ");
      Serial.println(epochTime);
      break;
    }
  }
}

void setup()
{

  Serial.begin(115200);

  // Init WIFI
  //
  initWifi();
  //
  // Init time
  //
  initTime();
  //
  // Init IoT client
  //
  iotHubClientHandle = IoTHubClient_LL_CreateFromConnectionString(IOT_CONFIG_CONNECTION_STRING, HTTP_Protocol);
  if (iotHubClientHandle == NULL)
  {
    Serial.println("Failed on IoTHubClient_LL_Create");
  }
}

void loop()
{
  // Read from local serial

  while (Serial.available() > 70) {
    float MassPM1 = Serial.readStringUntil('\r').toInt() / 100.0;
    float MassPM2 = Serial.readStringUntil('\r').toInt() / 100.0;
    float MassPM4 = Serial.readStringUntil('\r').toInt() / 100.0;
    float MassPM10 = Serial.readStringUntil('\r').toInt() / 100.0;
    float NumPM0 = Serial.readStringUntil('\r').toInt() / 100.0;
    float NumPM1 = Serial.readStringUntil('\r').toInt() / 100.0;
    float NumPM2 = Serial.readStringUntil('\r').toInt() / 100.0;
    float NumPM4 = Serial.readStringUntil('\r').toInt() / 100.0;
    float NumPM10 = Serial.readStringUntil('\r').toInt() / 100.0;
    float PartSize = Serial.readStringUntil('\r').toInt() / 100.0;
    int Humidity = Serial.readStringUntil('\r').toInt();
    int Temperature = Serial.readStringUntil('\r').toInt();
    int CO2 = Serial.readStringUntil('\r').toInt();
    int Audio = Serial.readStringUntil('\r').toInt();
    int Density = Serial.readStringUntil('\r').toInt();
    Serial.readBytesUntil('\r', camera_buffer, 400);
    
    sprintf(msg, "{'MassPM1': %.2f,'MassPM2': %.2f,'MassPM4': %.2f,'MassPM10': %.2f,'NumPM0': %.2f,'NumPM1': %.2f,'NumPM2': %.2f,'NumPM4': %.2f,'NumPM10': %.2f,'PartSize': %.2f, 'Humidity': %i,'Temperature': %i, 'CO2': %i, 'Audio': %i, 'Density': %i, 'Camera': [ %s ]}", MassPM1, MassPM2, MassPM4, MassPM10, NumPM0, NumPM1, NumPM2, NumPM4, NumPM10, PartSize, Humidity, Temperature, CO2, Audio, Density, camera_buffer);


    IOTHUB_MESSAGE_HANDLE messageHandle = IoTHubMessage_CreateFromByteArray((const unsigned char *)msg, strlen(msg));

    if (messageHandle == NULL)
    {
      Serial.println("unable to create a new IoTHubMessage");
    }
    else
    {
      if (IoTHubClient_LL_SendEventAsync(iotHubClientHandle, messageHandle, nullptr, nullptr) != IOTHUB_CLIENT_OK)
      {
        Serial.println("failed to hand over the message to IoTHubClient");
      }
      else
      {
        Serial.println("IoTHubClient accepted the message for delivery");
      }
      IoTHubMessage_Destroy(messageHandle);
    }

    IOTHUB_CLIENT_STATUS status;
    while ((IoTHubClient_LL_GetSendStatus(iotHubClientHandle, &status) == IOTHUB_CLIENT_OK) && (status == IOTHUB_CLIENT_SEND_STATUS_BUSY))
    {
      IoTHubClient_LL_DoWork(iotHubClientHandle);
      ThreadAPI_Sleep(100);
    }
  }
}
