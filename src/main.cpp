#include "sensor.h"
#include "sensor_bno055.h"
#include <Arduino.h>
#include <ESP8266WiFi.h>

Sensor *sensor = NULL;

WiFiServer server(4030);             // 4030 is the default port Skysafari uses for WiFi connection to telescopes
WiFiClient remoteClient;             // represents the connection to the remote app (Skysafari)
#define WiFi_Access_Point_Name "DSC" // Name of the WiFi access point this device will create for your tablet/phone to connect to.

#define STEPS_IN_FULL_CIRCLE 36000

void setup()
{
    Serial.begin(115200);

    Serial.println("Starting up");

    sensor = new SensorBNO055();
    sensor->init();

    Serial.println("Sensor init done");

    WiFi.mode(WIFI_AP);
    IPAddress ip(1, 2, 3, 4); // The "telescope IP address" that Skysafari should connect to is 1.2.3.4 which is easy to remember.
    IPAddress gateway(1, 2, 3, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.softAPConfig(ip, gateway, subnet);
    WiFi.softAP(WiFi_Access_Point_Name);
    IPAddress myIP = WiFi.softAPIP();

    server.begin();
    server.setNoDelay(true);

    Serial.println("Wifi init done");
}

void attendTcpRequests()
{
    // check for new or lost connections:
    if (server.hasClient())
    {
        if (!remoteClient || !remoteClient.connected())
        {
            if (remoteClient)
            {
                Serial.print("Client Disconnected\n");
                remoteClient.stop();
            }
            remoteClient = server.available();
            remoteClient.setNoDelay(true);
        }
    }

    // when we have a new incoming connection from Skysafari:
    while (remoteClient.available())
    {
        byte skySafariCommand = remoteClient.read();

        if (skySafariCommand == 81) // 81 is ascii for Q, which is the only command skysafari sends to "basic encoders"
        {

            char encoderResponse[20];

            Coordinates coord = sensor->getCoordinates();

            sprintf(encoderResponse, "%i\t%i\t\n", coord.getAz(), coord.getAlt());

            Serial.printf("Sending to Skysafari: ");
            Serial.println(encoderResponse);

            remoteClient.println(encoderResponse);
        }
        else if (skySafariCommand == 72) // 'H' - request for encoder resolution, e.g. 10000-10000\n
        {
            char response[20];
            // Resolution on both axis is equal
            snprintf(response, 20, "%u-%u", STEPS_IN_FULL_CIRCLE, STEPS_IN_FULL_CIRCLE);
            //      Serial.println(response);

            remoteClient.println(response);
        }
        else
        {
            Serial.print("*****");
            Serial.println(skySafariCommand);
        }
    }
}

void loop()
{
    attendTcpRequests(); // gets priority to prevent timeouts on Skysafari. Measured AVG execution time = 18ms
    yield();
}
