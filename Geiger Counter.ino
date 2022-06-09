#include <ArduinoNATS.h>

#include <WiFiNINA.h>
#include <samd.h>

#include "arduino_secrets.h"

int wifi_closest_rssi = -10; // Closest 
int wifi_farthest_rssi = -90; // Farthest
int rssi = wifi_farthest_rssi; // Wifi signal strengh variable

const float dist_closest = 0;
const float dist_farthest = 30;
float dist = dist_farthest; // Really rough distance to wifi signal, in meters?

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;     // the Wifi radio's status
int buzzerStatus = LOW;

bool IsRadiationLeaking = true;

WiFiClient client;
NATS nats(&client, SECRET_NATS_SERVER, NATS_DEFAULT_PORT, "Arduino Leak Detector");

void setup() {

  //Initialize serial and wait for port to open:
  Serial.begin(9600);

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to network: ");
    Serial.println(ssid);

    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);

    // wait 5 seconds for connection:
    delay(5000);
  }

  SetupNats();

  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);

  //When connection successful :
  Serial.println("You're connected to the network");
  Serial.println("----------------------------------------");

  delay(100);

  DAC->CTRLB.bit.REFSEL = DAC_CTRLB_REFSEL_INT1V_Val;

}

void nats_msg_radiationLeekHandler(NATS::msg msg)
{
  Serial.print("Nats Msg Received: [Subj: "); Serial.print(msg.subject); Serial.print(", data: "); Serial.println(msg.data);
  if (strcmp(msg.data, "true") == 0)
  {
    IsRadiationLeaking = true;
  }
  else if (strcmp(msg.data, "false") == 0)
  {
    IsRadiationLeaking = false;
  }
  else
  {
    Serial.println("Nats msg not recognized.");
  }
}
void nats_on_connect()
{
  Serial.println("Nats connection complete!");
  nats.subscribe("falcon.radiationleak.IsLeaking", nats_msg_radiationLeekHandler);
}
void nats_on_error()
{
  Serial.println("Nats error.");
}
void SetupNats()
{
  nats.on_connect = nats_on_connect;
  nats.on_error = nats_on_error;
  Serial.print("Connect to NATS - "); Serial.print(nats.hostname); Serial.print(":"); Serial.print(nats.port); Serial.print(" ---- ");
  if (nats.connect())
  {
    Serial.println("Success!");
  }
  else
  {
    Serial.println("Failure. :(");
  }
  Serial.print("Nats Remote IP according to client: "); Serial.println(client.remoteIP());

  if (nats.connected)
  {
    nats.process();
  }
}

void loop() {
  nats.process();

  rssi = WiFi.RSSI();
  if (IsRadiationLeaking)
  {
    dist = rssiDist(rssi);
  }
  else
  {
    dist = dist_farthest * 2; // really far value, will cause gauge to show nothing and clicking to stop
  }

  // if (Serial.peek() > -1){
  //   int value = Serial.read() - '0';
  //   if (value >= 0 && value <= 9)
  //   {
  //     rssi = map(value, 9, 0, wifi_closest_rssi, wifi_farthest_rssi);
  //   }
  // }
  print_data();
  analog_out2();
  
  unsigned long trigger = 1000./(dist*dist); 
  unsigned long r = random(2000);
  if (r < trigger) {
    buzzer();
  }
}

void buzzer() {
  digitalWrite(5, HIGH);
  delayMicroseconds(200);
  digitalWrite(5, LOW);
}

float rssiDist(double rssi){
  const float N = 2; // Environmental factor, measured on-site. Typically 2 to 4
  const int OneMeterRssi = -30;
  return pow(10, ((OneMeterRssi - rssi) / (10*N)));
}

// Function defining the display up to the signal quality
int lastPrintTime = 0;
const int printMillisInterval = 200;
void print_data() {
  int time = millis();
  if (time - lastPrintTime > printMillisInterval)
  {
    lastPrintTime = time;
    Serial.print("signal strength (RSSI):");
    Serial.println(rssi);

    Serial.print("Distance (m):");
    Serial.println(dist);

    if (IsRadiationLeaking)
    {
      Serial.println("Radiation Leaking: true");
    }
    else
    {
      Serial.println("Radiation Leaking: false");
    }
  }
}

void analog_out2() {
  analogWriteResolution(8);
  int percentVal = map(dist, dist_farthest, dist_closest, 0, 255);
  percentVal = constrain(percentVal, 0, 255);
  analogWrite(3, percentVal);
}