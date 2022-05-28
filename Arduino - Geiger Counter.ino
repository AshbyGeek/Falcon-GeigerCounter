#include <WiFiNINA.h>
#include <samd.h>

#include "arduino_secrets.h"

int wifi_closest_rssi = -10; // Closest 
int wifi_farthest_rssi = -90; // Farthest
int rssi = wifi_farthest_rssi; // Wifi signal strengh variable
float dist = 1000; // Really rough distance to wifi signal, in meters?

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;     // the Wifi radio's status
int buzzerStatus = LOW;

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


  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);

  //When connection successful :
  Serial.println("You're connected to the network");
  Serial.println("----------------------------------------");

  delay(100);

  DAC->CTRLB.bit.REFSEL = DAC_CTRLB_REFSEL_INT1V_Val;

}

void loop() {
  rssi = WiFi.RSSI();
  dist = rssiDist(rssi);
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
  }
}

void analog_out2() {
  const float closest = 0;
  const float farthest = 30;
  analogWriteResolution(8);
  int percentVal = map(dist, closest, farthest, 0, 255);
  analogWrite(3, percentVal);
}