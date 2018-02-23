#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>
#include <hpma115S0.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <WiFiUdp.h>
#include <ESP8266mDNS.h>

#define ONE_HOUR 3600000UL //for unix time

#define timeDelay   6000 // 6 second delay for response time of sensor

long nextOperation = millis() + timeDelay; //

SoftwareSerial hpmaSerial(12, 14, false, 128); // Feather TX, Feather RX

HPMA115S0 hpma115S0(hpmaSerial); //set up instance of sensor structure

ESP8266WiFiMulti wifiMulti;    // Create an instance of the ESP8266WiFiMulti class, called 'wifiMulti'

const char* mdnsName = "esp8266";        // Domain name for the mDNS responder

WiFiUDP UDP;                   // Create an instance of the WiFiUDP class to send and receive UDP messages

IPAddress timeServerIP;        // The time.nist.gov NTP server's IP address
const char* ntpServerName = "time.nist.gov";

const int NTP_PACKET_SIZE = 48;          // NTP time stamp is in the first 48 bytes of the message

byte packetBuffer[NTP_PACKET_SIZE];      // A buffer to hold incoming and outgoing packets

/*__________________________________________________________SETUP__________________________________________________________*/

void setup() {
  Serial.begin(57600);        // Start the serial communication to send messages to the computer
  hpmaSerial.begin(9600);     // Establish serial comms to sensor
  
  

  /*Some kind of reset function based on initialisation/open serial to make sure communication is working
  //if (SOMETHING IS WRONG) {
  //      ESP.reset();
  }*/

  startWiFi();                 // Start a Wi-Fi access point, and try to connect to some given access points. Then wait for either an AP or STA connection

  startUDP();                  // Start listening for UDP messages to port 123

  WiFi.hostByName(ntpServerName, timeServerIP); // Get the IP address of the NTP server
  Serial.print("Time server IP:\t");
  Serial.println(timeServerIP);
  Serial.println("Starting measurements");
  Serial.println("\r\n");
  hpma115S0.Init();                     //initialise   
  sendNTPpacket(timeServerIP);
}

/*__________________________________________________________LOOP__________________________________________________________*/

const unsigned long intervalNTP = ONE_HOUR; // Update the time every hour
unsigned long prevNTP = 0;
unsigned long lastNTPResponse = millis();

const unsigned long intervalPM = 60000;   // measure every minute
unsigned long prevPM = 0;
bool PMRequested = false;
const unsigned long DS_delay = 6000;         // Response time of HMPA115S0 is <6s
uint32_t timeUNIX = 0;                      // The most recent timestamp received from the time server

void loop() {
  unsigned long currentMillis = millis();
  unsigned int pm2_5, pm10;   //variables to store and print device data

  if (currentMillis - prevNTP > intervalNTP) { // Request the time from the time server every hour
    prevNTP = currentMillis;
    sendNTPpacket(timeServerIP);
  }

  uint32_t time = getTime();                   // Check if the time server has responded, if so, get the UNIX time
  if (time) {
    timeUNIX = time;
    Serial.print("NTP response:\t");
    Serial.println(timeUNIX);
    lastNTPResponse = millis();
  } else if ((millis() - lastNTPResponse) > 24UL * ONE_HOUR) {
    Serial.println("More than 24 hours since last NTP response. Rebooting.");
    Serial.flush();
    ESP.reset();
  }

  if (timeUNIX != 0) {
    if (currentMillis - prevPM > intervalPM) {  // Every minute, request the PM
      hpma115S0.StartParticleMeasurement(); // Request the sensor to start sending measurements (it takes some time to read it)
      PMRequested = true;
      prevPM = currentMillis;
      Serial.println("Request sent");
    }
      
    if (currentMillis - prevPM > DS_delay && PMRequested) { // after delay
      uint32_t actualTime = timeUNIX + (currentMillis - lastNTPResponse) / 1000;
      // The actual time is the last NTP time plus the time that has elapsed since the last NTP response
      PMRequested = false;
      hpma115S0.ReadParticleMeasurement(&pm2_5,&pm10); // Get the PM from the sensor
      hpma115S0.StopParticleMeasurement();

      sendJSON(pm2_5,actualTime);   
      
    }
  } else {                                    // If we didn't receive an NTP response yet, send another request
    sendNTPpacket(timeServerIP);
    delay(500);
  }
}

/*__________________________________________________________SETUP_FUNCTIONS__________________________________________________________*/

void startWiFi() { // Try to connect to some given access points. Then wait for a connection
  wifiMulti.addAP("Roy", "Royhanna44");   // add Wi-Fi networks you want to connect to
  wifiMulti.addAP("HAMKvisitor", "hamkvisitor");
  wifiMulti.addAP("ssid_from_AP_3", "your_password_for_AP_3"); //need Khoa's server's network!

  Serial.println("Connecting");
  while (wifiMulti.run() != WL_CONNECTED) {  // Wait for the Wi-Fi to connect
    delay(250);
    Serial.print('.');
  }
  Serial.println("\r\n");
  Serial.print("Connected to ");
  Serial.println(WiFi.SSID());             // Tell us what network we're connected to
  Serial.print("IP address:\t");
  Serial.print(WiFi.localIP());            // Send the IP address of the ESP8266 to the computer
  Serial.println("\r\n");
}

void startUDP() {
  Serial.println("Starting UDP");
  UDP.begin(123);                          // Start listening for UDP messages to port 123
  Serial.print("Local port:\t");
  Serial.println(UDP.localPort());
}

void startMDNS() { // Start the mDNS responder
  MDNS.begin(mdnsName);                        // start the multicast domain name server
  Serial.print("mDNS responder started: http://");
  Serial.print(mdnsName);
  Serial.println(".local");
}

/*__________________________________________________________HELPER_FUNCTIONS__________________________________________________________*/

String formatBytes(size_t bytes) { // convert sizes in bytes to KB and MB
  if (bytes < 1024) {
    return String(bytes) + "B";
  } else if (bytes < (1024 * 1024)) {
    return String(bytes / 1024.0) + "KB";
  } else if (bytes < (1024 * 1024 * 1024)) {
    return String(bytes / 1024.0 / 1024.0) + "MB";
  }
}

unsigned long getTime() { // Check if the time server has responded, if so, get the UNIX time, otherwise, return 0
  if (UDP.parsePacket() == 0) { // If there's no response (yet)
    return 0;
  }
  UDP.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
  // Combine the 4 timestamp bytes into one 32-bit number
  uint32_t NTPTime = (packetBuffer[40] << 24) | (packetBuffer[41] << 16) | (packetBuffer[42] << 8) | packetBuffer[43];
  // Convert NTP time to a UNIX timestamp:
  // Unix time starts on Jan 1 1970. That's 2208988800 seconds in NTP time:
  const uint32_t seventyYears = 2208988800UL;
  // subtract seventy years:
  uint32_t UNIXTime = NTPTime - seventyYears;
  return UNIXTime;
}


void sendNTPpacket(IPAddress& address) {
  Serial.println("Sending NTP request");
  memset(packetBuffer, 0, NTP_PACKET_SIZE);  // set all bytes in the buffer to 0
  // Initialize values needed to form NTP request
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode

  // send a packet requesting a timestamp:
  UDP.beginPacket(address, 123); // NTP requests are to port 123
  UDP.write(packetBuffer, NTP_PACKET_SIZE);
  UDP.endPacket();
}

void sendJSON(uint8_t data,uint32_t tstamp){
  StaticJsonBuffer<100> JSONbuffer;   //Declaring static JSON buffer
  JsonObject& JSONencoder = JSONbuffer.createObject(); 
  
  JSONencoder["sensorType"] = "PM concentration";
  
  JsonArray& values = JSONencoder.createNestedArray("values"); //JSON array
  values.add(data); //Add value to array
  
  JsonArray& timestamps = JSONencoder.createNestedArray("timestamps"); //JSON array
  timestamps.add(tstamp); //Add value to array
  
  
  
  char JSONmessageBuffer[100];
  JSONencoder.prettyPrintTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
  Serial.println(JSONmessageBuffer);
  
  HTTPClient http;    //Declare object of class HTTPClient
  
  http.begin("KHOAS SERVER");      //Specify request destination
  http.addHeader("Content-Type", "application/json");  //Specify content-type header
  
  int httpCode = http.POST(JSONmessageBuffer);   //Send the request
  String payload = http.getString();                                        //Get the response payload
  
  Serial.println(httpCode);   //Print HTTP return code
  Serial.println(payload);    //Print request response payload
  
  http.end();  //Close connection
}

