/* ------------------------------------------------- */

#include "ESPTelnet.h"          
#include <Servo.h>

/* ------------------------------------------------- */

#define SERIAL_SPEED    115200
#define WIFI_SSID       "magicnet-guest"
#define WIFI_PASSWORD   "APPassphrase"

#define SERVO_PIN	D4
#define OPEN_PIN	D1
#define CLOSE_PIN	D2
#define WATER_PIN	D0

/* ------------------------------------------------- */

// 1 => tell server we got open
// -1 => tell server we got close
int8_t posswitch; 

int16_t openpos=50;
int16_t closepos=150;

ESPTelnet telnet;
IPAddress ip;
uint16_t  port = 23;

Servo myservo;  // create servo object to control a servo
int16_t pos=90; // slightly opened
int16_t newpos=pos;

/* ------------------------------------------------- */

void setservo()
{
    if (newpos == pos) {
	Serial.print("Ignoring position move to ");
	Serial.println(pos);
	return;
    }   
    Serial.printf("Changing servo to %d\n", newpos);
    while (newpos != pos) {
      if (newpos > pos) { pos++; }
      if (newpos < pos) { pos--; }
      delay(40);
      myservo.write(pos);
    }
}


/* ------------------------------------------------- */

void setupSerial(long speed, String msg = "") {
  Serial.begin(speed);
  while (!Serial) {
  }
  delay(200);  
  Serial.println();
  Serial.println();
  if (msg != "") Serial.println(msg);
}

/* ------------------------------------------------- */

bool isConnected() {
  return (WiFi.status() == WL_CONNECTED);
}

/* ------------------------------------------------- */

bool connectToWiFi(const char* ssid, const char* password, int max_tries = 20, int pause = 500) {
  int i = 0;
  WiFi.mode(WIFI_STA);
  #if defined(ARDUINO_ARCH_ESP8266)
    WiFi.forceSleepWake();
    delay(200);
  #endif
  WiFi.begin(ssid, password);
  do {
    delay(pause);
    Serial.print(".");
  } while (!isConnected() || i++ < max_tries);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  return isConnected();
}

/* ------------------------------------------------- */

void errorMsg(String error, bool restart = true) {
  Serial.println(error);
  if (restart) {
    Serial.println("Rebooting now...");
    delay(2000);
    ESP.restart();
    delay(2000);
  }
}

/* ------------------------------------------------- */

void setupTelnet() {  
    // passing on functions for various telnet events
    telnet.onConnect(onTelnetConnect);
    telnet.onConnectionAttempt(onTelnetConnectionAttempt);
    telnet.onReconnect(onTelnetReconnect);
    telnet.onDisconnect(onTelnetDisconnect);

    // passing a lambda function
    telnet.onInputReceived([](String str) {
      // checks for a certain command
      if ((newpos = str.toInt())) {
	setservo();
	telnet.print("Moved servo angle to ");
	telnet.println(String(newpos));
	posswitch = 0; 
      } else if (str == "ping") {
	telnet.println("> pong");
	Serial.println("- Telnet: pong");
      // disconnect the client
      } else if (str == "bye") {
	telnet.print("> Open SW: ");
	telnet.print((char) (!digitalRead(OPEN_PIN)+48));
	telnet.print(" Close SW: ");
	telnet.print((char) (!digitalRead(CLOSE_PIN)+48));
	telnet.print(" Water: ");
	telnet.println((char) (digitalRead(WATER_PIN)+48));
	telnet.println("> disconnecting you... Current servo angle is");
	Serial.print("Disconnecting and sending servo angle ");
	if (posswitch) { telnet.print("new: "); Serial.print("new: "); posswitch = 0; };
	telnet.println(String(pos));
	Serial.println(pos);
	telnet.disconnectClient();
      }
    });

    Serial.print("- Telnet: ");
    if (telnet.begin(port)) {
      Serial.println("running");
    } else {
      Serial.println("error.");
      errorMsg("Will reboot...");
    }
}

/* ------------------------------------------------- */

// (optional) callback functions for telnet events
void onTelnetConnect(String ip) {
  Serial.print("- Telnet: ");
  Serial.print(ip);
  Serial.println(" connected");
  telnet.println("\nWelcome " + telnet.getIP());
}

void onTelnetDisconnect(String ip) {
  Serial.print("- Telnet: ");
  Serial.print(ip);
  Serial.println(" disconnected");
}

void onTelnetReconnect(String ip) {
  Serial.print("- Telnet: ");
  Serial.print(ip);
  Serial.println(" reconnected");
}

void onTelnetConnectionAttempt(String ip) {
  Serial.print("- Telnet: ");
  Serial.print(ip);
  Serial.println(" tried to connect");
}

/* ------------------------------------------------- */

void setup() {
  pinMode(OPEN_PIN, INPUT_PULLUP);
  pinMode(CLOSE_PIN, INPUT_PULLUP);
  pinMode(WATER_PIN, INPUT_PULLUP);

  setupSerial(SERIAL_SPEED, "Servo Init, opening to 90");
  myservo.attach(SERVO_PIN);  // attaches the servo on GIO2 to the servo object
  myservo.write(pos);
#if 0

  int pos;

  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
#endif

  Serial.println("Start");
  Serial.print("Wifi: ");
  connectToWiFi(WIFI_SSID, WIFI_PASSWORD);
  
    if (isConnected()) {
	Serial.println("Wifi connected, opening 1");
	newpos = 80;
	setservo();
	ip = WiFi.localIP();
	Serial.println();
	Serial.print("- Telnet: "); Serial.print(ip); Serial.print(" "); Serial.print(port);
	setupTelnet();
	Serial.println("telnet setup, opening 2");
	newpos = 85;
	setservo();
	Serial.println("setservo2, opening 3");
	newpos = 70;
	setservo();
    } else {
	Serial.println();    
	errorMsg("Error connecting to WiFi");
    }
    Serial.println("setservo3, opening 4");
    newpos = 75;
    setservo();
    Serial.print("Setup done, opening to ");
    newpos = 60;
    Serial.println(newpos);
    setservo();
}

/* ------------------------------------------------- */

void loop() {
    telnet.loop();

    // send serial input to telnet as output
    if (Serial.available()) {
	telnet.print(Serial.read());


    }

    if (millis() % 5000 < 1) Serial.printf("Open: %d, Close: %d, Water: %d\n", !digitalRead(OPEN_PIN), !digitalRead(CLOSE_PIN), digitalRead(WATER_PIN));
    delay(1);

    if (!digitalRead(OPEN_PIN) && posswitch != 1) {
	Serial.println("Switch to Open");
	posswitch = 1;
	newpos = openpos;
	setservo();
    }
    if (!digitalRead(CLOSE_PIN) && posswitch != -1) {
	Serial.println("Switch to Close");
	posswitch = -1;
	newpos = closepos;
	setservo();
    }
}
//* ------------------------------------------------- */
