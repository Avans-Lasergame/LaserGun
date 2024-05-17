#include <IRremote.hpp>

#include <WiFi.h>
#include <WiFiClient.h>

WiFiClient client;

#define RECEIVER_PIN 26    // Pin connected to the IR receiver
#define TRANSMITTER_PIN 2  // Pin connected to the IR transmitter
#define SHOOT_PIN 9        // Pin connected to the shoot button
#define RELOAD_PIN 27      // Pin connected to the reload button
#define motorPin 25         // Pin connected to te vibration motor
#define redPin 10         // Pin connected to te vibration motor
#define greenPin 5         // Pin connected to te vibration motor
#define bluePin 13         // Pin connected to te vibration motor

const char* ssid = "Lasergame";
const char* password = "avans-01";
const char* host = "192.168.137.1";  // IP address of your Java server
const int gunID = 0;

const bool debug = true;

int DELAY_BETWEEN_REPEAT = 50;
int maxAmmo = 12;
int ammo = 12;


bool ledState;
float seconds;
float secondsLed;
float secondsMotor;
float vibrateTime;

hw_timer_t* timer = NULL;

void IRAM_ATTR onTimer() {
  seconds += 0.1;
}

void setup() {
  Serial.begin(115200);
  if (debug)
    Serial.println("Connecting to WiFi");
  WiFi.begin(ssid, password);

  IrSender.begin(TRANSMITTER_PIN);
  IrReceiver.begin(RECEIVER_PIN);
  IrReceiver.start();
  pinMode(SHOOT_PIN, INPUT_PULLUP);
  pinMode(RELOAD_PIN, INPUT_PULLUP);
  pinMode(motorPin, OUTPUT);
  digitalWrite(motorPin, false);

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  led(0,0,1);

  timer = timerBegin(0, 8, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000000, true);
  timerAlarmEnable(timer);  //Just Enable
}

bool reloadStateLast, shootStateLast;

void readInputs() {
  bool shootState = (digitalRead(SHOOT_PIN) == LOW);    // Button pin is active LOW
  bool reloadState = (digitalRead(RELOAD_PIN) == LOW);  // Button pin is active LOW
  if (reloadState != reloadStateLast) {                 //check is button state changed
    reloadStateLast = reloadState;
    if (ammo < maxAmmo) {
      reload();
    }
  }
  if (shootState && !shootStateLast) {
    Serial.println(F("Stop receiving"));
    IrReceiver.stop();
  } else if (!shootState && shootStateLast) {
    // Restart receiver
    Serial.println(F("Button released -> start receiving"));
    IrReceiver.start();
  } else if (IrReceiver.decode()) {
    /*
         * Button is not pressed and data available -> send received data and resume
         */
    int protocol = IrReceiver.decodedIRData.protocol;
    int address = IrReceiver.decodedIRData.address;
    int command = IrReceiver.decodedIRData.command;
    if (debug) {
      IrReceiver.printIRResultShort(&Serial);
    }
    if (protocol == 8 && command == 0x01) {  //check for correct protcol and correct command
      hit(address);
    }
    delay(20);
    IrReceiver.resume();  // resume receiver
  }
  if (shootState != shootStateLast) {  //check is button state changed
    shootStateLast = shootState;
    if (shootState) {
      shoot();
    }
  }
}

void reload() {
  if (ammo != -1)
    ammo = maxAmmo;
}


void shoot() {
  if (ammo != 0) {
    if (ammo > 0) {
      ammo--;
    }
    IrSender.sendNEC(gunID, 0x01, 0);
    motor(0.2, 255);
  }
  Serial.println(ammo);
}


void hit(int address) {
  if (client.connected()) {
    motor(0.2, 255);
    client.println("hitby:" + String(address));
  }
}

void motor(float duration, int strenght) {
  if (strenght >= 255) {
    strenght = 255;
  }
  secondsMotor = seconds;
  vibrateTime = duration;
  // digitalWrite(motorPin, true);
  analogWrite(motorPin, strenght);
}
void led(int red, int green, int blue) {
  if (red == 1) {
    red = 0;
  } else {
    red = 1;
  }
  if (green == 1) {
    green = 0;
  } else {
    green = 1;
  }
  if (blue == 1) {
    blue = 0;
  } else {
    blue = 1;
  }
  digitalWrite(greenPin, green);
  digitalWrite(redPin, red);
  digitalWrite(bluePin, blue);
}

void checktime() {
  // Serial.println(seconds);
  // Serial.println(secondsMotor);
  // Serial.println(vibrateTime);
  if (seconds - secondsMotor >= vibrateTime) {
    secondsMotor = seconds;
    vibrateTime = 0;
    analogWrite(motorPin, 0);
  }
}

void loop() {
  if (!client.connect(host, 8888)) {
    if (debug)
      Serial.println("Connection failed.");
    delay(5000);
    return;
  } else {
    client.println("ID:" + String(gunID));  // send id to GUI server
  }
  while (client.connected()) {  // if we are connected run all logic
    checktime();
    readInputs();
    if (client.available()) {
      String line = client.readStringUntil('\n');
      if (line != "") {
        if (debug)
          Serial.println("Response from server: " + line);
        // String txt = line;
        // String command[16];
        // if (txt.indexOf(':') != -1) {
        //   for (int i = 0; !txt.isEmpty(); i++) {
        //     command[i] = txt.substring(0, txt.indexOf(':'));
        //     txt.remove(0, txt.indexOf(':') + 1);
        //     if (debug) {
        //       Serial.println(String(i) + ":" + command[i]);
        //     }
        //   }
        // }
      }
    }
    delay(20);
  }
}
