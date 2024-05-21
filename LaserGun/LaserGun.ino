#include <IRremote.hpp>

#include <WiFi.h>
#include <WiFiClient.h>

WiFiClient client;

#define RECEIVER_PIN 26    // Pin connected to the IR receiver
#define TRANSMITTER_PIN 2  // Pin connected to the IR transmitter
#define SHOOT_PIN 9        // Pin connected to the shoot button
#define RELOAD_PIN 27      // Pin connected to the reload button
#define motorPin 25        // Pin connected to te vibration motor
#define redPin 10          // Pin connected to te vibration motor
#define greenPin 5         // Pin connected to te vibration motor
#define bluePin 13         // Pin connected to te vibration motor

const char* ssid = "Lasergame";
const char* password = "avans-01";
const char* host = "192.168.4.4";  // IP address of your Java server
const int gunID = 1;

const bool debug = true;

int DELAY_BETWEEN_REPEAT = 50;
int maxAmmo = 12;
int ammo = 12;


bool ledState = true;
float seconds;
float secondsLed;
float secondsMotor;
float vibrateTime, blinkTime;


int blinkRed;
int blinkGreen;
int blinkBlue;


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

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  led(0, 0, 0);

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
    if (debug && protocol == 8) {
      IrReceiver.printIRResultShort(&Serial);
    }
    if (protocol == 8 && command == 0x01 && address != gunID) {  //check for correct protcol and correct command
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
    motor(0.5, 255);
    delay(1);
  }
  Serial.println(ammo);
}


void hit(int address) {
  if (client.connected()) {
    motor(0.5, 255);
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

// void blink_led() {
//   if (health != maxHealth && health != 0 && health != -1) {
//     if (seconds - secondsLed >= float(health) / float(maxHealth)) {
//       secondsLed = seconds;
//       if (ledState == HIGH) {
//         ledState = LOW;
//         led(0, 1, 0);
//       } else {
//         ledState = HIGH;
//         led(0, 0, 0);
//       }
//     }
//   }
// }


void ledblink(float time, int red, int green, int blue) {
  if (time != 1)
    blinkTime = time;
  else
    blinkTime = 0;
  blinkRed = red;
  blinkGreen = green;
  blinkBlue = blue;
}

void led(int red, int green, int blue) {
  if (!ledState) {
    red = 0;
    green = 0;
    blue = 0;
  }
  red = 256 - red;
  green = 256 - green;
  blue = 256 - blue;
  if (red > 256) {
    red = 255;
  }
  if (green > 256) {
    green = 255;
  }
  if (blue > 256) {
    blue = 255;
  }
  if (red <= 0) {
    red = 0;
  }
  if (green <= 0) {
    green = 0;
  }
  if (blue <= 0) {
    blue = 0;
  }
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);
}

void checktime() {
  delay(50);
  // Serial.println(seconds);
  // Serial.println(secondsMotor);
  // Serial.println(vibrateTime);
  if (seconds - secondsMotor >= vibrateTime) {
    secondsMotor = seconds;
    vibrateTime = 0;
    analogWrite(motorPin, 0);
    // Serial.println("stop motor");
  }

  if (blinkTime == 0) {
    ledState = true;
  } else if (seconds - secondsLed >= blinkTime) {
    secondsLed = seconds;
    if (ledState == HIGH) {
      ledState = false;
      led(blinkRed, blinkGreen, blinkBlue);
    } else {
      ledState = true;
      led(blinkRed, blinkGreen, blinkBlue);
    }
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
        String txt = line;
        String command[16];
        char deliminer = ',';
        for (int i = 0; !txt.isEmpty(); i++) {
          if (txt.indexOf(deliminer) != -1) {
            command[i] = txt.substring(0, txt.indexOf(deliminer));
            txt.remove(0, txt.indexOf(deliminer) + 1);
          } else {
            command[i] = txt;
            txt = "";
          }
          if (debug) {
            Serial.println(String(i) + deliminer + command[i]);
          }
          if (command[0] == "ledblink") {
            ledblink(command[1].toFloat(), command[2].toInt(), command[3].toInt(), command[4].toInt());
          }
          if (command[0] == "led") {
            led(command[1].toInt(), command[2].toInt(), command[3].toInt());
            blinkTime = 0;
          }
        }
      }
    }
  }
  // delay(20);
}
