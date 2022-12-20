//TODO - dorobit safety switch
//TODO UI
//TODO riadenie teploty aj na zaklade teploty v dome



#include <OneWire.h>
#include <DallasTemperature.h>

//******** Temperature sensors *****************************
#define ONE_WIRE_BUS_Water_Top 6     //modra kratka
#define ONE_WIRE_BUS_Water_Bottom 7  //modra dlha
#define ONE_WIRE_BUS_Heating_In 8    //zltozelena paska dlha
#define ONE_WIRE_BUS_Heating_Out 9   //zltozelena paska kratka

OneWire oneWireWaterTop(ONE_WIRE_BUS_Water_Top);
OneWire oneWireWaterBottom(ONE_WIRE_BUS_Water_Bottom);
OneWire oneWireHeatingIn(ONE_WIRE_BUS_Heating_In);
OneWire oneWireHeatingOut(ONE_WIRE_BUS_Heating_Out);

DallasTemperature sensorWaterTop(&oneWireWaterTop);
DallasTemperature sensorWaterBottom(&oneWireWaterBottom);
DallasTemperature sensorHeatingIn(&oneWireHeatingIn);
DallasTemperature sensorHeatingOut(&oneWireHeatingOut);


//**************** HOME thermistor measurement ****************
// which analog pin to connect
#define THERMISTORPIN A0
// resistance at 25 degrees C
#define THERMISTORNOMINAL 10000
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 5
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
#define SERIESRESISTOR 10000


//************ RELAYS  STEP MOTORS ***********
const int channel1_pin = 10;  // Define SSR channel 1 pin
const int channel2_pin = 11;  // Define SSR channel 2 pin
const int channel3_pin = 12;  // Define SSR channel 3 pin
const int channel4_pin = 13;  // Define SSR channel 4 pin

const int ZAP = LOW;   //relatka maju opacne spinanie, preto ZAP je low voltage
const int VYP = HIGH;  //relatka maju opacne spinanie, preto VYP je high voltage


//********** TEMPERATURES *****************
const int TmaxIN = 45;                 //max teplota na vstupe podlahovky
const int TmaxOUT = 35;                //max teplota na vystupe podlahovky
int ThomeRequired = 23;          //pozadovana teplota v dome
int TrequiredOUT = ThomeRequired + 3;  //inicializacia vystupnej teploty podlahovky

const int tempWaterRequiredHigh = 57;  //pozadovana horna teplota vody
const int tempWaterRequiredDown = 40;  //pozadovana dolna teplota vody

//********** STEP UP/DOWN speed ***********
const int stepUp = 500;     //cas v milisekundach kolko posuvame krokove motory smerom hore
const int stepDown = 1000;  //cas v milisekundach kolko posuvame krokove motory smerom dolu - dolu chladime rychlejsie ako hore


//********** TIME control ******************
int currentTime = 0;
int lastChangeTime = 0;
int lastChangeWaterTime = 0;
int lastChangeHome = 0;
const int delayBetweenChanges = 30;
const int delayBetweenWaterChanges = 5;
const int delayBetweenHomeChanges = 600;


//*********************** WIFI *************************
#include <SPI.h>
#include <WiFiNINA.h>
//#include "arduino_secrets.h"
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = "xxxxxxxx";    // your network SSID (name)
char pass[] = "xxxxxxxx";  // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;          // your network key Index number (needed only for WEP)
WiFiServer server(80);

int wifiConnected = 0;
int status = WL_IDLE_STATUS;



float getAnalogTemperature(int pinNr) {

  float average = 0;
  // take N samples in a row, with a slight delay
  for (int i = 0; i < NUMSAMPLES; i++) {
    average += analogRead(pinNr);
    delay(10);
  }
  average /= NUMSAMPLES;
  // convert the value to resistance
  average = 1023 / average - 1;
  average = SERIESRESISTOR / average;

  float steinhart;
  steinhart = average / THERMISTORNOMINAL;           // (R/Ro)
  steinhart = log(steinhart);                        // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                         // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15);  // + (1/To)
  steinhart = 1.0 / steinhart;                       // Invert
  steinhart -= 273.15;                               // convert absolute temp to C

  return steinhart;
}

boolean isSafetyTemperature(float Tin, float Tout) {
  return
    //TODO digitalRead(pinSafetySwitch) == HIGH ||
    Tout > TmaxOUT + 2 || Tin > TmaxIN + 2;
}

void move(int time, int pinNr) {
  switchOffStepMotors();
  delay(50);
  digitalWrite(pinNr, ZAP);
  delay(time);
  switchOffStepMotors();
}

void tempUp() {
  lastChangeTime = currentTime;
  move(stepUp, channel3_pin);
}

void tempDown() {
  lastChangeTime = currentTime;
  move(stepDown, channel4_pin);
}

void tempWaterUp(void) {
  lastChangeWaterTime = currentTime;
  move(stepUp, channel1_pin);
}

void tempWaterDown() {
  lastChangeWaterTime = currentTime;
  move(stepDown, channel2_pin);
}


void resetTime() {
  currentTime = 0;
  lastChangeTime = 0;
  lastChangeWaterTime = 0;
  lastChangeHome = 0;
}

void switchOffStepMotors(void) {
  digitalWrite(channel1_pin, VYP);  // Set SSR channel 1 pin LOW to disable output
  digitalWrite(channel2_pin, VYP);  // Set SSR channel 2 pin LOW to disable output
  digitalWrite(channel3_pin, VYP);  // Set SSR channel 3 pin LOW to disable output
  digitalWrite(channel4_pin, VYP);  // Set SSR channel 4 pin LOW to disable output
}

void printWiFiStatus() {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}

void setup(void) {

  Serial.begin(9600);
  sensorWaterTop.begin();
  sensorWaterBottom.begin();
  sensorHeatingIn.begin();
  sensorHeatingOut.begin();

  pinMode(channel1_pin, OUTPUT);  // Define SSR channel 1 pin as output
  pinMode(channel2_pin, OUTPUT);  // Define SSR channel 2 pin as output
  pinMode(channel3_pin, OUTPUT);  // Define SSR channel 3 pin as output
  pinMode(channel4_pin, OUTPUT);  // Define SSR channel 4 pin as output
  switchOffStepMotors();



  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
  } else {
    // by default the local IP address of will be 192.168.4.1
    // you can override it with the following:
    // WiFi.config(IPAddress(10, 0, 0, 1));
    Serial.print("Creating access point named: ");
    Serial.println(ssid);
    status = WiFi.beginAP(ssid, pass);

    if (status != WL_AP_LISTENING) {
      Serial.println("Creating access point failed");
    } else {
      // wait 10 seconds for connection:
      delay(10000);
      // start the web server on port 80
      server.begin();
      // you're connected now, so print out the status
      printWiFiStatus();
      wifiConnected = 1;
    }
  }
}

void loop(void) {
  sensorWaterTop.requestTemperatures();
  sensorWaterBottom.requestTemperatures();
  sensorHeatingIn.requestTemperatures();
  sensorHeatingOut.requestTemperatures();
  delay(1000);
  float Thome = getAnalogTemperature(THERMISTORPIN);
  float Tin = sensorHeatingIn.getTempCByIndex(0);
  float Tout = sensorHeatingOut.getTempCByIndex(0);
  float TwaterTop = sensorWaterTop.getTempCByIndex(0);
  float TwaterBottom = sensorWaterBottom.getTempCByIndex(0);
  Serial.println("- HOME ---------------------");
  Serial.println(Thome);
  Serial.println("- HEATING ---------------------");
  Serial.print("I: ");
  Serial.println(Tin);
  Serial.print("O: ");
  Serial.println(Tout);


  if (isSafetyTemperature(Tin, Tout)) {
    //safety temperature reached, cool down
    Serial.println("Cooling heating - safety");
    tempDown();
  } else {
    if ((currentTime - lastChangeTime) > delayBetweenChanges) {
      //standard control loop
      if (Tout > TrequiredOUT + 1 || Tin > TmaxIN) {
        //Cooling
        Serial.println("Cooling heating");
        if (Tin > TrequiredOUT + 1.5) {
          tempDown();
        }
      } else if (TrequiredOUT - 0.2 > Tout && Tin < TmaxIN && Tout < TmaxOUT) {
        //Heating
        Serial.println("Increasing heating");
        tempUp();
      } else {
        Serial.println("Heating set");
      }
    } else {
      Serial.print("Next heating run in ");
      Serial.println(delayBetweenChanges - (currentTime - lastChangeTime));
    }
  }

  if ((currentTime - lastChangeHome) > delayBetweenHomeChanges) {
    if (Thome > ThomeRequired) {
      if (TrequiredOUT > ThomeRequired) {
        TrequiredOUT -= 1;
        lastChangeHome = currentTime;
        Serial.print("!!!!!!!!!!   Decrease out temperature to ");
        Serial.println(TrequiredOUT);
      }
    } else if (Thome < ThomeRequired - 0.5) {
      if (TmaxOUT < TrequiredOUT) {
        TrequiredOUT += 1;
        lastChangeHome = currentTime;
        Serial.print("!!!!!!!!!!   Increase out temperature to ");
        Serial.println(TrequiredOUT);
      }
    }
  }
  Serial.print("Current required out temperature is ");
  Serial.println(TrequiredOUT);



  Serial.println("- WATER ---------------------");
  Serial.print("Top: ");
  Serial.println(TwaterTop);
  Serial.print("Bottom: ");
  Serial.println(TwaterBottom);

  if ((currentTime - lastChangeWaterTime) > delayBetweenWaterChanges) {
    if (TwaterTop > tempWaterRequiredHigh || TwaterBottom > tempWaterRequiredDown) {
      Serial.println("Water - cool down");
      tempWaterDown();
    } else if (tempWaterRequiredHigh - 3 > TwaterTop || tempWaterRequiredDown - 5 > TwaterBottom) {
      Serial.println("Water - increase temp");
      tempWaterUp();
    } else {
      Serial.println("Water - All set");
    }
  } else {
    Serial.print("Next water run in ");
    Serial.println(delayBetweenWaterChanges - (currentTime - lastChangeWaterTime));
  }


  Serial.println("********************************");
  Serial.println("");

  currentTime++;
  if (currentTime > 1000000) {
    resetTime();
  }

  //*************** WIFI web server **********************

  // compare the previous status to the current status

  if (status != WiFi.status()) {
    // it has changed update the variable
    status = WiFi.status();
    if (status == WL_AP_CONNECTED) {
      // a device has connected to the AP
      Serial.println("Device connected to AP");
      wifiConnected = 1;
    } else {
      // a device has disconnected from the AP, and we are back in listening mode
      Serial.println("Device disconnected from AP");
      wifiConnected = 0;
    }
  }

  if (wifiConnected) {
    WiFiClient client = server.available();  // listen for incoming clients
    if (client) {                            // if you get a client,
      Serial.println("new client");          // print a message out the serial port
      String currentLine = "";               // make a String to hold incoming data from the client
      while (client.connected()) {           // loop while the client's connected
        if (client.available()) {            // if there's bytes to read from the client,
          char c = client.read();            // read a byte, then
          Serial.write(c);                   // print it out the serial monitor
          if (c == '\n') {                   // if the byte is a newline character
            // if the current line is blank, you got two newline characters in a row.
            // that's the end of the client HTTP request, so send a response:
            if (currentLine.length() == 0) {
              // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
              // and a content-type so the client knows what's coming, then a blank line:
              client.println("HTTP/1.1 200 OK");
              client.println("Content-type:text/html");
              client.println();
              // the content of the HTTP response follows the header:
              client.print("Home Temperature <a href=\"/H\">UP</a><br><br><a href=\"/L\">Down</a><br><br><a href=\"/\">Refresh</a>");
              client.print("<br><br><br>");
              client.print("<br>Home required temp: ");
              client.print(ThomeRequired);
              client.print("<br>Home current temp: ");
              client.print(Thome);
              client.print("<br>Podlahovka IN temp: ");
              client.print(Tin);
              client.print("<br>Podlahovka OUT temp: ");
              client.print(Tout);              // The HTTP response ends with another blank line:
              client.print("<br>Voda TOP temp: ");
              client.print(TwaterTop);              // The HTTP response ends with another blank line:
              client.print("<br>Voda BOTTOM temp: ");
              client.print(TwaterBottom);              // The HTTP response ends with another blank line:
              client.println();
              // break out of the while loop:
              break;
            } else {  // if you got a newline, then clear currentLine:
              currentLine = "";
            }
          } else if (c != '\r') {  // if you got anything else but a carriage return character,
            currentLine += c;      // add it to the end of the currentLine
          }
          // Check to see if the client request was "GET /H" or "GET /L":
          if (currentLine.endsWith("GET /H")) {
            ThomeRequired += 1;
          }
          if (currentLine.endsWith("GET /L")) {
            ThomeRequired -= 1;
          }
        }
      }
      // close the connection:
      client.stop();
      Serial.println("client disconnected");
    }
  }
}
