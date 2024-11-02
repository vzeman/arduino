//#define WIFI_SSID "your wifi"
//#define WIFI_PASSWORD "wifi password"
#include "secrets.h"

#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <TimeLib.h>


void (*resetFunc)(void) = 0;



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

//***************** EEPROM addresses for persistent storage of values **************
#define ADDRESS_TmaxIN 0
#define ADDRESS_TmaxOUT 1
#define ADDRESS_ThomeRequired 2
#define ADDRESS_TrequiredOUT 3
#define ADDRESS_tempWaterRequiredHigh 4
#define ADDRESS_tempWaterRequiredDown 5


//************ SOLID STATE RELAYS - STEP MOTORS ***********
const int pin_step_motor_water_up = 10;      // Define SSR channel 1 pin
const int pin_step_motor_water_down = 11;    // Define SSR channel 2 pin
const int pin_step_motor_heating_up = 12;    // Define SSR channel 3 pin
const int pin_step_motor_heating_down = 13;  // Define SSR channel 4 pin

//************ SOLID STATE RELAYS - CERPADLO ***********
//TODO const int pin_water_pump = 9;

const int ZAP = LOW;   //relatka maju opacne spinanie, preto ZAP je low voltage
const int VYP = HIGH;  //relatka maju opacne spinanie, preto VYP je high voltage


//********** TEMPERATURES *****************
int TmaxIN = 45;         //max teplota na vstupe podlahovky
int TmaxOUT = 35;        //max teplota na vystupe podlahovky
int ThomeRequired = 23;  //pozadovana teplota v dome
float prevThome = 0;
int TrequiredOUT = ThomeRequired + 3;  //inicializacia vystupnej teploty podlahovky
int tempWaterRequiredHigh = 57;        //pozadovana horna teplota vody
int tempWaterRequiredDown = 40;        //pozadovana dolna teplota vody

//********** STEP UP/DOWN speed ***********
const int stepUp = 500;     //cas v milisekundach kolko posuvame krokove motory smerom hore
const int stepDown = 1000;  //cas v milisekundach kolko posuvame krokove motory smerom dolu - dolu chladime rychlejsie ako hore


//********** TIME control ******************
int currentTime = 0;
const int delayBetweenChanges = 30;
const int delayBetweenWaterChanges = 5;
const int delayBetweenHomeChanges = 3600;


//*********************** WIFI *************************
#include <SPI.h>
#include <WiFiNINA.h>


WiFiServer server(80);
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
  return Tout > TmaxOUT + 2 || Tin > TmaxIN + 2;
}

void move(int time, int pinNr) {
  switchOffStepMotors();
  delay(50);
  digitalWrite(pinNr, ZAP);
  delay(time);
  switchOffStepMotors();
}

void tempUp() {
  move(stepUp, pin_step_motor_heating_up);
}

void tempDown() {
  move(stepDown, pin_step_motor_heating_down);
}

void tempWaterUp(void) {
  move(stepUp, pin_step_motor_water_up);
}

void tempWaterDown() {
  move(stepDown, pin_step_motor_water_down);
}

void switchOffStepMotors(void) {
  digitalWrite(pin_step_motor_water_up, VYP);      // Set SSR channel 1 pin LOW to disable output
  digitalWrite(pin_step_motor_water_down, VYP);    // Set SSR channel 2 pin LOW to disable output
  digitalWrite(pin_step_motor_heating_up, VYP);    // Set SSR channel 3 pin LOW to disable output
  digitalWrite(pin_step_motor_heating_down, VYP);  // Set SSR channel 4 pin LOW to disable output
}

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
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

int initFromEEPROM(int address, int defaultValue, int minValue, int maxValue) {
  int value = EEPROM.read(address);
  if (value < minValue || value > maxValue) {
    return defaultValue;
  }
  return value;
}

void storeValuesToEEPROM(void) {
  saveEEPROMValue(ADDRESS_ThomeRequired, ThomeRequired);
  saveEEPROMValue(ADDRESS_TmaxIN, TmaxIN);
  saveEEPROMValue(ADDRESS_TrequiredOUT, TrequiredOUT);
  saveEEPROMValue(ADDRESS_TmaxOUT, TmaxOUT);
  saveEEPROMValue(ADDRESS_tempWaterRequiredHigh, tempWaterRequiredHigh);
  saveEEPROMValue(ADDRESS_tempWaterRequiredDown, tempWaterRequiredDown);
}

void saveEEPROMValue(int address, int value) {
  if (value != EEPROM.read(address)) {
    EEPROM.write(address, value);
  }
}

void printControlValue(WiFiClient client, char name[], float value, char urlPrefix[]) {
  client.print("<tr style='background-color:lightgrey;padding-top:3px;'><td> ");
  client.print(name);
  client.print(" </td><td>");
  client.print(" <a class='btn btn-primary' href=\"/");
  client.print(urlPrefix);
  client.print("/UP\"> UP </a> ");
  client.print(value);
  client.print(" <a class='btn btn-primary' href=\"/");
  client.print(urlPrefix);
  client.print("/DOWN\"> Down </a> ");
  client.print("</td></tr>");
}

void printStatusValue(WiFiClient client, char name[], float value) {
  client.print("<tr style='background-color:lightgrey;padding-top:3px;'><td> ");
  client.print(name);
  client.print(" </td><td> ");
  client.print(value);
  client.print(" </td></tr>");
}

void printStatusString(WiFiClient client, char name[], String value) {
  client.print("<tr style='background-color:lightgrey;padding-top:3px;'><td> ");
  client.print(name);
  client.print(" </td><td> ");
  client.print(value);
  client.print(" </td></tr>");
}

boolean notificationSent = false;
String last_notification = "";





float Thome;
float Tin;
float Tout;
float TwaterTop;
float TwaterBottom;



void setup(void) {
  //init values after restart from EEPROM or default
  TmaxIN = initFromEEPROM(ADDRESS_TmaxIN, 45, 10, 49);
  TmaxOUT = initFromEEPROM(ADDRESS_TmaxOUT, 35, 10, 37);
  ThomeRequired = initFromEEPROM(ADDRESS_ThomeRequired, 23, 10, 26);  //pozadovana teplota v dome
  prevThome = getAnalogTemperature(THERMISTORPIN);
  TrequiredOUT = initFromEEPROM(ADDRESS_TrequiredOUT, ThomeRequired + 3, 10,
                                TmaxOUT);                                             //inicializacia vystupnej teploty podlahovky
  tempWaterRequiredHigh = initFromEEPROM(ADDRESS_tempWaterRequiredHigh, 57, 10, 70);  //pozadovana horna teplota vody
  tempWaterRequiredDown = initFromEEPROM(ADDRESS_tempWaterRequiredDown, 40, 10, 65);  //pozadovana dolna teplota vody

  storeValuesToEEPROM();


  Serial.begin(9600);
  Serial.println("Booting");

  sensorWaterTop.begin();
  sensorWaterBottom.begin();
  sensorHeatingIn.begin();
  sensorHeatingOut.begin();

  pinMode(pin_step_motor_water_up, OUTPUT);      // Define SSR channel 1 pin as output
  pinMode(pin_step_motor_water_down, OUTPUT);    // Define SSR channel 2 pin as output
  pinMode(pin_step_motor_heating_up, OUTPUT);    // Define SSR channel 3 pin as output
  pinMode(pin_step_motor_heating_down, OUTPUT);  // Define SSR channel 4 pin as output
  switchOffStepMotors();


  // Create open network. Change this line if you want to create an WEP network:
  while (status != WL_CONNECTED) {
    status = WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    delay(5000);
  }

  Serial.println("Wifi connected");


  // start the web server on port 80
  server.begin();

  // you're connected now, so print out the status
  printWiFiStatus();

  if (WiFi.getTime() != 0) {
    setTime(WiFi.getTime());
  }

  notificationSent = false;
}



void loop(void) {

  Serial.println("Loop start");
  if (WiFi.getTime() != 0) {
    setTime(WiFi.getTime());
  }
  Serial.println(String(hour()) + ":" + String(minute()) + ":" + String(second()));

  sensorWaterTop.requestTemperatures();
  sensorWaterBottom.requestTemperatures();
  sensorHeatingIn.requestTemperatures();
  sensorHeatingOut.requestTemperatures();
  delay(1000);
  Thome = getAnalogTemperature(THERMISTORPIN);
  Tin = sensorHeatingIn.getTempCByIndex(0);
  Tout = sensorHeatingOut.getTempCByIndex(0);
  TwaterTop = sensorWaterTop.getTempCByIndex(0);
  TwaterBottom = sensorWaterBottom.getTempCByIndex(0);
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
    if (currentTime % delayBetweenChanges == 0) {
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
      Serial.println(currentTime % delayBetweenChanges);
    }
  }

  if (currentTime % delayBetweenHomeChanges == 0) {
    if (Thome > ThomeRequired && TrequiredOUT > ThomeRequired && prevThome <= Thome) {
      TrequiredOUT -= 1;
      Serial.print("!!!!!!!!!!   Decreasing out temperature: ");
    } else if (TmaxOUT < TrequiredOUT && ((Thome < (ThomeRequired - 0.2) && prevThome >= Thome) || Thome < (ThomeRequired - 0.5))) {
      TrequiredOUT += 1;
      Serial.print("!!!!!!!!!!   Increasing out temperature: ");
    } else {
      Serial.print("!!!!!!!!!!   Keep out temperature: ");
    }
    Serial.println(TrequiredOUT);
    prevThome = Thome;
  }
  Serial.print("Current required out temperature is ");
  Serial.println(TrequiredOUT);


  Serial.println("- WATER ---------------------");
  Serial.print("Top: ");
  Serial.println(TwaterTop);
  Serial.print("Bottom: ");
  Serial.println(TwaterBottom);


  //fix errors in sensor data - This is critical error !!!
  if (TwaterBottom < 0 && TwaterTop > 0) {
    Serial.print("CRITICAL ERROR: Bottom Water Temperature sensor indicates negative value!!!");
    TwaterBottom = TwaterTop - (tempWaterRequiredHigh - tempWaterRequiredDown);
  }
  if (TwaterTop < 0 && TwaterBottom > 0) {
    Serial.print("CRITICAL ERROR: Top Water Temperature sensor indicates negative value!!!");
    TwaterTop = TwaterBottom + (tempWaterRequiredHigh - tempWaterRequiredDown);
  }

  if (currentTime % delayBetweenWaterChanges == 0) {
    if (TwaterTop > tempWaterRequiredHigh || TwaterBottom > tempWaterRequiredDown) {
      Serial.println("Water - cool down");
      tempWaterDown();
    } else if (tempWaterRequiredHigh - 1 > TwaterTop || tempWaterRequiredDown - 1 > TwaterBottom) {
      Serial.println("Water - increase temp");
      tempWaterUp();
    } else {
      Serial.println("Water - All set");
    }
  } else {
    Serial.print("Next water run in ");
    Serial.println(currentTime % delayBetweenWaterChanges);
  }


  Serial.println("********************************");
  Serial.println("");

  if (currentTime % 3600 == 0) {
    storeValuesToEEPROM();
  }

  if (currentTime % 30000 == 0) {
    notificationSent = false;
  }

  currentTime++;

  //*************** WIFI web server **********************

  // compare the previous status to the current status

  // compare the previous status to the current status
  if (status != WiFi.status()) {
    // it has changed update the variable
    status = WiFi.status();

    if (status == WL_AP_CONNECTED) {
      // a device has connected to the AP
      Serial.println("Device connected to AP");
    } else {
      // a device has disconnected from the AP, and we are back in listening mode
      Serial.println("Device disconnected from AP");
    }
  }


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

            client.print("<link href='https://maxcdn.bootstrapcdn.com/bootstrap/4.0.0-alpha.6/css/bootstrap.min.css' rel='stylesheet'/>");
            client.print("</br></br><a class='btn btn-primary' href=\"/\"> REFRESH </a> ");

            client.print("<table style='border: 1px solid black;'>");
            printStatusValue(client, "Current Home Temperature", Thome);
            printControlValue(client, "Requested Home Temperature", ThomeRequired, "HomeReq");

            printStatusValue(client, "Current Heating IN", Tin);
            printControlValue(client, "MAX Heating IN", TmaxIN, "HeatingMaxIn");

            printStatusValue(client, "Current Heating OUT", Tout);
            printControlValue(client, "Required Heating OUT", TrequiredOUT, "HeatingReqOut");
            printControlValue(client, "MAX Heating OUT", TmaxOUT, "HeatingMaxOut");

            printStatusValue(client, "Current Water Top", TwaterTop);
            printControlValue(client, "Required Water TOP", tempWaterRequiredHigh, "WaterReqTop");
            printStatusValue(client, "Current Water Bottom", TwaterBottom);
            printControlValue(client, "Required Water Bottom", tempWaterRequiredDown, "WaterReqBottom");
            printStatusValue(client, "Current Loop Nr", currentTime);
            printStatusString(client, "Current Time", String(hour()) + ":" + String(minute()) + ":" + String(second()));
            printStatusString(client, "Last message", last_notification);
            client.print("</table>");
            client.print("</br></br><a class='btn btn-primary' href=\"/reboot\"> REBOOT </a> ");
            
            client.println();
            // break out of the while loop:
            break;
          } else {  // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        //Handle requests
        if (currentLine.endsWith("GET /HomeReq/UP")) {
          ThomeRequired += 1;
          storeValuesToEEPROM();
        } else if (currentLine.endsWith("GET /HomeReq/DOWN")) {
          ThomeRequired -= 1;
          storeValuesToEEPROM();

        } else if (currentLine.endsWith("GET /HeatingMaxIn/UP")) {
          TmaxIN += 1;
          storeValuesToEEPROM();
        } else if (currentLine.endsWith("GET /HeatingMaxIn/DOWN")) {
          TmaxIN -= 1;
          storeValuesToEEPROM();

        } else if (currentLine.endsWith("GET /HeatingReqOut/UP")) {
          TrequiredOUT += 1;
          storeValuesToEEPROM();
        } else if (currentLine.endsWith("GET /HeatingReqOut/DOWN")) {
          TrequiredOUT -= 1;
          storeValuesToEEPROM();

        } else if (currentLine.endsWith("GET /HeatingMaxOut/UP")) {
          TmaxOUT += 1;
          storeValuesToEEPROM();
        } else if (currentLine.endsWith("GET /HeatingMaxOut/DOWN")) {
          TmaxOUT -= 1;
          storeValuesToEEPROM();

        } else if (currentLine.endsWith("GET /WaterReqTop/UP")) {
          tempWaterRequiredHigh += 1;
          storeValuesToEEPROM();
        } else if (currentLine.endsWith("GET /WaterReqTop/DOWN")) {
          tempWaterRequiredHigh -= 1;
          storeValuesToEEPROM();

        } else if (currentLine.endsWith("GET /WaterReqBottom/UP")) {
          tempWaterRequiredDown += 1;
          storeValuesToEEPROM();
        } else if (currentLine.endsWith("GET /WaterReqBottom/DOWN")) {
          tempWaterRequiredDown -= 1;
          storeValuesToEEPROM();
        } else if (currentLine.endsWith("GET /reboot")) {
          Serial.println("rebooting...");
          storeValuesToEEPROM();
          client.stop();
          resetFunc();
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
}
