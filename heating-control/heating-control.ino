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
int ThomeRequired = 24;  //pozadovana teplota v dome
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
const int delayBetweenHomeChanges = 600;  // 10 minutes for responsive control

//********** TEMPERATURE HISTORY for better control ******************
#define HISTORY_SIZE 4
float homeHistory[HISTORY_SIZE] = {0, 0, 0, 0};
int historyIndex = 0;
boolean historyInitialized = false;

//********** THERMAL CAPACITY TRACKING ******************
float thermalTimeConstant = 0;      // Minutes to reach 63% of target (tau)
float heatLossRate = 0;              // Degrees per hour when not heating
float heatingEfficiency = 0;         // Temperature rise per degree of heating delta
unsigned long lastThermalCalc = 0;
#define THERMAL_CALC_INTERVAL 3600   // Recalculate every hour


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

void calculateThermalCharacteristics(void) {
  // Only calculate if we have enough history
  if (!historyInitialized) {
    return;
  }

  // Get oldest temperature reading
  int oldestIndex = historyIndex;
  float oldestTemp = homeHistory[oldestIndex];
  float tempChange = Thome - oldestTemp;
  float timeSpanHours = (HISTORY_SIZE * delayBetweenHomeChanges) / 3600.0;

  // Calculate heating delivery (proxy for heat input)
  float heatingDelta = Tin - Tout;  // Heat being delivered to floor

  // Calculate temperature change rate (degrees per hour)
  float tempChangeRate = tempChange / timeSpanHours;

  // Estimate heat loss rate (when heating is minimal)
  if (heatingDelta < 2.0 && abs(tempChangeRate) > 0.01) {
    // House is cooling down or maintaining with minimal heating
    heatLossRate = abs(tempChangeRate);
    Serial.print("Heat loss rate: ");
    Serial.print(heatLossRate);
    Serial.println(" C/hour");
  }

  // Estimate heating efficiency (how much house warms per degree of heating delta)
  if (heatingDelta > 5.0 && abs(tempChangeRate) > 0.01) {
    // Actively heating
    heatingEfficiency = tempChangeRate / heatingDelta;
    Serial.print("Heating efficiency: ");
    Serial.print(heatingEfficiency);
    Serial.println(" C/hour per degree delta");
  }

  // Estimate thermal time constant (tau)
  // tau = how long to reach 63% of steady state
  // For a simple model: tau ~ thermal_mass / heat_loss_coefficient
  if (heatLossRate > 0.01) {
    float steadyStateTemp = ThomeRequired;
    float tempError = steadyStateTemp - oldestTemp;
    if (abs(tempError) > 0.5) {
      // Time constant estimation based on exponential response
      // T(t) = T_ss - (T_ss - T0) * e^(-t/tau)
      // Rough estimate: tau ~ time / ln((T0-T_ss)/(T-T_ss))
      float ratio = abs(tempError) / abs(ThomeRequired - Thome);
      if (ratio > 1.01) {  // Avoid log of values near 1
        thermalTimeConstant = (timeSpanHours * 60) / log(ratio);
        Serial.print("Thermal time constant: ");
        Serial.print(thermalTimeConstant);
        Serial.println(" minutes");
      }
    }
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

  // Initialize temperature history with current reading
  for (int i = 0; i < HISTORY_SIZE; i++) {
    homeHistory[i] = prevThome;
  }

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
      if (Tout > TrequiredOUT + 1) {
        //Output too high - cool down
        Serial.println("Cooling heating - output too high");
        tempDown();
      } else if (Tin > TmaxIN) {
        //Input too high - safety cooling
        Serial.println("Cooling heating - input too high");
        tempDown();
      } else if (TrequiredOUT - 0.2 > Tout && Tin < TmaxIN && Tout < TmaxOUT) {
        //Heating - output below target and safe to increase
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

  // Update temperature history
  homeHistory[historyIndex] = Thome;
  historyIndex = (historyIndex + 1) % HISTORY_SIZE;
  if (historyIndex == 0) {
    historyInitialized = true;  // We've filled the array at least once
  }

  if (currentTime % delayBetweenHomeChanges == 0) {
    // Calculate thermal characteristics every hour
    if (currentTime % THERMAL_CALC_INTERVAL == 0) {
      calculateThermalCharacteristics();
    }

    // Calculate temperature error (positive = too cold, negative = too warm)
    float homeError = ThomeRequired - Thome;

    // Calculate rate of change (positive = warming, negative = cooling)
    float homeRate = Thome - prevThome;

    // Calculate trend over full history if available
    float tempTrend = 0;
    if (historyInitialized) {
      int oldestIndex = historyIndex;  // Next position is the oldest
      float timeSpanHours = (HISTORY_SIZE * delayBetweenHomeChanges) / 3600.0;
      tempTrend = (Thome - homeHistory[oldestIndex]) / timeSpanHours;  // C per hour
      Serial.print("Temperature trend over ");
      Serial.print(HISTORY_SIZE * delayBetweenHomeChanges / 60);
      Serial.print(" min: ");
      Serial.print(tempTrend);
      Serial.println(" C/hour");
    }

    // Proportional control with rate damping
    float adjustment = 0;
    const float deadband = 0.2;  // Don't adjust if within 0.2°C of target

    if (abs(homeError) > deadband) {
      // Proportional term: bigger error = bigger adjustment (2x multiplier)
      adjustment = homeError * 2.0;

      // Derivative term: if changing in right direction, slow down adjustment
      // This prevents overshoot
      adjustment -= homeRate * 1.5;

      // Clamp adjustment to reasonable range (-2 to +2 degrees)
      if (adjustment > 2.0) adjustment = 2.0;
      if (adjustment < -2.0) adjustment = -2.0;

      Serial.print("Home temp control - Error: ");
      Serial.print(homeError);
      Serial.print("C, Rate: ");
      Serial.print(homeRate);
      Serial.print("C, Adjustment: ");
      Serial.println(adjustment);

      TrequiredOUT += adjustment;

      // Ensure we stay within safe limits
      if (TrequiredOUT < ThomeRequired) TrequiredOUT = ThomeRequired;
      if (TrequiredOUT > TmaxOUT) TrequiredOUT = TmaxOUT;

      Serial.print("!!!!!!!!!!   Adjusted out temperature to: ");
    } else {
      Serial.print("!!!!!!!!!!   Temperature within deadband, keeping out temperature: ");
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

            // Temperature History
            client.print("</br><h5>Temperature History (Last ");
            client.print(HISTORY_SIZE);
            client.print(" readings)</h5>");
            client.print("<table style='border: 1px solid black;'>");
            client.print("<tr style='background-color:#4CAF50;color:white;'><th>Reading</th><th>Home Temp (C)</th></tr>");

            // Display history in reverse order (newest first)
            for (int i = 0; i < HISTORY_SIZE; i++) {
              int displayIndex = (historyIndex - 1 - i + HISTORY_SIZE) % HISTORY_SIZE;
              if (homeHistory[displayIndex] != 0 || historyInitialized) {
                client.print("<tr style='background-color:lightgrey;'><td>");
                if (i == 0) {
                  client.print("Latest");
                } else {
                  client.print("-");
                  client.print(i * (delayBetweenHomeChanges / 60));
                  client.print(" min");
                }
                client.print("</td><td>");
                client.print(homeHistory[displayIndex]);
                client.print("</td></tr>");
              }
            }
            client.print("</table>");

            // Thermal Characteristics
            client.print("</br><h5>House Thermal Characteristics</h5>");
            client.print("<table style='border: 1px solid black;'>");
            client.print("<tr style='background-color:#2196F3;color:white;'><th>Metric</th><th>Value</th><th>Meaning</th></tr>");

            // Heating Delta
            float heatingDelta = Tin - Tout;
            client.print("<tr style='background-color:lightgrey;'><td>Heat Delivery</td><td>");
            client.print(heatingDelta);
            client.print(" C</td><td>");
            if (heatingDelta > 5) {
              client.print("Actively heating");
            } else if (heatingDelta > 2) {
              client.print("Moderate heating");
            } else {
              client.print("Minimal heating");
            }
            client.print("</td></tr>");

            // Temperature trend
            if (historyInitialized) {
              int oldestIndex = historyIndex;
              float timeSpanHours = (HISTORY_SIZE * delayBetweenHomeChanges) / 3600.0;
              float tempTrend = (Thome - homeHistory[oldestIndex]) / timeSpanHours;
              client.print("<tr style='background-color:lightgrey;'><td>Temp Change Rate</td><td>");
              client.print(tempTrend);
              client.print(" C/h</td><td>");
              if (tempTrend > 0.1) {
                client.print("Warming up");
              } else if (tempTrend < -0.1) {
                client.print("Cooling down");
              } else {
                client.print("Stable");
              }
              client.print("</td></tr>");
            }

            // Heat loss rate
            if (heatLossRate > 0) {
              client.print("<tr style='background-color:lightgrey;'><td>Heat Loss Rate</td><td>");
              client.print(heatLossRate);
              client.print(" C/h</td><td>");
              if (heatLossRate > 1.0) {
                client.print("Poor insulation");
              } else if (heatLossRate > 0.5) {
                client.print("Average insulation");
              } else {
                client.print("Good insulation");
              }
              client.print("</td></tr>");
            }

            // Heating efficiency
            if (heatingEfficiency != 0) {
              client.print("<tr style='background-color:lightgrey;'><td>Heating Efficiency</td><td>");
              client.print(heatingEfficiency, 4);
              client.print("</td><td>C/h per degree delta</td></tr>");
            }

            // Thermal time constant
            if (thermalTimeConstant > 0) {
              client.print("<tr style='background-color:lightgrey;'><td>Time Constant (tau)</td><td>");
              client.print(thermalTimeConstant);
              client.print(" min</td><td>");
              if (thermalTimeConstant > 180) {
                client.print("High thermal mass");
              } else if (thermalTimeConstant > 60) {
                client.print("Medium thermal mass");
              } else {
                client.print("Low thermal mass");
              }
              client.print("</td></tr>");
            }

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
