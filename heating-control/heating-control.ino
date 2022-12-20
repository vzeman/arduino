//TODO - safety switch
//TODO add option to switch off Heating
//TODO add option to switch off water

#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>

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
int TmaxIN = 45;                 //max teplota na vstupe podlahovky
int TmaxOUT = 35;                //max teplota na vystupe podlahovky
int ThomeRequired = 23;                //pozadovana teplota v dome
float prevThome = 0;
int TrequiredOUT = ThomeRequired + 3;  //inicializacia vystupnej teploty podlahovky
int tempWaterRequiredHigh = 57;  //pozadovana horna teplota vody
int tempWaterRequiredDown = 40;  //pozadovana dolna teplota vody

//********** STEP UP/DOWN speed ***********
const int stepUp = 500;     //cas v milisekundach kolko posuvame krokove motory smerom hore
const int stepDown = 1000;  //cas v milisekundach kolko posuvame krokove motory smerom dolu - dolu chladime rychlejsie ako hore


//********** TIME control ******************
int currentTime = 0;
const int delayBetweenChanges = 30;
const int delayBetweenWaterChanges = 5;
const int delayBetweenHomeChanges = 1800;


//*********************** WIFI *************************
#include <SPI.h>
#include <WiFiNINA.h>

char ssid[] = "Heating0001";  // your network SSID (name)
char pass[] = "heatison";     // your network password (use for WPA, or use as key for WEP)
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
    move(stepUp, channel3_pin);
}

void tempDown() {
    move(stepDown, channel4_pin);
}

void tempWaterUp(void) {
    move(stepUp, channel1_pin);
}

void tempWaterDown() {
    move(stepDown, channel2_pin);
}

void switchOffStepMotors(void) {
    digitalWrite(channel1_pin, VYP);  // Set SSR channel 1 pin LOW to disable output
    digitalWrite(channel2_pin, VYP);  // Set SSR channel 2 pin LOW to disable output
    digitalWrite(channel3_pin, VYP);  // Set SSR channel 3 pin LOW to disable output
    digitalWrite(channel4_pin, VYP);  // Set SSR channel 4 pin LOW to disable output
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

#define ADDRESS_TmaxIN 0
#define ADDRESS_TmaxOUT 1
#define ADDRESS_ThomeRequired 2
#define ADDRESS_TrequiredOUT 3
#define ADDRESS_tempWaterRequiredHigh 4
#define ADDRESS_tempWaterRequiredDown 5

void setup(void) {
    //init values after restart from EEPROM or default
    TmaxIN = initFromEEPROM(ADDRESS_TmaxIN, 45, 30, 49);
    TmaxOUT = initFromEEPROM(ADDRESS_TmaxOUT, 35, 15, 38);
    ThomeRequired = initFromEEPROM(ADDRESS_ThomeRequired, 23, 10, 26);                //pozadovana teplota v dome
    prevThome = getAnalogTemperature(THERMISTORPIN);
    TrequiredOUT = initFromEEPROM(ADDRESS_TrequiredOUT, ThomeRequired + 3, 10,
                                  TmaxOUT);  //inicializacia vystupnej teploty podlahovky
    tempWaterRequiredHigh = initFromEEPROM(ADDRESS_tempWaterRequiredHigh, 57, 10, 70);  //pozadovana horna teplota vody
    tempWaterRequiredDown = initFromEEPROM(ADDRESS_tempWaterRequiredDown, 40, 10, 65);  //pozadovana dolna teplota vody

    storeValuesToEEPROM();


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


    Serial.println("Access Point Web Server");
    // check for the WiFi module:
    if (WiFi.status() == WL_NO_MODULE) {
        Serial.println("Communication with WiFi module failed!");
        // don't continue
        while (true);
    }


    String fv = WiFi.firmwareVersion();
    if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
        Serial.println("Please upgrade the firmware");
    }

    // print the network name (SSID);
    Serial.print("Creating access point named: ");
    Serial.println(ssid);

    // Create open network. Change this line if you want to create an WEP network:
    status = WiFi.beginAP(ssid, pass);
    if (status != WL_AP_LISTENING) {
        Serial.println("Creating access point failed");
        // don't continue
        while (true);
    }

    // wait 10 seconds for connection:
    delay(10000);

    // start the web server on port 80
    server.begin();

    // you're connected now, so print out the status
    printWiFiStatus();
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
                Serial.print("!!!!!!!!!!   Decrease out temperature to ");
                Serial.println(TrequiredOUT);
        } else if (Thome < (ThomeRequired - 0.5) && TmaxOUT < TrequiredOUT && prevThome >= Thome) {
                TrequiredOUT += 1;
                Serial.print("!!!!!!!!!!   Increase out temperature to ");
                Serial.println(TrequiredOUT);
        }
        prevThome = Thome;
    }
    Serial.print("Current required out temperature is ");
    Serial.println(TrequiredOUT);


    Serial.println("- WATER ---------------------");
    Serial.print("Top: ");
    Serial.println(TwaterTop);
    Serial.print("Bottom: ");
    Serial.println(TwaterBottom);

    if (currentTime % delayBetweenWaterChanges == 0) {
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
        Serial.println(currentTime % delayBetweenWaterChanges);
    }


    Serial.println("********************************");
    Serial.println("");

    if (currentTime % 3600 == 0) {
        storeValuesToEEPROM();
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
                        client.print("</table>");

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
                    EEPROM.write(ADDRESS_ThomeRequired, ThomeRequired);
                } else if (currentLine.endsWith("GET /HomeReq/DOWN")) {
                    ThomeRequired -= 1;
                    EEPROM.write(ADDRESS_ThomeRequired, ThomeRequired);

                } else if (currentLine.endsWith("GET /HeatingMaxIn/UP")) {
                    TmaxIN += 1;
                    EEPROM.write(ADDRESS_TmaxIN, TmaxIN);
                } else if (currentLine.endsWith("GET /HeatingMaxIn/DOWN")) {
                    TmaxIN -= 1;
                    EEPROM.write(ADDRESS_TmaxIN, TmaxIN);

                } else if (currentLine.endsWith("GET /HeatingReqOut/UP")) {
                    TrequiredOUT += 1;
                    EEPROM.write(ADDRESS_TrequiredOUT, TrequiredOUT);

                } else if (currentLine.endsWith("GET /HeatingReqOut/DOWN")) {
                    TrequiredOUT -= 1;
                    EEPROM.write(ADDRESS_TrequiredOUT, TrequiredOUT);

                } else if (currentLine.endsWith("GET /HeatingMaxOut/UP")) {
                    TmaxOUT += 1;
                    EEPROM.write(ADDRESS_TmaxOUT, TmaxOUT);
                } else if (currentLine.endsWith("GET /HeatingMaxOut/DOWN")) {
                    TmaxOUT -= 1;
                    EEPROM.write(ADDRESS_TmaxOUT, TmaxOUT);

                } else if (currentLine.endsWith("GET /WaterReqTop/UP")) {
                    tempWaterRequiredHigh += 1;
                    EEPROM.write(ADDRESS_tempWaterRequiredHigh, tempWaterRequiredHigh);
                } else if (currentLine.endsWith("GET /WaterReqTop/DOWN")) {
                    tempWaterRequiredHigh -= 1;
                    EEPROM.write(ADDRESS_tempWaterRequiredHigh, tempWaterRequiredHigh);

                } else if (currentLine.endsWith("GET /WaterReqBottom/UP")) {
                    tempWaterRequiredDown += 1;
                    EEPROM.write(ADDRESS_tempWaterRequiredDown, tempWaterRequiredDown);
                } else if (currentLine.endsWith("GET /WaterReqBottom/DOWN")) {
                    tempWaterRequiredDown -= 1;
                    EEPROM.write(ADDRESS_tempWaterRequiredDown, tempWaterRequiredDown);
                }
            }
        }
        // close the connection:
        client.stop();
        Serial.println("client disconnected");
    }
}

void storeValuesToEEPROM(void) {
    EEPROM.write(ADDRESS_ThomeRequired, ThomeRequired);
    EEPROM.write(ADDRESS_TmaxIN, TmaxIN);
    EEPROM.write(ADDRESS_TrequiredOUT, TrequiredOUT);
    EEPROM.write(ADDRESS_TmaxOUT, TmaxOUT);
    EEPROM.write(ADDRESS_tempWaterRequiredHigh, tempWaterRequiredHigh);
    EEPROM.write(ADDRESS_tempWaterRequiredDown, tempWaterRequiredDown);
}

void printControlValue(WiFiClient client, char name[], float value, char urlPrefix[]) {
    client.print("<tr style='background-color:lightgrey;padding-top:3px;'><td> ");
    client.print(name);
    client.print(" </td><td>");
    client.print(" <a href=\"/");
    client.print(urlPrefix);
    client.print("/UP\"> UP </a> ");
    client.print(value);
    client.print(" <a href=\"/");
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