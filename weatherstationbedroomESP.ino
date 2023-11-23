#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <BlynkSimpleEsp32.h>
#include "time.h"
//#include <PMserial.h> // Arduino library for PM sensors with serial interface
#include <FastLED.h>
#include "Adafruit_SHT31.h"
#include <Average.h>
#if defined(ARDUINO_ARCH_ESP32) || (ARDUINO_ARCH_ESP8266)
#include <EEPROM.h>
#define USE_EEPROM
#endif
#include <bsec2.h>
#include "config/bme680_iaq_33v_3s_28d/bsec_iaq.h"

#include "Plantower_PMS7003.h"
char output[256];
Plantower_PMS7003 pms7003 = Plantower_PMS7003();

#define LED_PIN 15  //d2
#define NUM_LEDS 3
CRGB leds[NUM_LEDS];
AsyncWebServer server(80);
Average<float> pm1Avg(30);
Average<float> pm25Avg(30);
Average<float> pm10Avg(30);
Average<float> pm1aAvg(30);
Average<float> pm25aAvg(30);
Average<float> pm10aAvg(30);
Average<float> wifiAvg(30);

Adafruit_SHT31 sht31 = Adafruit_SHT31();

//SerialPM pms(PMSx003, Serial); // PMSx003, UART

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -18000;  //Replace with your GMT offset (secs)
const int daylightOffset_sec = 0;   //Replace with your daylight offset (secs)
int hours, mins, secs;

char auth[] = "eT_7FL7IUpqonthsAr-58uTK_-su_GYy"; //BLYNK
char remoteAuth[] = "pO--Yj8ksH2fjJLMW6yW9trkHBhd9-wc";
char remoteAuth2[] = "8_-CN2rm4ki9P3i_NkPhxIbCiKd5RXhK";

const char* ssid = "mikesnet";
const char* password = "springchicken";

float old1p0, old2p5, old10, new1p0, new2p5, new10;
float old1p0a, old2p5a, old10a, new1p0a, new2p5a, new10a;
unsigned int up3, up5, up10, up25, up50, up100;
float abshumBME, tempBME, presBME, humBME, gasBME, dewpoint, humidex, tempSHT, humSHT, abshumSHT, dewpointSHT, humidexSHT;
float bmeiaq, bmeiaqAccuracy, bmestaticIaq, bmeco2Equivalent, bmebreathVocEquivalent, bmestabStatus, bmerunInStatus, bmegasPercentage;
int firstvalue = 1;
int blynkWait = 30000;
float bridgedata, bridgetemp, bridgehum, windbridgedata, windmps, winddir;
double windchill;

unsigned long lastmillis = 0;
unsigned long millisBlynk = 0;
unsigned long millisAvg = 0;
unsigned long deltamillis = 0;
unsigned long rgbmillis = 0;
int zebraR, zebraG, zebraB, sliderValue;
int menuValue = 2;
float  pmR, pmG, pmB;
bool rgbON = true;

WidgetTerminal terminal(V14); //terminal widget
WidgetBridge bridge1(V70);
WidgetBridge bridge2(V60);


#define STATE_SAVE_PERIOD UINT32_C(720 * 60 * 1000) /* 360 minutes - 4 times a day */
#define PANIC_LED 2
#define ERROR_DUR 1000



/* Helper functions declarations */
/**
 * @brief : This function toggles the led continuously with one second delay
 */
void errLeds(void);

/**
 * @brief : This function checks the BSEC status, prints the respective error code. Halts in case of error
 * @param[in] bsec  : Bsec2 class object
 */
void checkBsecStatus(Bsec2 bsec);

/**
 * @brief : This function updates/saves BSEC state
 * @param[in] bsec  : Bsec2 class object
 */
void updateBsecState(Bsec2 bsec);

/**
 * @brief : This function is called by the BSEC library when a new output is available
 * @param[in] input     : BME68X sensor data before processing
 * @param[in] outputs   : Processed BSEC BSEC output data
 * @param[in] bsec      : Instance of BSEC2 calling the callback
 */
void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec);

/**
 * @brief : This function retrieves the existing state
 * @param : Bsec2 class object
 */
bool loadState(Bsec2 bsec);

/**
 * @brief : This function writes the state into EEPROM
 * @param : Bsec2 class object
 */
bool saveState(Bsec2 bsec);


Bsec2 envSensor;
#ifdef USE_EEPROM
static uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE];
#endif
/* Gas estimate names will be according to the configuration classes used */
const String gasName[] = { "Field Air", "Hand sanitizer", "Undefined 3", "Undefined 4"};


BLYNK_CONNECTED() {
  bridge1.setAuthToken (remoteAuth);
  bridge2.setAuthToken (remoteAuth2);
}

BLYNK_WRITE(V61){
    float pinData = param.asFloat();
    bridgedata = pinData;
}

BLYNK_WRITE(V62){
    bridgetemp = param.asFloat();

}

BLYNK_WRITE(V63){
    bridgehum = param.asFloat();

}




BLYNK_WRITE(V15)
{
   menuValue = param.asInt(); // assigning incoming value from pin V1 to a variable
}

BLYNK_WRITE(V16)
{
     zebraR = param[0].asInt();
     zebraG = param[1].asInt();
     zebraB = param[2].asInt();
}



/*BLYNK_WRITE(V35)
{
    redboost = param.asInt();
}*/




void printLocalTime()
{
  time_t rawtime;
  struct tm * timeinfo;
  time (&rawtime);
  timeinfo = localtime (&rawtime);
  terminal.print(asctime(timeinfo));
  terminal.flush();
}

bool heater = false;

BLYNK_WRITE(V14)
{
    if (String("help") == param.asStr()) 
    {
    terminal.println("==List of available commands:==");
    terminal.println("wifi");
    terminal.println("temps");
    terminal.println("wets");
    terminal.println("particles");
    terminal.println("bsec");
    terminal.println("rapidon");
    terminal.println("rapidoff");
    terminal.println("heater");
     terminal.println("==End of list.==");
    }
        if (String("wifi") == param.asStr()) 
    {
        terminal.print("Connected to: ");
        terminal.println(WiFi.SSID());
        terminal.print("IP address:");
        terminal.println(WiFi.localIP());
        terminal.print("Signal strength: ");
        terminal.println(WiFi.RSSI());
        printLocalTime();
    }


    if (String("temps") == param.asStr()) {
        tempSHT = sht31.readTemperature();
        humSHT = sht31.readHumidity();
        terminal.print("tempBME[v0],tempPool[v5],humidex[v31],dewpoint[v2]: ");
        terminal.print(tempBME);
        terminal.print(",,,");
        terminal.print(humidex);
        terminal.print(",,,");
        terminal.println(dewpoint);
        terminal.print("tempSHT =");
        terminal.println(tempSHT);
        terminal.print("humSHT =");
        terminal.println(humSHT);
    }
    if (String("wets") == param.asStr()) {
        terminal.print("humBME[v3],abshumBME[v4],presBME[v1],gasBME[v7]: ");
        terminal.print(humBME);
        terminal.print(",,,");
        terminal.print(abshumBME);
        terminal.print(",,,");
        terminal.print(presBME);
        terminal.print(",,,");
        terminal.println(gasBME);
    }
    if (String("particles") == param.asStr()) {


        sprintf(output, "\nSensor Version: %d    Error Code: %d\n",
                      pms7003.getHWVersion(),
                      pms7003.getErrorCode());
        Serial.print(output);

        sprintf(output, "    PM1.0 (ug/m3): %2d     [atmos: %d]\n",
                      pms7003.getPM_1_0(),
                      pms7003.getPM_1_0_atmos());              
        Serial.print(output);
        sprintf(output, "    PM2.5 (ug/m3): %2d     [atmos: %d]\n",
                      pms7003.getPM_2_5(),
                      pms7003.getPM_2_5_atmos());
        Serial.print(output);
        sprintf(output, "    PM10  (ug/m3): %2d     [atmos: %d]\n",
                      pms7003.getPM_10_0(),
                      pms7003.getPM_10_0_atmos());              
        Serial.print(output);

        sprintf(output, "\n    RAW: %2d[>0.3] %2d[>0.5] %2d[>1.0] %2d[>2.5] %2d[>5.0] %2d[>10]\n",
                      pms7003.getRawGreaterThan_0_3(),
                      pms7003.getRawGreaterThan_0_5(),
                      pms7003.getRawGreaterThan_1_0(),
                      pms7003.getRawGreaterThan_2_5(),
                      pms7003.getRawGreaterThan_5_0(),
                      pms7003.getRawGreaterThan_10_0());
        Serial.print(output);
      

    }
    if (String("bsec") == param.asStr()) {
        terminal.print("bmeiaq[v23],bmeiaqAccuracy[v24],bmestaticIaq[v25],bmeco2Equivalent[v26],bmebreathVocEquivalent[v27],bmestabStatus[v28],bmerunInStatus[v29],bmegasPercentage[v30]:");
        terminal.print(bmeiaq);
        terminal.print(",,,");
        terminal.print(bmeiaqAccuracy);
        terminal.print(",,,");
        terminal.print(bmestaticIaq);
        terminal.print(",,,");
        terminal.print(bmeco2Equivalent);
        terminal.print(",,,");
        terminal.print(bmebreathVocEquivalent);
        terminal.print(",,,");
        terminal.print(bmestabStatus);
        terminal.print(",,,");
        terminal.print(bmerunInStatus);
        terminal.print(",,,");
        terminal.println(bmegasPercentage);
    }

    if (String("rapidon") == param.asStr()) {
        blynkWait = 50;
    }

    if (String("rapidoff") == param.asStr()) {
      blynkWait = 30000;
    }

    if (String("heater") == param.asStr()) {
        heater = !heater;
        sht31.heater(heater);
      terminal.print("> Heater is now: ");
      terminal.print(heater);
    }
    terminal.flush();

}

void errLeds(void)
{
    //while(1)
    //{
        digitalWrite(PANIC_LED, HIGH);
        delay(ERROR_DUR);
        digitalWrite(PANIC_LED, LOW);
        delay(ERROR_DUR);
    //}
}

void updateBsecState(Bsec2 bsec)
{
    static uint16_t stateUpdateCounter = 0;
    bool update = false;

    if (!stateUpdateCounter || (stateUpdateCounter * STATE_SAVE_PERIOD) < millis())
    {
        /* Update every STATE_SAVE_PERIOD minutes */
        update = true;
        stateUpdateCounter++;
    }

    if (update && !saveState(bsec))
        checkBsecStatus(bsec);
}

void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec)
{
    if (!outputs.nOutputs)
        return;

    Serial.println("BSEC outputs:\n\ttimestamp = " + String((int) (outputs.output[0].time_stamp / INT64_C(1000000))));
    for (uint8_t i = 0; i < outputs.nOutputs; i++)
    {
        const bsecData output  = outputs.output[i];
        switch (output.sensor_id)
        {
            case BSEC_OUTPUT_RAW_GAS:
                gasBME = (1 / (output.signal / 1000.0)) * 10;
                break;
            case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
                tempBME = output.signal;
                break;
            case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
                humBME = output.signal;
                break;
            case BSEC_OUTPUT_IAQ:
            bmeiaq = output.signal;
            bmeiaqAccuracy = output.accuracy;
                break;
            case BSEC_OUTPUT_STATIC_IAQ:
            bmestaticIaq = output.signal;
                break;
            case BSEC_OUTPUT_CO2_EQUIVALENT:
            bmeco2Equivalent = output.signal;
                break;
            case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
            bmebreathVocEquivalent = output.signal;
                break;
            case BSEC_OUTPUT_RAW_PRESSURE:
            presBME = (output.signal / 100.0);
                break;
            case BSEC_OUTPUT_STABILIZATION_STATUS:
            bmestabStatus = output.signal;
                break;
            case BSEC_OUTPUT_RUN_IN_STATUS:
            bmerunInStatus = output.signal;
                break;
            case BSEC_OUTPUT_GAS_PERCENTAGE:
            bmegasPercentage = output.signal;
                break;

            default:
                break;
        }
    }

    updateBsecState(envSensor);
}

void checkBsecStatus(Bsec2 bsec)
{
    if (bsec.status < BSEC_OK)
    {
        Serial.println("BSEC error code : " + String(bsec.status));
        errLeds(); /* Halt in case of failure */
    } else if (bsec.status > BSEC_OK)
    {
        Serial.println("BSEC warning code : " + String(bsec.status));
    }

    if (bsec.sensor.status < BME68X_OK)
    {
        Serial.println("BME68X error code : " + String(bsec.sensor.status));
        errLeds(); /* Halt in case of failure */
    } else if (bsec.sensor.status > BME68X_OK)
    {
        Serial.println("BME68X warning code : " + String(bsec.sensor.status));
    }
}

bool loadState(Bsec2 bsec)
{
#ifdef USE_EEPROM
    

    if (EEPROM.read(0) == BSEC_MAX_STATE_BLOB_SIZE)
    {
        /* Existing state in EEPROM */
        terminal.println("Reading state from EEPROM");
        terminal.print("State file: ");
        for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++)
        {
            bsecState[i] = EEPROM.read(i + 1);
            terminal.print(String(bsecState[i], HEX));
        }
        terminal.println();

        if (!bsec.setState(bsecState))
            return false;
    } else
    {
        /* Erase the EEPROM with zeroes */
        terminal.println("Erasing EEPROM");

        for (uint8_t i = 0; i <= BSEC_MAX_STATE_BLOB_SIZE; i++)
            EEPROM.write(i, 0);

        EEPROM.commit();
        terminal.flush();
    }
#endif
    return true;
}

bool saveState(Bsec2 bsec)
{
#ifdef USE_EEPROM
    if (!bsec.getState(bsecState))
        return false;

    terminal.println("Writing state to EEPROM");
    printLocalTime();
    terminal.print("State file: ");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++)
    {
        EEPROM.write(i + 1, bsecState[i]);
        terminal.print(String(bsecState[i], HEX));
    }
    terminal.println();
    terminal.flush();
    EEPROM.write(0, BSEC_MAX_STATE_BLOB_SIZE);
    EEPROM.commit();
#endif
    return true;
}

void readPMS() {
    if (pms7003.hasNewData()) {
        new1p0 = pms7003.getPM_1_0();
        new2p5 = pms7003.getPM_2_5();
        new10 = pms7003.getPM_10_0();
                if (firstvalue == 0)  //do not do this on the first run
        {
            if (new1p0 > 200) {new1p0 = old1p0;} //check for data spikes in particle counter, ignore data that is >200
            if (new2p5 > 200) {new2p5 = old2p5;} //data spikes ruin pretty graph
            if (new10 > 200) {new10 = old10;}
            if (new1p0 - old1p0 > 50) {new1p0 = old1p0;} //also ignore data that is >50 off from last data
            if (new2p5 - old2p5 > 50) {new2p5 = old2p5;}
            if (new10 - old10 > 50) {new10 = old10;}
        }
        pm1Avg.push(new1p0);
        pm25Avg.push(new2p5);
        pm10Avg.push(new10);
        old1p0 = new1p0; //reset data spike check variable
        old2p5 = new2p5;
        old10 = new10;

        new1p0a = pms7003.getPM_1_0_atmos(); 
        new2p5a = pms7003.getPM_2_5_atmos();
        new10a = pms7003.getPM_10_0_atmos();
                if (firstvalue == 0)  //do not do this on the first run
        {
            if (new1p0a > 200) {new1p0a = old1p0a;} //check for data spikes in particle counter, ignore data that is >200
            if (new2p5a > 200) {new2p5a = old2p5a;} //data spikes ruin pretty graph
            if (new10a > 200) {new10a = old10a;}
            if (new1p0a - old1p0a > 50) {new1p0a = old1p0a;} //also ignore data that is >50 off from last data
            if (new2p5a - old2p5a > 50) {new2p5a = old2p5a;}
            if (new10a - old10a > 50) {new10a = old10a;}
        }
        pm1aAvg.push(new1p0a);
        pm25aAvg.push(new2p5a);
        pm10aAvg.push(new10a);
        old1p0a = new1p0a; //reset data spike check variable
        old2p5a = new2p5a;
        old10a = new10a;        
       up3 = pms7003.getRawGreaterThan_0_3();
       up5 = pms7003.getRawGreaterThan_0_5();
       up10 = pms7003.getRawGreaterThan_1_0();
       up25 = pms7003.getRawGreaterThan_2_5();
       up50 = pms7003.getRawGreaterThan_5_0();
       up100 = pms7003.getRawGreaterThan_10_0();
    firstvalue = 0;
  }
}


void setup() {
  setCpuFrequencyMhz(80);
  Serial.begin(9600);
  Serial1.begin(9600, SERIAL_8N1, 3, 1);
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");
  sht31.begin(0x44);
  pms7003.init(&Serial1);
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    leds[0] = CRGB(100, 100, 0);
    FastLED.show();
    delay(250);
    leds[0] = CRGB(0, 0, 0);
    FastLED.show();
    delay(250);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
    leds[0] = CRGB(0, 100, 0);
    FastLED.show();
    delay(1000);
    leds[0] = CRGB(0, 0, 0);
    FastLED.show();
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    delay(250);
    struct tm timeinfo;
    getLocalTime(&timeinfo);
    hours = timeinfo.tm_hour;
    mins = timeinfo.tm_min;
    secs = timeinfo.tm_sec;
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! I am ESP32, weatherstationbedroomESP.");
  });

  AsyncElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();
  Serial.println("HTTP server started");
    Blynk.config(auth, IPAddress(192, 168, 50, 197), 8080);
    Blynk.connect();
  tempBME = sht31.readTemperature();
   humBME = sht31.readHumidity();


    bsecSensor sensorList[13] = {
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_STABILIZATION_STATUS,
    BSEC_OUTPUT_RUN_IN_STATUS,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
    BSEC_OUTPUT_GAS_PERCENTAGE
    };
  
    #ifdef USE_EEPROM
    EEPROM.begin(BSEC_MAX_STATE_BLOB_SIZE + 1);
  #endif
    Wire.begin();
    pinMode(PANIC_LED, OUTPUT);

    /* Valid for boards with USB-COM. Wait until the port is open */
    while (!Serial) delay(10);
   
    /* Initialize the library and interfaces */
    if (!envSensor.begin(BME68X_I2C_ADDR_LOW, Wire))
    {
        checkBsecStatus(envSensor);
    }

    /* Load the configuration string that stores information on how to classify the detected gas */
    if (!envSensor.setConfig(bsec_config_iaq))
    {
        checkBsecStatus (envSensor);
    }

    /* Copy state from the EEPROM to the algorithm */
    if (!loadState(envSensor))
    {
        checkBsecStatus (envSensor);
    }

    /* Subscribe for the desired BSEC2 outputs */
    if (!envSensor.updateSubscription(sensorList, 13, BSEC_SAMPLE_RATE_LP))
    {
        checkBsecStatus (envSensor);
    }

    /* Whenever new data is available call the newDataCallback function */
    envSensor.attachCallback(newDataCallback);

  String output = "\nBSEC library version " + String(envSensor.version.major) + "." + String(envSensor.version.minor) + "." + String(envSensor.version.major_bugfix) + "." + String(envSensor.version.minor_bugfix);
                //terminal.clear();
				terminal.println("----------------------------------");
				terminal.println("STARTING BEDROOM BLYNK SERVER v2.1");
                terminal.println(output);
				printLocalTime(); //print current time to Blynk terminal
                terminal.println("----------------------------------");
                
                terminal.println("Type 'help' for a list of commands");
				terminal.flush();
}

void loop() {
      if (!envSensor.run()) {
        checkBsecStatus (envSensor);
    }
    pms7003.updateFrame();
    readPMS();
  if (WiFi.status() == WL_CONNECTED) {Blynk.run();} 

  if  (millis() - millisAvg >= 1000)  //if it's been 1 second
    {
        wifiAvg.push(WiFi.RSSI());
        millisAvg = millis();
    }

  if  (millis() - millisBlynk >= blynkWait) //if it's been blynkWait seconds 
    {
        tempSHT = sht31.readTemperature();
        humSHT = sht31.readHumidity();
        millisBlynk = millis();
        abshumBME = (6.112 * pow(2.71828, ((17.67 * tempBME)/(tempBME + 243.5))) * humBME * 2.1674)/(273.15 + tempBME);
        abshumSHT = (6.112 * pow(2.71828, ((17.67 * tempSHT)/(tempSHT + 243.5))) * humSHT * 2.1674)/(273.15 + tempSHT);
        dewpoint = tempBME - ((100 - humBME)/5); //calculate dewpoint
        humidex = tempBME + 0.5555 * (6.11 * pow(2.71828, 5417.7530*( (1/273.16) - (1/(273.15 + dewpoint)) ) ) - 10); //calculate humidex using Environment Canada formula
        dewpointSHT = tempSHT - ((100 - humSHT)/5); //calculate dewpoint
        humidexSHT = tempSHT + 0.5555 * (6.11 * pow(2.71828, 5417.7530*( (1/273.16) - (1/(273.15 + dewpoint)) ) ) - 10); //calculate humidex using Environment Canada formula
        
        if ((tempBME <= 0) && (humBME <= 0)) {}
        else {
        Blynk.virtualWrite(V1, presBME);
        Blynk.virtualWrite(V2, tempBME);
        Blynk.virtualWrite(V3, humBME);
        Blynk.virtualWrite(V4, abshumBME);
        Blynk.virtualWrite(V31, humidex);
        Blynk.virtualWrite(V32, dewpoint);
        }
        Blynk.virtualWrite(V5, pm1Avg.mean());
        Blynk.virtualWrite(V6, pm25Avg.mean());
        bridge1.virtualWrite(V71, pm25Avg.mean());
        bridge1.virtualWrite(V72, bridgedata);
        bridge1.virtualWrite(V73, bridgetemp);
        bridge1.virtualWrite(V74, bridgehum);
        bridge2.virtualWrite(V71, pm25Avg.mean());
        bridge2.virtualWrite(V72, tempSHT);
        bridge2.virtualWrite(V73, humSHT);
        bridge2.virtualWrite(V74, abshumSHT);
        bridge2.virtualWrite(V75, bmeiaq);
        Blynk.virtualWrite(V7, pm10Avg.mean());
        Blynk.virtualWrite(V8, up3);
        Blynk.virtualWrite(V9, up5);
        Blynk.virtualWrite(V10, up10);
        Blynk.virtualWrite(V11, up25);
        Blynk.virtualWrite(V12, up50);
        Blynk.virtualWrite(V13, up100);
        Blynk.virtualWrite(V17, pm1aAvg.mean());
        Blynk.virtualWrite(V18, pm25aAvg.mean());
        Blynk.virtualWrite(V19, pm10aAvg.mean());
        Blynk.virtualWrite(V23, bmeiaq);
        Blynk.virtualWrite(V24, bmeiaqAccuracy);
        Blynk.virtualWrite(V25, bmestaticIaq);
        Blynk.virtualWrite(V26, bmeco2Equivalent);
        Blynk.virtualWrite(V27, bmebreathVocEquivalent);
        Blynk.virtualWrite(V28, bmestabStatus);
        Blynk.virtualWrite(V29, bmerunInStatus);
        Blynk.virtualWrite(V30, bmegasPercentage);

        Blynk.virtualWrite(V33, gasBME);
        Blynk.virtualWrite(V34, wifiAvg.mean());
        Blynk.virtualWrite(V36, tempSHT);
        Blynk.virtualWrite(V37, humSHT);
        Blynk.virtualWrite(V38, abshumSHT);
        Blynk.virtualWrite(V39, humidexSHT);
        Blynk.virtualWrite(V41, dewpointSHT);
    }
}
