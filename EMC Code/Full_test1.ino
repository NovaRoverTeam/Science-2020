#include "math.h"
#include <Wire.h>
#include "Adafruit_VEML6075.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include "Adafruit_CCS811.h"


//define objects

Adafruit_VEML6075 uv = Adafruit_VEML6075();
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
Adafruit_CCS811 ccs;

//define pins

const int NG = A1;
const int CO = A0;
const int CORelayPin = 6;
const int COVoltPin = 7;


//initialise data output variables
int NGRaw;
int NGppm;
int CORaw;
int COppm;



//Set variables for CO sensor (MQ7) voltage cycle
unsigned long startMillis;
unsigned long switchTimeMillis;
const int CO_5V_Interval = 60000; //60s for 5V interval
const int CO_1_5V_Interval = 90000; //90s for 1.5V interval //constant overflow??????????????
bool heaterInHighPhase;




//Declare functions-------------------------------------------------------------------

int NG_ppm(double rawValue) {

  double ppm = 10.938 * exp(1.7742 * (rawValue * 3.3 / 4095));
  return ppm;
}



void configureVEML(void)
{
  // Set the integration constant
  uv.setIntegrationTime(VEML6075_100MS);
  // Get the integration constant and print it!
  Serial.print("Integration time set to 100ms");
  switch (uv.getIntegrationTime()) {
    case VEML6075_50MS: Serial.print("50"); break;
    case VEML6075_100MS: Serial.print("100"); break;
    case VEML6075_200MS: Serial.print("200"); break;
    case VEML6075_400MS: Serial.print("400"); break;
    case VEML6075_800MS: Serial.print("800"); break;
  }
  Serial.println("ms");
  // Set the high dynamic mode
  uv.setHighDynamic(true);
  // Get the mode
  if (uv.getHighDynamic()) {
    Serial.println("High dynamic reading mode");
  } else {
    Serial.println("Normal dynamic reading mode");
  }

  // Set the mode
  uv.setForcedMode(false);
  // Get the mode
  if (uv.getForcedMode()) {
    Serial.println("Forced reading mode");
  } else {
    Serial.println("Continuous reading mode");
  }
  // Set the calibration coefficients
  uv.setCoefficients(2.22, 1.33,  // UVA_A and UVA_B coefficients
                     2.95, 1.74,  // UVB_C and UVB_D coefficients
                     0.001461, 0.002591); // UVA and UVB responses
}

//Functions to configure sensors VEML/TSL
void configureTSL(void)
{
  /* You can also manually set the gain or enable auto-gain support */
  // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */

  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */

  /* Update these values depending on what you've set above! */
  Serial.println("------------------------------------");
  Serial.print  ("Gain:         "); Serial.println("Auto");
  Serial.print  ("Timing:       "); Serial.println("13 ms");
  Serial.println("------------------------------------");
}




//Functions to switch heater voltage on MQ7 (CO) sensor
void turnHeaterHigh() {
  // 5v phase
  digitalWrite(COVoltPin, LOW);
  digitalWrite(CORelayPin, HIGH);

  heaterInHighPhase = true;
  switchTimeMillis = millis() + CO_5V_Interval;
}

void turnHeaterLow() {
  // 1.4v phase
  digitalWrite(COVoltPin, HIGH);
  digitalWrite(CORelayPin, LOW);

  heaterInHighPhase = false;
  switchTimeMillis = millis() + CO_1_5V_Interval;
}

//Function to read CO sensor voltage (just before switching to 1.5V)
int measureCOSensor() {
  unsigned int gasLevel = analogRead(CO);
  unsigned int time = (millis() - startMillis) / 1000;
  delay(time);

  return gasLevel;
}

int CO_ppm(double rawValue){
    
    double ppm = 3.027*exp(1.0698*(rawValue*3.3/4095));
    return ppm;
}



//setup -------------------------------------------------------------------
void setup() {
  Serial.begin(9600);

  //Initialize CO sensor heater pins
  pinMode(CORelayPin, OUTPUT);
  pinMode(COVoltPin, OUTPUT);

  //Set start time (for CO sensor heater voltage)
  startMillis = millis();
  turnHeaterHigh();


  //VEML
  Serial.println("VEML6075 Full Test");
  if (! uv.begin()) {
    Serial.println("Failed to communicate with VEML6075 sensor, check wiring?");
  }
  Serial.println("Found VEML6075 sensor");

  //configure VEML
  configureVEML();

  //CCS
  if (!ccs.begin()) {
    Serial.println("Failed to start sensor! Please check your wiring.");
    while (1);
  }


  //TSL
  Serial.println("Light Sensor Test"); Serial.println("");
  /* Initialise the sensor */
  //use tsl.begin() to default to Wire,
  //tsl.begin(&Wire2) directs api to use Wire2, etc.
  if (!tsl.begin())
  {
    /* There was a problem detecting the TSL2561 ... check your connections */
    Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  /* Setup the sensor gain and integration time */
  configureTSL();
  /* We're ready to go! */
  Serial.println("Found TSL2561 sensor");




}

//--------------------------------------------------------------------
void loop() {

  //VEML data
  Serial.print("Raw UVA reading:  "); Serial.println(uv.readUVA());
  Serial.print("Raw UVB reading:  "); Serial.println(uv.readUVB());
  Serial.print("UV Index reading: "); Serial.println(uv.readUVI());


  //TSL data
  /* Get a new sensor event */
  sensors_event_t event;
  tsl.getEvent(&event);
  /* Display the results (light is measured in lux) */
  if (event.light)
  {
    Serial.print("TSL =>"); Serial.print(event.light); Serial.println(" lux");
  }
  else
  {
    /* If event.light = 0 lux the sensor is probably saturated
       and no reliable data could be generated! */
    Serial.println("TSL sensor overload");
  }


  //CCS
  if (ccs.available()) {
    if (!ccs.readData()) {
      Serial.print("CCS =>");
      Serial.print("CO2: ");      Serial.print(ccs.geteCO2());
      Serial.print("ppm, TVOC: ");      Serial.println(ccs.getTVOC());
    }
    else {
      Serial.println("CCS ERROR!");
      while (1);
    }
  }


  //MQ4 data
  NGRaw = analogRead(NG);
  NGppm = NG_ppm(NGRaw);

  //print values
  Serial.print("NG Raw value=> ");
  Serial.println(NGRaw);
  Serial.print("NG ppm = ");
  Serial.println(NGppm);


  //MQ7 data
  //Cycle CO sensor (MQ7) heater voltage

  if (heaterInHighPhase) {
    // 5V phase - check to switch
    if (millis() > switchTimeMillis) {
      turnHeaterLow();
    }
  }
  else {
    // 1.4V phase - check to switch
    if (millis() > switchTimeMillis) {
      turnHeaterHigh();
    }
  }
  CORaw = measureCOSensor();
  COppm = CO_ppm(CORaw);
  Serial.print("CO ppm = ");
  Serial.println(COppm);

  
  //wait
  delay(1000);
}
