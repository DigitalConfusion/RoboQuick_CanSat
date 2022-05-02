/* ----------- Required libaries --------------*/
#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_SGP30.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_PM25AQI.h>
#include <SparkFun_SCD30_Arduino_Library.h>
#include <TeensyThreads.h>
#include <TinyGPSPlus.h>
#include <SD.h>
#include <Entropy.h>
#include <TimeLib.h>
#include <DS1307RTC.h>

/* ---------------- Lora ----------------- */
// LoRa pins
// Don't change these
const int csPin = 24;
const int resetPin = 26;
const int irqPin = 25;

// Frquency for LoRa in Hz
const long freq = 4337E5; // 433.7 MHz

/* ----------------- GPS ------------------*/
// Gps object
TinyGPSPlus gps;

// Configuration for GPS to update at 5 times a second
const unsigned char UBLOX_INIT[] PROGMEM = {
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A, //(5Hz)
                                                                                        // 0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39 //(1Hz)
};

/* --------------- SD Card ---------------- */
File myFile;
String file_name;

/* --------------- Sensors ---------------- */
// Sensor power pins
const int sensor_board_power = 22;

// Mics2714 variables
const int mics_input = 21;
const int mics_power = 20;
double mics_voltage = 0.0;
float no2_ppm;

// SGP30 object
Adafruit_SGP30 sgp;

// SGP30 Variables
float tvoc, eco2;

// PMS Object
Adafruit_PM25AQI pms = Adafruit_PM25AQI();

// Object to store data from PMS
PM25_AQI_Data pms_data;

// PMS variables
float pm10_std, pm25_std, pm100_std, p03, p05, p10, p25, p50, p100;

// SCD30 object
SCD30 scd;

// SCD30 variables
float co2;
float temp_scd30 = 25.0;
float humidity_scd30 = 35.0;

// Constants for NTC sensor
const int resistance_def = 10000; // Resistance for the sensor
const int constant_B = 3977;      // Sensors constant for temperature
const float t_25_kelvin = 298.15; // 25 degree temperature in kelvins
const float VCC = 3.3;            // Teensy voltage

// Variables for NTC sensor
float ntc_analog_value, voltage_diff, ntc_resistance, ln_value, temp_thermo;

// Sensor object for BMP280
Adafruit_BMP280 bmp;

// Variables for BMP280
float temp_bmp = 0;
float pressure = 0;
float altitude = 0;
float highest_altitude = 0;
unsigned long highest_alt_time = 0;
const float ambient_pressure = 1010.3;

/* ----------------------- Time ----------------------------- */
// Time zone offset
const int offset = 3;

// Variables to store time
tmElements_t tm;

// Variable to check if correct time has been set to internal clock from GPS
bool time_is_set = false;

/* ---------------- Miscellaneous variables ----------------- */
// Variables fot timing
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
const long interval = 1000; // 1 second

// How many steps are in analogRead, it differs from original 1024,
// because later on resolution gets changed from 8 to 12 bits
int analog_steps;

// Beeper power pin
const int beeper_power = 30; // Change this to correct pin!!!!

/* ---------------------- Functions  -------------------------*/
// Converts relative humidity to absolute humidity
uint32_t getAbsoluteHumidity(float temperature, float humidity)
{
  // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
  const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
  const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity);                                                                // [mg/m^3]
  return absoluteHumidityScaled;
}

// Function that updates gps data
void get_gps_data()
{
  if (digitalRead(sensor_board_power) == HIGH)
  {
    while (Serial1.available())
    {
      gps.encode(Serial1.read());
    }
  }
}

// Function that turns on and begins communication with all senors
void turn_on_all_sensors()
{
  // Turn on sensor board
  digitalWrite(sensor_board_power, HIGH);
  Serial.println("Sensor board power is ON. Waiting 2 second for sensors to turn on!");
  delay(2000);

  // Begin communication with SCD30 sensor
  while (!scd.begin(Wire1))
  {
    Serial.println("SCD30 not found. This warning should only show up once!");
    delay(1000);
  }
  Serial.println("SCD30 started");
  LoRa.beginPacket();
  LoRa.println("SCD30 started");
  LoRa.endPacket();

  // Begin communication with PMSA003I sensor
  while (!pms.begin_I2C(&Wire2))
  {
    Serial.println("PMS not found. It usually takes a few tries to turn on!");
    delay(1000);
  }
  Serial.println("PMS started");
  LoRa.beginPacket();
  LoRa.println("PMS started");
  LoRa.endPacket();

  // Begin communication with SGP30 sensor
  while (!sgp.begin(&Wire2))
  {
    Serial.println("SGP not found");
    delay(100);
  }
  LoRa.beginPacket();
  LoRa.println("SGP started");
  LoRa.endPacket();
  // Loads sensor's baseline from EEPROM
  /*
    int e_base = 0;
    int t_base = 0;
    EEPROM.get(0, e_base);
    EEPROM.get(100, t_base);
    sgp.setIAQBaseline(e_base, t_base);
  */
  Serial.println("SGP started");

  // Turn on Mics-2714
  digitalWrite(mics_power, LOW);
  Serial.println("Mics-2714 turned on");
  LoRa.beginPacket();
  LoRa.println("MICS started");
  LoRa.endPacket();

  // Begin communication with the GPS module
  Serial1.begin(9600);
  /*
  // Send configuration data in UBX protocol
  for (unsigned int i = 0; i < sizeof(UBLOX_INIT); i++)
  {
    Serial1.write(pgm_read_byte(UBLOX_INIT + i));
  }
  */
  LoRa.beginPacket();
  LoRa.println("GPS started");
  LoRa.endPacket();

  Serial.println("GPS, SCD30, PMSA003I, Mics-2714 and SGP30 sensors are working!");
  LoRa.beginPacket();
  LoRa.println("GPS, SCD30, PMSA003I, Mics-2714 and SGP30 sensors are working!");
  LoRa.endPacket();
}

// Turns off all sensors and gps
void turn_off_all_sensors()
{
  digitalWrite(sensor_board_power, LOW);
  digitalWrite(mics_power, HIGH);
  Serial.println("Sensors and gps turned off!");
  LoRa.beginPacket();
  LoRa.println("Sensors and gps turned off!");
  LoRa.endPacket();
}

void write_to_sd()
{
  String dataString = "";

  dataString += String(tm.Hour);
  dataString += String(":");
  dataString += String(tm.Minute);
  dataString += String(":");
  dataString += String(tm.Second);
  dataString += String(",");
  dataString += String(gps.location.lat());
  dataString += String(",");
  dataString += String(gps.location.lng());
  dataString += String(",");
  dataString += String(gps.altitude.meters());
  dataString += String(",");
  dataString += String(gps.satellites.value());
  dataString += String(",");
  dataString += String(altitude);
  dataString += String(",");
  dataString += String(temp_bmp);
  dataString += String(",");
  dataString += String(temp_thermo);
  dataString += String(",");
  dataString += String(temp_scd30);
  dataString += String(",");
  dataString += String(humidity_scd30);
  dataString += String(",");
  dataString += String(eco2);
  dataString += String(",");
  dataString += String(tvoc);
  dataString += String(",");
  dataString += String(pm10_std);
  dataString += String(",");
  dataString += String(pm25_std);
  dataString += String(",");
  dataString += String(pm100_std);
  dataString += String(",");
  dataString += String(co2);
  dataString += String(",");
  dataString += String(no2_ppm);

  // Opens a file
  File myFile = SD.open(file_name.c_str(), FILE_WRITE);

  // if the file is available, write to it:
  if (myFile) {
    myFile.println(dataString);
    myFile.close();
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("Error opening file");
  }
}

void write_sd_log_header()
{
  String header = "";
  header = "Time,Lattitude,Longitude,GPSAltitude,Satellites,AltitudeBMP,TempBMP,TempThermoresistor,TempSCD,Humidity,eCO2,TVOC,PM10,PM25,PM100,CO2,NO2";
  File myFile = SD.open(file_name.c_str(), FILE_WRITE);
  // if the file is available, write to it:
  if (myFile) {
    myFile.println(header);
    myFile.close();
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("Error opening file");
  }
}

// Beeps and sends location, when Cansat has landed
void recovery_mode()
{
  while (true)
  {
    // Beep
    digitalWrite(beeper_power, HIGH);
    delay(1000);
    digitalWrite(beeper_power, LOW);
    // Transmit location
    LoRa.beginPacket();
    LoRa.print(gps.location.lat(), 6);
    LoRa.print(",");
    LoRa.println(gps.location.lng(), 6);
    LoRa.endPacket();
    delay(5000);
  }
}

// Function checks if Cansat has landed and puts cansat in power saving and recovery mode
void check_if_landed()
{
  // Check if altitude has been higher than 300m, that means that launch was successful
  // then check that Cansat is falling down, and then we can be sure that after 5 minutes
  // Cansat must have landed and stayed still
  if (highest_altitude > 400 && altitude < 200 && millis() - highest_alt_time > 300000)
  {
    recovery_mode();
  }
}

/* ---------------------- Setup -----------------------*/
void setup()
{
  // Set sensor power pins to output
  pinMode(sensor_board_power, OUTPUT);
  pinMode(mics_power, OUTPUT);

  // Set input pin from mics to input mode
  pinMode(mics_input, INPUT);
  // At the start set mics to off
  digitalWrite(mics_power, HIGH);

  // Start random number generator
  Entropy.Initialize();
  // Begin serial and i2c comunication.
  Serial.begin(9600);
  Serial.println("Starting CanSat");
  Wire.begin();
  Wire1.begin();
  Wire2.begin();
  Serial.println("I2C started");

  // Change analog resolution to 12 bits. That is from 0 to 4095
  analogReadResolution(12);
  analog_steps = 4096;

  // Sets up required pins for LoRa radio module
  LoRa.setPins(csPin, resetPin, irqPin);

  // Tries to start up LoRa module. If it doesn't start, goes into infinite loop
  if (!LoRa.begin(freq))
  {
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
  }
  
  // Settings for LoRa
  LoRa.setTxPower(20);
  LoRa.setSpreadingFactor(10);
  // LoRa.setSignalBandwidth(62.5E3);
  // LoRa.enableCrc();
  // LoRa.setGain(6);
  Serial.println("LoRa started");

  // see if the card is present and initialize it
  while (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("Initializing SD card failed! Trying again!");
    LoRa.beginPacket();
    LoRa.println("Initializing SD card failed! Trying again!");
    LoRa.endPacket();
    delay(1000);
  }
  file_name = "datalog" + String(Entropy.random(10000)) + ".csv";
  write_sd_log_header();
  Serial.println("SD card initialized!");
  LoRa.beginPacket();
  LoRa.println("SD card initialized!");
  LoRa.endPacket();
  
  // Starts BMP280 sensor
  bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  // Settings for BMP280 sensor
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,   /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,   /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X4,   /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X2,     /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */
  Serial.println("BMP280 found and started");

  // Turn on all sensors for testing purposes
  turn_on_all_sensors();

  Serial.println("Setup done");
}

/*------------------- Loop -------------------------*/
void loop()
{
  // Check if new GPS data is available
  get_gps_data();
  
  if (gps.time.isValid() && !time_is_set)
  {
    tm.Hour = gps.time.hour() + offset;
    tm.Minute = gps.time.minute();
    tm.Second = gps.time.second();
    tm.Day = gps.date.day();
    tm.Month = gps.date.month();
    tm.Year = gps.date.year();
    RTC.write(tm);
    time_is_set == true;
  }
  if (time_is_set)
  {
    RTC.read(tm);
  }
  
  // Checks if Cansat has landed
  check_if_landed();
  
  // Get time since turned on in miliseconds
  currentMillis = millis();
  
  // Run only if interval of time has passed
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;

    // If sensor board is on, get data from sensors
    if (digitalRead(sensor_board_power) == HIGH)
    {
      // If SCD30 has data ready, get the data
      if (scd.dataAvailable())
      {
        co2 = scd.getCO2();
        temp_scd30 = scd.getTemperature();
        humidity_scd30 = scd.getHumidity();
      }
      
      // Gets all readings from PMSA003I
      pms.read(&pms_data);
      pm10_std = pms_data.pm10_standard;
      pm25_std = pms_data.pm25_standard;
      pm100_std = pms_data.pm100_standard;
      p03 = pms_data.particles_03um;
      p05 = pms_data.particles_05um;
      p10 = pms_data.particles_10um;
      p25 = pms_data.particles_25um;
      p50 = pms_data.particles_50um;
      p100 = pms_data.particles_100um;
      
      // Sets SGP30 humidity from SCD30 humidity measurment, to get the most accurate readings
      sgp.setHumidity(getAbsoluteHumidity(temp_thermo, humidity_scd30));
      // Gets readings from SGP30
      sgp.IAQmeasure();
      tvoc = sgp.TVOC;
      eco2 = sgp.eCO2;
    }

    // Get data from BMP280 sensor
    temp_bmp = bmp.readTemperature();
    pressure = bmp.readPressure();
    altitude = bmp.readAltitude(ambient_pressure);
    // If higher altitude has been recorded, save it and get time when it happened
    if (altitude > highest_altitude)
    {
      highest_altitude = altitude;
      highest_alt_time = millis();
    }

    // Get temperature from NTC thermoresistor
    ntc_analog_value = (VCC / analog_steps) * analogRead(A10); // Analog reading from NTC
    voltage_diff = VCC - ntc_analog_value;
    ntc_resistance = ntc_analog_value / (voltage_diff / resistance_def); // Calculates the resistance of the sensor
    ln_value = log(ntc_resistance / resistance_def);                     // Calculates natural log value
    temp_thermo = (1 / ((ln_value / constant_B) + (1 / t_25_kelvin)));   // Temperature from sensor in kelvin
    temp_thermo = temp_thermo - 273.15 + 17.4;                           // Converts to celsius with correction offset

    // Get data from MICS2714 sensor
    if (digitalRead(mics_power) == LOW)
    {
      // Gets voltage from mics input
      mics_voltage = (analogRead(mics_input) * 3.3) / analog_steps;
      // Calculates no2 concentration in ppm, using graph from datasheet
      no2_ppm = ((3.3 - mics_voltage) / mics_voltage) / 6.667;
      }
    
    // Sends data to LoRa radio module
    LoRa.beginPacket();
    LoRa.print(tm.Hour);
    LoRa.print(":");
    LoRa.print(tm.Minute);
    LoRa.print(":");
    LoRa.print(tm.Second);
    LoRa.print(",");
    LoRa.print(gps.location.lat(), 6);
    LoRa.print(",");
    LoRa.print(gps.location.lng(), 6);
    LoRa.print(",");
    LoRa.print(gps.altitude.meters());
    LoRa.print(",");
    LoRa.print(gps.satellites.value());
    LoRa.print(",");
    LoRa.print(altitude);
    LoRa.print(",");
    LoRa.print(temp_bmp);
    LoRa.print(",");
    LoRa.print(temp_thermo);
    LoRa.print(",");
    LoRa.print(temp_scd30);
    LoRa.print(",");
    LoRa.print(humidity_scd30);
    LoRa.print(",");
    LoRa.print(eco2);
    LoRa.print(",");
    LoRa.print(tvoc);
    LoRa.print(",");
    LoRa.print(pm10_std);
    LoRa.print(",");
    LoRa.print(pm25_std);
    LoRa.print(",");
    LoRa.print(pm100_std);
    LoRa.print(",");
    LoRa.print(co2);
    LoRa.print(",");
    LoRa.println(no2_ppm);
    LoRa.endPacket();
  
    // Print all data to the serial console
    Serial.print(tm.Hour);
    Serial.print(":");
    Serial.print(tm.Minute);
    Serial.print(":");
    Serial.print(tm.Second);
    Serial.print(",");
    Serial.print(gps.location.lat(), 6);
    Serial.print(",");
    Serial.print(gps.location.lng(), 6);
    Serial.print(",");
    Serial.print(gps.altitude.meters());
    Serial.print(",");
    Serial.print(gps.satellites.value());
    Serial.print(",");
    Serial.print(altitude);
    Serial.print(",");
    Serial.print(temp_bmp);
    Serial.print(",");
    Serial.print(temp_thermo);
    Serial.print(",");
    Serial.print(temp_scd30);
    Serial.print(",");
    Serial.print(pressure);
    Serial.print(",");
    Serial.print(humidity_scd30);
    Serial.print(",");
    Serial.print(eco2);
    Serial.print(",");
    Serial.print(tvoc);
    Serial.print(",");
    Serial.print(pm10_std);
    Serial.print(",");
    Serial.print(pm25_std);
    Serial.print(",");
    Serial.print(pm100_std);
    Serial.print(",");
    Serial.print(co2);
    Serial.print(",");
    Serial.println(no2_ppm);
    
    // Write data to SD card log file
    write_to_sd();
  }
}
