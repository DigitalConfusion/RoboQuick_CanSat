/* Required libaries */
#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_SGP30.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_PM25AQI.h>
#include <SparkFun_SCD30_Arduino_Library.h>
#include <Adafruit_INA219.h>
#include <TeensyThreads.h>

// Sensor power pins
const int sensor_board_power = 20; // Set correct pin!

// Analog input sensors pins
const int mics_input = A15;
const int mics_power = 20; // Set correct pin!

// Mics2714 variables
float no2_ppm;

// LoRa pins
const int csPin = 24;
const int resetPin = 26;
const int irqPin = 25;

// Frquency for LoRa in Hz
const long freq = 4337E5; // 433.7 MHz

// SGP30 object
Adafruit_SGP30 sgp;

// SGP30 Variables
float tvoc, eco2;

// PMS Object
Adafruit_PM25AQI pms = Adafruit_PM25AQI();

// Place to store all data from the senosor
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

// Variables fot timing
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
const long interval = 1000; // 1 second

// How many steps are in analogRead, it differs from original 1024,
// because later on resolution gets changed from 8 to 12 bits
const int analog_steps = 4096;

// Converts relative humidity to absolute humidity
uint32_t getAbsoluteHumidity(float temperature, float humidity)
{
  // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
  const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
  const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity);                                                                // [mg/m^3]
  return absoluteHumidityScaled;
}

void turn_on_all_sensors()
{
  // Turn on sensor board
  digitalWrite(sensor_board_power, HIGH);
  Serial.println("Sensor board power is ON. Waiting 1 second for sensors to turn on!")
      delay(1000);

  // Begin communication with SCD30 sensor
  while (!scd.begin(Wire1))
  {
    Serial.println("SCD30 not found");
    delay(100);
  }
  Serial.println("SCD30 started");

  // Begin communication with PMSA003I sensor
  while (!pms.begin_I2C(&Wire2))
  {
    Serial.println("PMS not found");
    delay(100);
  }
  Serial.println("PMS started");

  // Begin communication with SGP30 sensor
  while (!sgp.begin(&Wire2))
  {
    Serial.println("SGP not found");
    delay(100);
  }
  // Loads sensor's baseline from EEPROM
  // TODO: Check if calibration is needed
  int e_base = 0;
  int t_base = 0;
  EEPROM.get(0, e_base);
  EEPROM.get(100, t_base);
  sgp.setIAQBaseline(e_base, t_base);
  Serial.println("SGP started");

  // Turn on Mics-2714
  digtalWrite(mics_power, LOW);
  Serial.println("Mics-2714 turned on");

  Serial.println("SCD30, PMSA003I, Mics-2714 and SGP30 sensors are working!");
  delay(1000);
}

// Runs only once
void setup()
{
  // Set sensor board power pin to output
  pinMode(sensor_board_power, OUTPUT);
  pinMode(mics_power, OUTPUT);
  pinMode(mics_input, INPUT);
  digtialWrite(mics_power, HIGH);
  // Begin serial comunication.
  Serial.begin(9600);
  Serial.println("Starting CanSat");
  Wire.begin();
  Wire1.begin();
  Wire2.begin();
  Serial.println("I2C started");

  // Change analog resolution to 12 bits. That is from 0 to 4095
  analogReadResolution(12);

  // Sets up required pins for LoRa radio module
  LoRa.setPins(csPin, resetPin, irqPin);

  // Tries to start up LoRa module. If it doesn't start, goes into infinite loop
  if (!LoRa.begin(freq))
  {
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
  }
  LoRa.setTxPower(20);
  LoRa.setSpreadingFactor(8);
  LoRa.setSignalBandwidth(62.5E3);
  LoRa.enableCrc();
  LoRa.setGain(6);
  Serial.println("LoRa started");

  // Starts BMP280 sensor
  bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  // Settings for BMP280 sensor
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,   /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,   /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X8,   /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X2,     /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */
  Serial.println("BMP280 found and started");

  // Turn on all sensors
  turn_on_all_sensors();

  // Delay to make sure everything has turned on
  delay(1000);
  Serial.println("Setup done");
}

// Loops
void loop()
{
  // Checks if a message has been received
  // onReceive(LoRa.parsePacket());

  // Get time since turned on in miliseconds
  currentMillis = millis();

  // Run only if interval of time has passed
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;

    // If SCD30 has data ready, get the data
    if (digitalRead(sensor_board_power) == HIGH)
    {
      if (scd.dataAvailable())
      {
        co2 = scd.getCO2();
        temp_scd30 = scd.getTemperature();
        humidity_scd30 = scd.getHumidity();
      }
    }

    // Get data from BMP280 sensor
    temp_bmp = bmp.readTemperature();
    pressure = bmp.readPressure();

    // Get temperature from NTC thermoresistor
    ntc_analog_value = (VCC / analog_steps) * analogRead(A10); // Analog reading from NTC
    voltage_diff = VCC - ntc_analog_value;
    ntc_resistance = ntc_analog_value / (voltage_diff / resistance_def); // Calculates the resistance of the sensor
    ln_value = log(ntc_resistance / resistance_def);                     // Calculates natural log value
    temp_thermo = (1 / ((ln_value / constant_B) + (1 / t_25_kelvin)));   // Temperature from sensor in kelvin
    temp_thermo = temp_thermo - 273.15 + 17.4;                           // Converts to celsius with correction offset

    // Sets SGP30 humidity from SCD30 humidity measurment, to get the most accurate readings
    sgp.setHumidity(getAbsoluteHumidity(temp_thermo, humidity_scd30));
    // Gets readings from SGP30
    sgp.IAQmeasure();
    tvoc = sgp.TVOC;
    eco2 = sgp.eCO2;

    // Gets all readings from PMSA003I
    if (digitalRead(sensor_board_power) == HIGH)
    {
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
    }

    // Get data from MICS2714 sensor
    if (digitalRead(sensor_board_power) == HIGH)
    {
      float mics_voltage;
      int samples = 100;
      for (int i = 0; i < samples; i++)
      {
        mics_voltage += (analogRead(mics_input) / analog_steps) * 3.3;
      }
      mics_voltage = mics_voltage / samples;
      no2_ppm = (5 - mics_voltage) / mics_voltage;
    }

    // Checks if a message has been received, before sending data
    // onReceive(LoRa.parsePacket());

    // Sends data to LoRa radio module
    LoRa.beginPacket();
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

    // Print all data to Serial console
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
  }
}
