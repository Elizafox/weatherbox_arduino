#include <mbed.h>
#include <rtos.h>
#include <mbed_wait_api.h>
#include <Wire.h>
#include <Adafruit_SI1145.h>
#include <Adafruit_GPS.h>
#include <RTClib.h>
#include <bsec.h>

#include "level_deg_table.h"
#include "weatherbox_common/weather.h"
#include "weatherbox_common/common.h"

// Pin assignments
static const int PIN_WVANE = A0;
static const int PIN_ANEN = 2;
static const int PIN_GAUGE = 3;

// Serial port of the GPS
#define GPSSerial Serial1

Adafruit_SI1145 uv = Adafruit_SI1145();
Adafruit_GPS GPS(&GPSSerial);
Bsec iaq_sensor;
RTC_DS1307 rtc;

Weather w;

// Anenometer counter
volatile unsigned long counter_anen = 0;
float last_anen_measurement = 0;
unsigned long last_anen_time = millis();

// Rain gauge counter
volatile unsigned long gauge_counter = 0;
float last_gauge_measurement = 0;
unsigned long last_gauge_time = millis();

// RTC
unsigned long last_rtc_adjust_time = 0;

// Debounce
volatile unsigned long last_debounce_gauge_time = 0;
const unsigned long debounce_gauge_delay = 50;
volatile unsigned long last_debounce_anen_time = 0;
const unsigned long debounce_anen_delay = 25;

// Interrupt handlers
void int_anen();
void int_gauge();
void request_event();

// Tasks
void task_gps();
void task_readings();
rtos::Thread thread_gps;
rtos::Thread thread_readings;
rtos::Semaphore sem_gps(1);


// Routine to unstick the bus
// Taken from https://www.forward.com.au/pfod/ArduinoProgramming/I2C_clear_bus/
int I2C_clear_bus() {
  Wire.end();
  int PIN_SDA = I2C_SDA;
  int PIN_SCL = I2C_SCL;

#if defined(TWCR) && defined(TWEN)
  TWCR &= ~(_BV(TWEN)); //Disable the Atmel 2-Wire interface so we can control the SDA and SCL pins directly
#endif

  pinMode(PIN_SDA, INPUT_PULLUP); // Make SDA (data) and SCL (clock) pins Inputs with pullup.
  pinMode(PIN_SCL, INPUT_PULLUP);

  delay(2500);  // Wait 2.5 secs. This is strictly only necessary on the first power
  // up of the DS3231 module to allow it to initialize properly,
  // but is also assists in reliable programming of FioV3 boards as it gives the
  // IDE a chance to start uploaded the program
  // before existing sketch confuses the IDE by sending Serial data.

  boolean SCL_LOW = (digitalRead(PIN_SCL) == LOW); // Check is SCL is Low.
  if (SCL_LOW) { //If it is held low Arduno cannot become the I2C master.
    return 1; //I2C bus error. Could not clear SCL clock line held low
  }

  boolean SDA_LOW = (digitalRead(PIN_SDA) == LOW);  // vi. Check SDA input.
  int clockCount = 20; // > 2x9 clock

  while (SDA_LOW && (clockCount > 0)) { //  vii. If SDA is Low,
    clockCount--;
    // Note: I2C bus is open collector so do NOT drive SCL or SDA high.
    pinMode(PIN_SCL, INPUT); // release SCL pullup so that when made output it will be LOW
    pinMode(PIN_SCL, OUTPUT); // then clock SCL Low
    delayMicroseconds(10); //  for >5uS
    pinMode(PIN_SCL, INPUT); // release SCL LOW
    pinMode(PIN_SCL, INPUT_PULLUP); // turn on pullup resistors again
    // do not force high as slave may be holding it low for clock stretching.
    delayMicroseconds(10); //  for >5uS
    // The >5uS is so that even the slowest I2C devices are handled.
    SCL_LOW = (digitalRead(PIN_SCL) == LOW); // Check if SCL is Low.
    int counter = 20;
    while (SCL_LOW && (counter > 0)) {  //  loop waiting for SCL to become High only wait 2sec.
      counter--;
      delay(100);
      SCL_LOW = (digitalRead(PIN_SCL) == LOW);
    }
    if (SCL_LOW) { // still low after 2 sec error
      return 2; // I2C bus error. Could not clear. SCL clock line held low by slave clock stretch for >2sec
    }
    SDA_LOW = (digitalRead(PIN_SDA) == LOW); //   and check SDA input again and loop
  }
  if (SDA_LOW) { // still low
    return 3; // I2C bus error. Could not clear. SDA data line held low
  }

  return 0;
}

inline float get_closest_degree(size_t val1, size_t val2, float level) {
  float v1 = level_deg_table[val1].level;
  float v2 = level_deg_table[val2].level;
  if (level - v1 >= v2 - level)
    return level_deg_table[val2].deg;
  else
    return level_deg_table[val1].deg;
}

float vane_direction(float level) {
  // Corner cases
  if (level <= level_deg_table[0].level)
    return level_deg_table[0].deg;
  if (level >= level_deg_table[level_deg_table_len - 1].level)
    return level_deg_table[level_deg_table_len - 1].deg;

  // Binary search
  size_t i = 0, j = level_deg_table_len, mid = 0;
  while (i < j) {
    mid = (i + j) / 2;

    if (level_deg_table[mid].level == level)
      return level_deg_table[mid].deg;

    // If level is less than array element, search left
    if (level < level_deg_table[mid].level) {
      // If level is greater than previous to mid, return closest of two
      if (mid > 0 && level > level_deg_table[mid - 1].level)
        return get_closest_degree(mid - 1, mid, level);

      // Repeat for left half
      j = mid;
    }
    else {  // If level is greater than mid
      if (mid < level_deg_table_len - 1 && level > level_deg_table[mid + 1].level)
        return get_closest_degree(mid, mid + 1, level);

      i = mid + 1;
    }
  }

  // Only single element left after search
  return level_deg_table[mid].deg;
}

inline float get_vane() {
  int pin_read = analogRead(PIN_WVANE);
  return vane_direction(pin_read);
}

inline float get_wind_speed() {
  const float kph_per_rotation = 2.401;
  unsigned long current_time = millis();

  if (last_anen_time == 0) {
    last_anen_time = current_time;
    return last_anen_measurement;
  }

  float delta = (current_time - last_anen_time) / 1000.0;
  if (delta < 10)
    // No meaningful result can be gathered
    return last_anen_measurement;

  noInterrupts();
  float spd;
  spd = (counter_anen / delta) * kph_per_rotation;
  counter_anen = 0;
  interrupts();

  last_anen_measurement = spd;
  last_anen_time = current_time;
  return spd;
}

inline float get_rainfall() {
  const float mm_per_empty = 2.794;
  unsigned long current_time = millis();

  if (last_gauge_time == 0) {
    last_gauge_time = current_time;
    return last_gauge_measurement;
  }

  // In hours
  float delta = (current_time - last_gauge_time) / 3600000.0;
  if (delta < 1) // Once per hour
    // No meaningful result can be gathered
    return last_gauge_measurement;

  noInterrupts();
  float mm;
  mm = (gauge_counter / delta) * mm_per_empty;
  gauge_counter = 0;
  interrupts();

  last_gauge_measurement = mm;
  last_gauge_time = current_time;
  return mm;
}

// Stolen from the basic example for BSEC
bool check_iaq_sensor_status()
{
  if (iaq_sensor.status != BSEC_OK) {
    if (iaq_sensor.status < BSEC_OK) {
      Serial.print(F("BSEC error code: ")); Serial.println(iaq_sensor.status);
      // Assume this is a transient problem and continue
      if (I2C_clear_bus() == 0) {
        Wire.begin();
      }
      else {
        Serial.println(F("Bus is stuck, restart required!"));
      }
      return false;
    }
    else {
      Serial.print(F("BSEC warning code: ")); Serial.println(iaq_sensor.status);
    }
  }

  if (iaq_sensor.bme680Status != BME680_OK) {
    if (iaq_sensor.bme680Status < BME680_OK) {
      Serial.print(F("BME680 error code: ")); Serial.println(iaq_sensor.bme680Status);
      if (I2C_clear_bus() == 0) {
        Wire.begin();
      }
      else {
        Serial.println(F("Bus is stuck, restart required!"));
      }
      return false;
    }
    else {
      Serial.print(F("BME680 warning code: ")); Serial.println(iaq_sensor.bme680Status);
    }
  }

  return true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  int rtn;
  rtn = I2C_clear_bus(); // clear the I2C bus first before calling Wire.begin()
  if (rtn != 0) {
    Serial.println(F("I2C bus error. Could not clear"));
    if (rtn == 1) {
      Serial.println(F("SCL clock line held low"));
    } else if (rtn == 2) {
      Serial.println(F("SCL clock line held low by slave clock stretch"));
    } else if (rtn == 3) {
      Serial.println(F("SDA data line held low"));
    }
  }
  else { // bus clear
    // re-enable Wire
    // now can start Wire Arduino master
    Wire.begin();
  }

  rtc.begin();
  if (!rtc.isrunning()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  iaq_sensor.begin(BME680_I2C_ADDR_SECONDARY, Wire);
  check_iaq_sensor_status();
  bsec_virtual_sensor_t sensor_list[10] = {
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };
  iaq_sensor.updateSubscription(sensor_list, 10, BSEC_SAMPLE_RATE_LP);
  check_iaq_sensor_status();

  if (!uv.begin()) {
    Serial.println(F("Didn't find Si1145 :("));
    while (true);
  }

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ);  // once every 10 seconds

  // Set up gauge inputs
  pinMode(PIN_WVANE, INPUT);
  pinMode(PIN_ANEN, INPUT);
  pinMode(PIN_GAUGE, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ANEN), int_anen, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_GAUGE), int_gauge, CHANGE);

  thread_gps.start(task_gps);
  thread_readings.start(task_readings);

  Serial.println(F("Weatherbox is starting..."));
}

void task_gps() {
  while (true) {
    sem_gps.acquire();
    GPS.read();
    if (GPS.newNMEAreceived()) {
      GPS.parse(GPS.lastNMEA());
    }
    sem_gps.release();
  }
}

void task_readings() {
  while (true) {
    DateTime now = rtc.now();
    while (now.second() != 0) {
      now = rtc.now();
      delay(200);
    }

    w.set_hour(now.hour());
    w.set_minute(now.minute());
    w.set_second(now.second());
    sem_gps.acquire();
    w.set_msec(GPS.milliseconds);
    sem_gps.release();
    w.set_day(now.day());
    w.set_month(now.month());
    w.set_year(now.year());

    Serial.println(F("Taking readings"));

    w.set_uv(uv.readUV() / 100.0);

    if (!iaq_sensor.run()) {
      check_iaq_sensor_status();
    }
    else {
      w.set_temperature(iaq_sensor.temperature);
      w.set_pressure(iaq_sensor.pressure / 100.0);
      w.set_humidity(iaq_sensor.humidity);
      w.set_aqi(iaq_sensor.staticIaq);
      w.set_co2e(iaq_sensor.co2Equivalent);
      w.set_breath_voce(iaq_sensor.breathVocEquivalent);
    }

    w.set_wind(get_wind_speed());
    w.set_wind_direction(get_vane());

    sem_gps.acquire();
    if (GPS.year > 0 && (last_rtc_adjust_time == 0 ||
                         (millis() - last_rtc_adjust_time) > 3600000)) {
      // Adjust RTC
      last_rtc_adjust_time = millis();
      rtc.adjust(DateTime(GPS.year + 2000, GPS.month, GPS.day,
                          GPS.hour, GPS.minute, GPS.seconds));
    }
    sem_gps.release();

    if (now.minute() == 0) {
      w.set_rainfall_per_hour(get_rainfall());
    }

    Serial.println(F("-----------------------"));
    Serial.println(F("Readings:\n"));
    Serial.print(F("Temperature: "));       Serial.println(w.get_temperature());
    Serial.print(F("Humidity: "));          Serial.println(w.get_humidity());
    Serial.print(F("Pressure: "));          Serial.println(w.get_pressure());
    Serial.print(F("Rainfall: "));          Serial.println(w.get_rainfall_per_hour());
    Serial.print(F("Wind: "));              Serial.println(w.get_wind());
    Serial.print(F("Wind direction: "));    Serial.println(w.get_wind_direction());
    Serial.print(F("UV index: "));          Serial.println(w.get_uv());
    Serial.print(F("AQI: "));               Serial.println(w.get_aqi());
    Serial.print(F("CO2 equiv.: "));        Serial.println(w.get_co2e());
    Serial.print(F("Breath VOC equiv.: ")); Serial.println(w.get_breath_voce());
    Serial.println(F("-----------------------"));

    Serial.println(F("Transferring data..."));
    Wire.beginTransmission(I2C_ADDRESS_ESP32);
    Wire.write(w.to_data(), w.packet_size);
    Wire.endTransmission();
    Serial.println(F("Done~"));

    delay(60000);
  }
}

void loop() {
  // Nothing
  delay(100000);
}

void int_anen() {
  unsigned long current_time = millis();
  if (current_time - last_debounce_anen_time > debounce_anen_delay) {
    counter_anen++;
  }

  last_debounce_anen_time = current_time;
}

void int_gauge() {
  unsigned long current_time = millis();
  if (current_time - last_debounce_gauge_time > debounce_gauge_delay) {
    gauge_counter++;
  }

  last_debounce_gauge_time = current_time;
}
