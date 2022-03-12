#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <time.h>
#include <NTPClient.h>
#include <Adafruit_SI1145.h>
#include <ArduinoJson.h>
#include <bsec.h>
#include <MDNS_Generic.h>
#include <RTClib.h>

#include "level_deg_table.h"
#include "favicon.h"
#include "secrets.h"
#include "weather.h"

// Pin assignments
static const int PIN_WVANE = A0;
static const int PIN_ANEN = 2;
static const int PIN_GAUGE = 3;

// Sensors
Adafruit_SI1145 uv = Adafruit_SI1145();
Bsec iaq_sensor;
RTC_Millis rtc;

// Weather structure
static Weather w;

// WiFi
static int status = WL_IDLE_STATUS;
WiFiServer server(80);

// NTP
WiFiUDP ntp_udp;
NTPClient time_client(ntp_udp);
unsigned long ntp_previous_time = 0;

// Web server
static const unsigned long refresh_rate = 60;
const long ws_timeout_time = 1000;

// mDNS
WiFiUDP mdns_udp;
MDNS mdns(mdns_udp);

// Readings timers
unsigned long reading_previous_time = 0;

// Anenometer counter
static volatile unsigned long counter_anen = 0;
static unsigned long last_anen_time = 0;
static float last_anen_measurement = 0;

// Rain gauge counter
static volatile unsigned long counter_gauge = 0;
static unsigned long last_gauge_time = 0;
static float last_gauge_measurement = 0;

// Interrupt handlers
void int_anen();
void int_gauge();
void request_event();


static void do_error() __attribute__((noreturn)) {
  bool state = false;
  while (true) {
    digitalWrite(LED_BUILTIN, state);
    state = !state;
    delay(250);
  }
}

// Routine to unstick the bus
// Taken from https://www.forward.com.au/pfod/ArduinoProgramming/I2C_clear_bus/
int I2C_clear_bus() {
  Wire.end();

#if defined(TWCR) && defined(TWEN)
  TWCR &= ~(_BV(TWEN)); //Disable the Atmel 2-Wire interface so we can control the SDA and SCL pins directly
#endif

  pinMode(SDA, INPUT_PULLUP); // Make SDA (data) and SCL (clock) pins Inputs with pullup.
  pinMode(SCL, INPUT_PULLUP);

  delay(2500);  // Wait 2.5 secs. This is strictly only necessary on the first power
  // up of the DS3231 module to allow it to initialize properly,
  // but is also assists in reliable programming of FioV3 boards as it gives the
  // IDE a chance to start uploaded the program
  // before existing sketch confuses the IDE by sending Serial data.

  boolean SCL_LOW = (digitalRead(SCL) == LOW); // Check is SCL is Low.
  if (SCL_LOW) { //If it is held low Arduno cannot become the I2C master.
    return 1; //I2C bus error. Could not clear SCL clock line held low
  }

  boolean SDA_LOW = (digitalRead(SDA) == LOW);  // vi. Check SDA input.
  int clockCount = 20; // > 2x9 clock

  while (SDA_LOW && (clockCount > 0)) { //  vii. If SDA is Low,
    clockCount--;
    // Note: I2C bus is open collector so do NOT drive SCL or SDA high.
    pinMode(SCL, INPUT); // release SCL pullup so that when made output it will be LOW
    pinMode(SCL, OUTPUT); // then clock SCL Low
    delayMicroseconds(10); //  for >5uS
    pinMode(SCL, INPUT); // release SCL LOW
    pinMode(SCL, INPUT_PULLUP); // turn on pullup resistors again
    // do not force high as slave may be holding it low for clock stretching.
    delayMicroseconds(10); //  for >5uS
    // The >5uS is so that even the slowest I2C devices are handled.
    SCL_LOW = (digitalRead(SCL) == LOW); // Check if SCL is Low.
    int counter = 20;
    while (SCL_LOW && (counter > 0)) {  //  loop waiting for SCL to become High only wait 2sec.
      counter--;
      delay(100);
      SCL_LOW = (digitalRead(SCL) == LOW);
    }
    if (SCL_LOW) { // still low after 2 sec error
      return 2; // I2C bus error. Could not clear. SCL clock line held low by slave clock stretch for >2sec
    }
    SDA_LOW = (digitalRead(SDA) == LOW); //   and check SDA input again and loop
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
  mm = (counter_gauge / delta) * mm_per_empty;
  counter_gauge = 0;
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

int day_of_week(int year, int month, int day) { // 1 <= m <= 12, y > 1752 (in the U.K.)
  static int t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
  if (month < 3) {
    year -= 1;
  }
  return (year + year / 4 - year / 100 + year / 400 + t[month - 1] + day) % 7;
}

void client_send_style(WiFiClient& client) {
  // Feel free to change the background-color and font-size attributes to fit your preferences
  client.println(F("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"));
  client.println(F("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}"));
  client.println(F("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}"));
  client.println(F("</style>"));
}

// Send a 400 response to a client
void client_send_400(WiFiClient& client) {
  client.println(F("HTTP/1.1 400 Bad Request"));
  client.println(F("Content-Type: text/html"));
  client.println(F("Cache-Control: no-cache"));
  client.println(F("Connection: close"));
  client.println();

  // Display the HTML web page
  client.println(F("<!DOCTYPE html><html>"));

  client_send_style(client);

  // Web Page Heading
  client.println(F("<body><h1>400</h1>"));
  client.println(F("<p>Bad request. How naughty~.</p>"));
  client.println(F("</body></html>"));

  // The client response ends with another blank line
  client.println();
}

// Send a 404 response to a client
void client_send_404(WiFiClient& client) {
  client.println(F("HTTP/1.1 404 Not Found"));
  client.println(F("Content-Type: text/html"));
  client.println(F("Cache-Control: no-cache"));
  client.println(F("Connection: close"));
  client.println();

  // Display the HTML web page
  client.println(F("<!DOCTYPE html><html>"));

  client_send_style(client);

  // Web Page Heading
  client.println(F("<body><h1>404</h1>"));
  client.println(F("<p>Not found</p>"));
  client.println(F("</body></html>"));

  // The client response ends with another blank line
  client.println();
}

// Send a webpage with the current weather
void client_send_weather_page(WiFiClient& client) {
  // Send back response headers; tell the client to refresh every so often
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-type: text/html"));
  client.println(F("Cache-Control: no-cache"));
  client.print(F("Refresh: ")); client.println(refresh_rate);
  client.println(F("Connection: close"));
  client.println();

  // Display the HTML web page
  client.println(F("<!DOCTYPE html><html>"));
  client.println(F("<head>"));

  client_send_style(client);

  // Web Page Heading
  client.println(F("<body><h1>Weather data</h1>"));

  client.print(F("<p>Time: "));
  struct tm t = {
    .tm_sec = w.get_second(),
    .tm_min = w.get_minute(),
    .tm_hour = w.get_hour(),
    .tm_mday = w.get_day(),
    .tm_mon = w.get_month() - 1,
    .tm_year = w.get_year() - 1900,
    .tm_wday = day_of_week(w.get_year(), w.get_month(), w.get_day()),
    .tm_yday = -1,
    .tm_isdst = -1
  };
  client.print(asctime(&t)); client.println(F("</p>"));
  client.print(F("<p>Temp: ")); client.print(w.get_temperature()); client.println(F("&deg;C</p>"));
  client.print(F("<p>Humidity: ")); client.print(w.get_humidity()); client.println(F("%</p>"));
  client.print(F("<p>Pressure: ")); client.print(w.get_pressure()); client.println(F("hPa</p>"));
  client.print(F("<p>Wind: ")); client.print(w.get_wind()); client.print(F("KPH, direction "));
  client.print(w.get_wind_direction()); client.println(F("&deg;</p>"));
  client.print(F("<p>UV index: ")); client.print(w.get_uv()); client.println(F("</p>"));
  client.print(F("<p>Visible light: ")); client.print(w.get_visible_light()); client.println(F("</p>"));
  client.print(F("<p>IR light: ")); client.print(w.get_ir()); client.println(F("</p>"));
  client.print(F("<p>AQI: ")); client.print(w.get_aqi()); client.println(F("</p>"));
  client.print(F("<p>CO2 equivalent: ")); client.print(w.get_co2e()); client.println(F("</p>"));
  client.print(F("<p>VOC breath equivalent: ")); client.print(w.get_voce()); client.println(F("</p>"));
  client.print(F("<p>Rainfall per hour: ")); client.print(w.get_rainfall_per_hour()); client.println(F("</p>"));

  client.println(F("</body></html>"));

  // The HTTP response ends with another blank line
  client.println();
}

// Send JSON data
void client_send_api(WiFiClient& client) {
  String data;
  w.to_json(data);

  // Send back response headers; tell the client to refresh every so often
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: application/json"));
  client.println(F("Cache-Control: no-cache"));
  client.print(F("Refresh: ")); client.println(refresh_rate);
  client.println(F("Connection: close"));
  client.println();
  client.print(data);
  client.println();
}

void client_send_favicon(WiFiClient& client) {
  // Send back response headers
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: image/png"));
  client.print(F("Content-Length: ")); client.println(favicon_size);
  client.println(F("Cache-Control: public,max-age=31536000,immutable"));
  client.println(F("Connection: close"));
  client.println();

  // Send the icon
  client.write(favicon, favicon_size);
  client.println();
}

// Route the client's GET request
void route_client(WiFiClient& client, String& path) {
  if (path == F("/api"))
    client_send_api(client);
  else if (path == F("/"))
    client_send_weather_page(client);
  else if (path == F("/favicon.ico"))
    client_send_favicon(client);
  else
    client_send_404(client);
}

void handle_web_client(WiFiClient& client) {
  unsigned long current_time = millis();
  unsigned long previous_time = current_time;
  String header = "";
  String current_line = "";

  Serial.print(F("New Client: "));
  Serial.println(client.remoteIP());

  while (client.connected() && (current_time - previous_time < ws_timeout_time)) {
    current_time = millis();
    if (client.available()) {
      char c = client.read();
      Serial.write(c);
      header += c;
      if (c == '\n') {
        // if the current line is blank, you got two newline characters in a row.
        // that's the end of the client HTTP request, so send a response
        if (current_line.length() == 0) {
          // We only care about GET requests
          int request_type_pos = header.indexOf(F("GET"));
          if (request_type_pos < 0) {
            // Bad request
            client_send_400(client);
          }
          else {
            // We have a valid request, advance to the position of the path
            request_type_pos += 4;
            // Now at the page path.
            // Everything to the next space is the path
            int path_end_pos = header.indexOf(" ", request_type_pos);
            if (path_end_pos < 0) {
              client_send_400(client);
              break;
            }

            // Route to the correct path
            String path = header.substring(request_type_pos, path_end_pos);
            route_client(client, path);
          }
          // Break out of the while loop
          break;
        }
        else {
          current_line = "";
        }
      }
      else if (c != '\r') {  // if you got anything else but a carriage return character,
        current_line += c;   // add it to the end of the current_line
      }
    }
  }

  client.stop();
  Serial.println(F("Client disconnected."));
}

void handle_readings() {
  DateTime now = rtc.now();
  Serial.println(F("Taking readings"));

  w.set_hour(now.hour());
  w.set_minute(now.minute());
  w.set_second(now.second());
  w.set_day(now.day());
  w.set_month(now.month());
  w.set_year(now.year());

  w.set_uv(uv.readUV() / 100.0);
  w.set_visible_light(uv.readVisible());
  w.set_ir(uv.readIR());

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

  if (now.minute() == 0) {
    w.set_rainfall_per_hour(get_rainfall());
  }

  Serial.println(F("Readings finished"));
}

void setup() {
  Serial.begin(115200);

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
    do_error();
  }
  else { // bus clear
    // now can start Wire Arduino master
    Wire.begin();
  }

  rtc.begin(DateTime(F(__DATE__), F(__TIME__)));

  if (!uv.begin()) {
    Serial.println(F("Didn't find Si1145 :("));
    do_error();
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
  iaq_sensor.updateSubscription(sensor_list, 10, BSEC_SAMPLE_RATE_ULP);
  check_iaq_sensor_status();

  // Set up gauge inputs
  pinMode(PIN_WVANE, INPUT);
  pinMode(PIN_ANEN, INPUT);
  pinMode(PIN_GAUGE, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ANEN), int_anen, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_GAUGE), int_gauge, CHANGE);

  while (status != WL_CONNECTED) {
    Serial.print(F("Attempting to connect to Network named: "));
    Serial.println(ssid);

    WiFi.setHostname(hostname);
    status = WiFi.begin(const_cast<char*>(ssid), password);
    // wait 10 seconds for connection:
    delay(10000);
  }

  server.begin();

  time_client.begin();
  time_client.update();

  mdns.begin(WiFi.localIP(), hostname);
  mdns.addServiceRecord("Weatherbox._http", 80, MDNSServiceTCP);

  Serial.print(F("SSID: "));
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print(F("IP Address: "));
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print(F("signal strength (RSSI): "));
  Serial.print(rssi);
  Serial.println(F(" dBm"));

  // Turn off LED
  digitalWrite(LED_BUILTIN, 0);

  Serial.println(F("Weatherbox is starting..."));
}

void loop() {
  unsigned long current_time = millis();

  mdns.run();
  time_client.update();

  if (ntp_previous_time == 0 || current_time - ntp_previous_time >= 60000) {
    ntp_previous_time = current_time;
    rtc.adjust(DateTime(time_client.getEpochTime()));
  }

  DateTime now = rtc.now();
  if (now.second() == 0 && (reading_previous_time == 0 || current_time - reading_previous_time >= 60000)) {
    reading_previous_time = current_time;
    handle_readings();
  }

  WiFiClient client = server.available();  // Listen for incoming clients
  if (client) {
    Serial.println(F("Handling web client!"));
    handle_web_client(client);
    Serial.println(F("Done handling web client!"));
  }
}

void int_anen() {
  counter_anen++;
}

void int_gauge() {
  counter_gauge++;
}
