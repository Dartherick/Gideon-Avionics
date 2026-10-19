/***************************************************************************
  Avionics data logger - Adafruit BMP581 + BNO08x + SD card

  Minor issue # 16 - License / credits restored (BSD notice from the original
  Adafruit example code this project is derived from).

  Original library header (Adafruit BMP3XX example):
    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing products
    from Adafruit!
    Written by Limor Fried & Kevin Townsend for Adafruit Industries.
    BSD license, all text above must be included in any redistribution

  Based on the Gideon rocket avionics / flight computer logger from
  AsteriaAerospace.com. Adapted to the Adafruit BMP581 breakout
  (http://www.adafruit.com/products/6407, Adafruit_BMP5xx library).

  ISSUE INDEX - search this file for "issue # N" to find each change
  ---------------------------------------------------------------------
  Critical  1  IMU data lagging (drain event queue, drop unused RAW reports)
  Critical  2  Truncated decimals in CSV
  Critical  3  Unique log file names (no more mixed flights)
  Critical  4  SERIAL_DEBUG was always on (#ifdef -> #if)
  Critical  5  Sensor failure no longer freezes the sketch
  Critical  6  SD failure: error blink + retries
  Important 6  Valid CSV (header newline, no trailing comma)
  Important 7  Altitude relative to ground pressure (AGL), single computation
  Important 8  BaroCal is now actually calculated
  Important 9  True 50 Hz row timing + dataReady()
  Important 10 Flush every 1 s instead of 5 s
  Important 11 Row built in RAM, one SD write per row (SdFat not used)
  Minor     12 flush() protected with if (dataLogFile)
  Minor     13 Per-sensor timestamps (BmpTime_s, ImuTime_s)
  Minor     14 I2C clock set to 400 kHz
  Minor     15 Units in CSV header
  Minor     16 License / credits
  Minor     17 Cleanup (dead code, typos, named constants)

  Error blink codes on the RED LED at power-up (repeated twice each):
    2 blinks = BNO08x not found
    3 blinks = BMP581 not found
    4 blinks = SD card / log file problem
  GREEN LED on = log file is open and rows are being written.
 ***************************************************************************/

// Critical issue # 4 - "#define SERIAL_DEBUG 0" combined with "#ifdef" left
// all debug code active (including a 10 s delay at every boot). Every debug
// block below now uses "#if SERIAL_DEBUG", so 0 = off and 1 = on.
#define SERIAL_DEBUG 0

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP5xx.h"   // BMP581 change: Adafruit_BMP5xx library
#include <Adafruit_BNO08x.h>
#include <SPI.h>
#include <SD.h>

// Minor issue # 17 - Magic numbers moved into named constants.
// ---------------------------------------------------------------------
// SD card
const int chipSelect = 4;
#define LOG_FILE_PATTERN      "LOG%04u.CSV"   // 8.3 name, counter 0001-9999
#define LOG_FILE_MAX_INDEX    9999
#define SD_INIT_RETRIES       5
#define SD_RETRY_DELAY_MS     200
#define SD_RETRY_INTERVAL_MS  5000            // later retries from loop()

// Important issue # 10 - flush every second (was 5 s) to lose less data
// if power is cut at landing.
unsigned long lastFlushTimeMilliseconds = 0, flushRateMilliseconds = 1000;

// Barometric pressure/temp sensor
#define SEALEVELPRESSURE_HPA  (1013.25)       // fallback reference only
// BMP581 change: I2C address of the sensor. The official Adafruit example uses
// BMP5XX_ALTERNATIVE_ADDRESS. If an I2C scanner shows a different address,
// change this constant (see the constants defined in Adafruit_BMP5xx.h).
#define BMP_I2C_ADDRESS       BMP5XX_ALTERNATIVE_ADDRESS
#define GROUND_CAL_SAMPLES        50          // ~1 s at 50 Hz
#define GROUND_CAL_MIN_SAMPLES    10
#define GROUND_CAL_TIMEOUT_MS     3000

// BNO08X IMU
#define BNO08X_RESET -1
// Critical issue # 1 - explicit report interval (microseconds). 20 ms = 50 Hz
// per report, matching the 50 Hz CSV row rate.
#define IMU_REPORT_INTERVAL_US    20000
#define IMU_MAX_EVENTS_PER_LOOP   40

// Minor issue # 14 - faster I2C bus to cope with the IMU report traffic.
// If the BNO08x becomes unstable, lower this (e.g. 100000) or remove the
// Wire.setClock() call in setup().
#define I2C_CLOCK_HZ              400000

// Diagnostic LEDs
#define GREEN_LED_PORT 8
#define RED_LED_PORT 13

// Sensor / startup retries
#define INIT_RETRIES              3
#define INIT_RETRY_DELAY_MS       100

// Important issue # 9 - CSV row period (20 ms = 50 Hz)
#define ROW_PERIOD_MS             20

// Critical issue # 2 - decimals printed for each kind of value.
// (Print.print(double) defaults to 2 decimals, which destroyed quaternions,
// gyro, gravity and time resolution.)
#define DIG_TIME   3
#define DIG_BARO   4
#define DIG_TEMP   3
#define DIG_ALT    3
#define DIG_ACCEL  4
#define DIG_GYRO   5
#define DIG_MAG    3
#define DIG_QUAT   6

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// instantiate program object for BMP sensor on I2C bus
Adafruit_BMP5xx bmp;   // BMP581 change

// Critical issue # 5 - sensors are optional at runtime: a failed sensor no
// longer stops the other sensor from being logged.
bool imuOk = false;
bool bmpOk = false;

// These variables accumulate the most current data as it arrives, for later reporting
double Time, BmpTime, ImuTime;   // Minor issue # 13 - per-sensor timestamps (seconds)
double Baro, TempC, TempF, BaroCal, AltiM, AltiF;
double AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ, MagX, MagY, MagZ;
double LAccX, LAccY, LAccZ, GravX, GravY, GravZ;
double RotTheta, RotI, RotJ, RotK;
double GeoRotTheta, GeoRotI, GeoRotJ, GeoRotK;
double GameRotTheta, GameRotI, GameRotJ, GameRotK;

File dataLogFile;
unsigned long nextRowMillis = 0;
unsigned long lastSdRetryMillis = 0;

// Important issue # 11 - each CSV row is assembled in RAM and written to the
// SD card with a single write() call (instead of ~70 small print() calls),
// which shortens the time the SD card blocks the loop.
#define ROW_BUFFER_SIZE 768
class RowBuffer : public Print {
 public:
  using Print::write;
  size_t write(uint8_t c) override {
    if (len < ROW_BUFFER_SIZE) {
      buf[len++] = c;
      return 1;
    }
    return 0; // row too long: extra characters are dropped
  }
  size_t write(const uint8_t *s, size_t n) override {
    size_t written = 0;
    while (written < n && write(s[written])) {
      written++;
    }
    return written;
  }
  void clear() { len = 0; }
  const uint8_t *data() const { return buf; }
  size_t size() const { return len; }
 private:
  uint8_t buf[ROW_BUFFER_SIZE];
  size_t len = 0;
};
RowBuffer rowBuffer;

// Prints ", " followed by the value (so rows never end with a trailing comma)
// Important issue # 6 - helper used to build valid CSV rows.
void field(Print &out, double value, uint8_t digits) {
  out.print(", ");
  out.print(value, digits);
}

// Blink the red LED 'blinks' times, twice, to report an error at power-up.
// Critical issue # 5 / # 6 - visible error codes instead of silent freezes.
void signalError(uint8_t blinks) {
  for (uint8_t repeat = 0; repeat < 2; repeat++) {
    for (uint8_t i = 0; i < blinks; i++) {
      digitalWrite(RED_LED_PORT, HIGH);
      delay(200);
      digitalWrite(RED_LED_PORT, LOW);
      delay(200);
    }
    delay(1000);
  }
}

// Minor issue # 15 - header now carries units. Quaternion components
// (Rot*, GeoRot*, GameRot*) are unitless.
// Important issue # 6 - println() so the first data row starts on a new line.
// Minor issue # 13 - BmpTime_s / ImuTime_s columns added.
// AltiM_m / AltiF_ft are relative to the ground pressure (BaroCal_hPa).
void printCSVHeader(Print &outputFile) {
  outputFile.println("Time_s, BmpTime_s, ImuTime_s, Baro_hPa, TempC_degC, TempF_degF, BaroCal_hPa, AltiM_m, AltiF_ft, AccelX_mps2, AccelY_mps2, AccelZ_mps2, GyroX_rads, GyroY_rads, GyroZ_rads, MagX_uT, MagY_uT, MagZ_uT, LAccX_mps2, LAccY_mps2, LAccZ_mps2, GravX_mps2, GravY_mps2, GravZ_mps2, RotTheta, RotI, RotJ, RotK, GeoRotTheta, GeoRotI, GeoRotJ, GeoRotK, GameRotTheta, GameRotI, GameRotJ, GameRotK");
}

// Critical issue # 3 - unique log file per power-up (LOG0001.CSV, LOG0002.CSV...)
// so different flights are never appended into the same file. Also fixes the
// old "%04d" with an unsigned long.
// Critical issue # 6 - returns false (instead of failing silently) so the
// caller can signal the error and retry.
bool openLogFile(void) {
#if SERIAL_DEBUG
  Serial.print("Initializing SD card...");
#endif

  if (!SD.begin(chipSelect)) {
#if SERIAL_DEBUG
    Serial.println("Card failed, or not present");
#endif
    return false;
  }

  char newfilename[13]; // 8.3 name + terminator
  unsigned int index;
  for (index = 1; index <= LOG_FILE_MAX_INDEX; index++) {
    snprintf(newfilename, sizeof(newfilename), LOG_FILE_PATTERN, index);
    if (!SD.exists(newfilename)) {
      break;
    }
  }
  if (index > LOG_FILE_MAX_INDEX) {
#if SERIAL_DEBUG
    Serial.println("No free log file name");
#endif
    return false;
  }

  dataLogFile = SD.open(newfilename, FILE_WRITE);
  if (!dataLogFile) {
#if SERIAL_DEBUG
    Serial.println("Could not open log file");
#endif
    return false;
  }

  printCSVHeader(dataLogFile);
  digitalWrite(GREEN_LED_PORT, HIGH); // indicate we're writing data
#if SERIAL_DEBUG
  Serial.println("card initialized. Filename:");
  Serial.println(newfilename);
#endif
  return true;
}

// Critical issue # 1 - the three RAW_* reports were enabled but never used;
// they are removed to free up bus bandwidth. Every remaining report uses an
// explicit interval.
// Minor issue # 17 - helper replaces 8 near-identical if/#ifdef blocks.
bool enableImuReport(sh2_SensorId_t id, const char *name) {
  if (!bno08x.enableReport(id, IMU_REPORT_INTERVAL_US)) {
#if SERIAL_DEBUG
    Serial.print("Could not enable ");
    Serial.println(name);
#endif
    return false;
  }
  return true;
}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
#if SERIAL_DEBUG
  Serial.println("Setting desired reports");
#endif
  enableImuReport(SH2_ACCELEROMETER, "accelerometer");
  enableImuReport(SH2_GYROSCOPE_CALIBRATED, "gyroscope");
  enableImuReport(SH2_MAGNETIC_FIELD_CALIBRATED, "magnetic field calibrated");
  enableImuReport(SH2_LINEAR_ACCELERATION, "linear acceleration");
  enableImuReport(SH2_GRAVITY, "gravity vector");
  enableImuReport(SH2_ROTATION_VECTOR, "rotation vector");
  enableImuReport(SH2_GEOMAGNETIC_ROTATION_VECTOR, "geomagnetic rotation vector");
  enableImuReport(SH2_GAME_ROTATION_VECTOR, "game rotation vector");
}

// Critical issue # 5 - returns false instead of spinning forever; retries a
// few times before giving up.
bool initIMU(void) {
  for (uint8_t attempt = 0; attempt < INIT_RETRIES; attempt++) {
    if (bno08x.begin_I2C()) {
#if SERIAL_DEBUG
      Serial.println("BNO08x Found!");
#endif
      setReports(); // Choose what data we want to receive from IMU
      return true;
    }
    delay(INIT_RETRY_DELAY_MS);
  }
#if SERIAL_DEBUG
  Serial.println("Failed to find BNO08x chip");
#endif
  return false;
}

// Critical issue # 5 - same soft-fail approach for the barometer.
bool initBMP(void) {
  bool found = false;
  for (uint8_t attempt = 0; attempt < INIT_RETRIES; attempt++) {
    if (bmp.begin(BMP_I2C_ADDRESS, &Wire)) {   // BMP581 change: hardware I2C, address + bus
      found = true;
      break;
    }
    delay(INIT_RETRY_DELAY_MS);
  }
  if (!found) {
#if SERIAL_DEBUG
    Serial.println("Could not find a valid BMP5 sensor, check wiring!");
#endif
    return false;
  }

  // Set up BMP with oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_8X);   // BMP581 change: BMP5XX_* constants
  bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_4X);      // BMP581 change
  bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);         // BMP581 change
  bmp.setOutputDataRate(BMP5XX_ODR_50_HZ);                  // BMP581 change
  bmp.setPowerMode(BMP5XX_POWERMODE_NORMAL);                // BMP581 change: normal mode, as in the official example
  return true;
}

// Important issue # 7 / # 8 - average the pressure on the ground at power-up
// and use it as the reference ("BaroCal"), so altitude is relative to the
// launch site instead of depending on the weather (1 hPa is about 8 m).
// If not enough samples arrive, falls back to the standard sea level pressure
// (altitude is then relative to sea level, not to the ground).
// NOTE: if the board reboots in flight, this re-zeroes at flight altitude.
// The raw Baro_hPa column is always logged, so data can be reprocessed.
void calibrateGroundPressure(void) {
  double sum = 0;
  uint16_t samples = 0;
  unsigned long start = millis();

  while (samples < GROUND_CAL_SAMPLES && (millis() - start) < GROUND_CAL_TIMEOUT_MS) {
    if (bmp.dataReady() && bmp.performReading()) {
      sum += bmp.pressure;
      samples++;
    }
  }

  if (samples >= GROUND_CAL_MIN_SAMPLES) {
    BaroCal = sum / samples;
  } else {
    BaroCal = SEALEVELPRESSURE_HPA;
  }
}

// set up devices
void setup() {
#if SERIAL_DEBUG
  // Set up serial port for debugging
  Serial.begin(115200); // set up serial port for debugging
  delay(10000); // don't spinlock waiting for serial in case it's not connected
  //while (!Serial);  // wait for serial access object to be ready
  Serial.println("AsteriaAerospace.com 'Discovery' Avionics Data Logger");
#endif

  // Set LED pins to output for debugging/status
  pinMode(RED_LED_PORT, OUTPUT);
  pinMode(GREEN_LED_PORT, OUTPUT);

  // Turn both LEDs off to start
  digitalWrite(GREEN_LED_PORT, LOW);
  digitalWrite(RED_LED_PORT, LOW);

  // Critical issue # 5 - a missing sensor no longer freezes the sketch
  imuOk = initIMU();
  bmpOk = initBMP();

  // Minor issue # 14 - faster I2C bus (set after both sensors have started)
  Wire.setClock(I2C_CLOCK_HZ);

  // Important issue # 7 / # 8 - ground pressure reference
  if (bmpOk) {
    calibrateGroundPressure();
  } else {
    BaroCal = SEALEVELPRESSURE_HPA;
  }

  // Critical issue # 6 - set up data logging to SD card, with retries
  bool logOpen = false;
  for (uint8_t attempt = 0; attempt < SD_INIT_RETRIES && !logOpen; attempt++) {
    logOpen = openLogFile();
    if (!logOpen) {
      delay(SD_RETRY_DELAY_MS);
    }
  }

#if SERIAL_DEBUG
  printCSVHeader(Serial);
#endif

  // Critical issue # 5 / # 6 - report problems with blink codes, then keep going
  if (!imuOk) signalError(2);
  if (!bmpOk) signalError(3);
  if (!logOpen) signalError(4);

  // Important issue # 9 - start the row scheduler
  nextRowMillis = millis();
  lastFlushTimeMilliseconds = millis();
  lastSdRetryMillis = millis();
}

// Critical issue # 1 - drain the IMU event queue every loop (up to a limit)
// instead of reading a single event per row; otherwise 8 reports at 50 Hz
// pile up and the logged data lags further and further behind.
// Minor issue # 13 - ImuTime records when the last IMU event was processed.
void readIMU(void) {

  if (bno08x.wasReset()) {
#if SERIAL_DEBUG
    //Serial.print("sensor was reset ");
#endif
    setReports();
  }

  uint8_t events = 0;
  while (events < IMU_MAX_EVENTS_PER_LOOP && bno08x.getSensorEvent(&sensorValue)) {
    events++;
    ImuTime = millis() / 1000.0;

    switch (sensorValue.sensorId) {

    case SH2_ACCELEROMETER:
      AccelX = sensorValue.un.accelerometer.x;
      AccelY = sensorValue.un.accelerometer.y;
      AccelZ = sensorValue.un.accelerometer.z;
      break;
    case SH2_GYROSCOPE_CALIBRATED:
      GyroX = sensorValue.un.gyroscope.x;
      GyroY = sensorValue.un.gyroscope.y;
      GyroZ = sensorValue.un.gyroscope.z;
      break;
    case SH2_MAGNETIC_FIELD_CALIBRATED:
      MagX = sensorValue.un.magneticField.x;
      MagY = sensorValue.un.magneticField.y;
      MagZ = sensorValue.un.magneticField.z;
      break;
    case SH2_LINEAR_ACCELERATION:
      LAccX = sensorValue.un.linearAcceleration.x;
      LAccY = sensorValue.un.linearAcceleration.y;
      LAccZ = sensorValue.un.linearAcceleration.z;
      break;
    case SH2_GRAVITY:
      GravX = sensorValue.un.gravity.x;
      GravY = sensorValue.un.gravity.y;
      GravZ = sensorValue.un.gravity.z;
      break;
    case SH2_ROTATION_VECTOR:
      RotTheta = sensorValue.un.rotationVector.real;
      RotI = sensorValue.un.rotationVector.i;
      RotJ = sensorValue.un.rotationVector.j;
      RotK = sensorValue.un.rotationVector.k;
      break;
    case SH2_GEOMAGNETIC_ROTATION_VECTOR:
      GeoRotTheta = sensorValue.un.geoMagRotationVector.real;
      GeoRotI = sensorValue.un.geoMagRotationVector.i;
      GeoRotJ = sensorValue.un.geoMagRotationVector.j;
      GeoRotK = sensorValue.un.geoMagRotationVector.k;
      break;
    case SH2_GAME_ROTATION_VECTOR:
      GameRotTheta = sensorValue.un.gameRotationVector.real;
      GameRotI = sensorValue.un.gameRotationVector.i;
      GameRotJ = sensorValue.un.gameRotationVector.j;
      GameRotK = sensorValue.un.gameRotationVector.k;
      break;
    }
  }
}

// Important issue # 9 - only read when the sensor reports a new sample, so
// the same sample is not logged twice.
// Minor issue # 13 - BmpTime records when the last barometer sample was read.
void readBMP(void) {
  if (!bmp.dataReady()) {
    return;
  }

  if (!bmp.performReading()) {
#if SERIAL_DEBUG
    Serial.println("Failed to perform reading.");
#endif
    return;
  }

  BmpTime = millis() / 1000.0;

  // Store data to temp variables

  // Raw Pressure reading
  // BMP581 change: bmp.pressure is already in hPa (the BMP3XX returned Pa), so it is no longer divided by 100
  Baro = bmp.pressure;

  // Celsius
  TempC = bmp.temperature;

  // Farenheit
  TempF = (bmp.temperature * 9 / 5) + 32;

  // Important issue # 7 - altitude relative to the ground pressure measured
  // at power-up (BaroCal), computed once and converted to feet from meters
  // (it used to call readAltitude() twice).
  AltiM = bmp.readAltitude(BaroCal);
  AltiF = AltiM * 3.28084;
}

void reportBMP(Print &outputFile) {
  // Raw Pressure reading
  field(outputFile, Baro, DIG_BARO);

  // Celsius
  field(outputFile, TempC, DIG_TEMP);

  // Farenheit
  field(outputFile, TempF, DIG_TEMP);

  // Important issue # 8 - ground reference pressure (hPa) used for altitude
  field(outputFile, BaroCal, DIG_BARO);

  // Altitude Meters (relative to ground)
  field(outputFile, AltiM, DIG_ALT);

  // Altitude Feet (relative to ground)
  field(outputFile, AltiF, DIG_ALT);
}

// Important issue # 6 - no trailing ", " after the last field, so each CSV
// row has the same number of columns as the header.
void reportIMU(Print &outputFile) {
  field(outputFile, AccelX, DIG_ACCEL);
  field(outputFile, AccelY, DIG_ACCEL);
  field(outputFile, AccelZ, DIG_ACCEL);
  field(outputFile, GyroX, DIG_GYRO);
  field(outputFile, GyroY, DIG_GYRO);
  field(outputFile, GyroZ, DIG_GYRO);
  field(outputFile, MagX, DIG_MAG);
  field(outputFile, MagY, DIG_MAG);
  field(outputFile, MagZ, DIG_MAG);
  field(outputFile, LAccX, DIG_ACCEL);
  field(outputFile, LAccY, DIG_ACCEL);
  field(outputFile, LAccZ, DIG_ACCEL);
  field(outputFile, GravX, DIG_ACCEL);
  field(outputFile, GravY, DIG_ACCEL);
  field(outputFile, GravZ, DIG_ACCEL);
  field(outputFile, RotTheta, DIG_QUAT);
  field(outputFile, RotI, DIG_QUAT);
  field(outputFile, RotJ, DIG_QUAT);
  field(outputFile, RotK, DIG_QUAT);
  field(outputFile, GeoRotTheta, DIG_QUAT);
  field(outputFile, GeoRotI, DIG_QUAT);
  field(outputFile, GeoRotJ, DIG_QUAT);
  field(outputFile, GeoRotK, DIG_QUAT);
  field(outputFile, GameRotTheta, DIG_QUAT);
  field(outputFile, GameRotI, DIG_QUAT);
  field(outputFile, GameRotJ, DIG_QUAT);
  field(outputFile, GameRotK, DIG_QUAT);
}

// Builds one complete CSV row (time columns, BMP columns, IMU columns).
// Critical issue # 2 - time printed with millisecond resolution.
// Minor issue # 13 - BmpTime / ImuTime printed next to Time.
void buildRow(Print &out) {
  out.print(Time, DIG_TIME);
  field(out, BmpTime, DIG_TIME);
  field(out, ImuTime, DIG_TIME);
  reportBMP(out);
  reportIMU(out);
  out.println(); // new line
}

void loop() {
  unsigned long nowMillis = millis();

  // Critical issue # 5 - only talk to sensors that were found at power-up
  if (bmpOk) {
    readBMP();
  }
  if (imuOk) {
    readIMU();
  }

  // Important issue # 9 - rows are written on a fixed 50 Hz schedule using
  // millis() (instead of delay(20), which added the read/write time on top
  // and made the real rate drift below 50 Hz).
  if ((long)(nowMillis - nextRowMillis) >= 0) {
    nextRowMillis += ROW_PERIOD_MS;
    if ((long)(nowMillis - nextRowMillis) >= 0) {
      // fell behind (e.g. a slow SD write): resynchronize instead of bursting
      nextRowMillis = nowMillis + ROW_PERIOD_MS;
    }

    Time = nowMillis / 1000.0;
    rowBuffer.clear();
    buildRow(rowBuffer);

    // print a CSV row to file (single write, see Important issue # 11)
    if (dataLogFile) {
      dataLogFile.write(rowBuffer.data(), rowBuffer.size());
    }

#if SERIAL_DEBUG
    // print a CSV row to Serial
    Serial.write(rowBuffer.data(), rowBuffer.size());
#endif
  }

  // is it time to flush the writes to the SD card yet?
  // Minor issue # 12 - flush() only if the file is actually open.
  // Important issue # 10 - 1 s flush interval (see flushRateMilliseconds).
  if (dataLogFile && (nowMillis - lastFlushTimeMilliseconds >= flushRateMilliseconds)) {
    digitalWrite(RED_LED_PORT, HIGH);
    dataLogFile.flush();
    digitalWrite(RED_LED_PORT, LOW);
    lastFlushTimeMilliseconds = nowMillis;
  }

  // Critical issue # 6 - if the log file is not open (card missing or failed),
  // try again every few seconds instead of giving up for the whole flight.
  if (!dataLogFile && (nowMillis - lastSdRetryMillis >= SD_RETRY_INTERVAL_MS)) {
    lastSdRetryMillis = nowMillis;
    openLogFile();
  }
}