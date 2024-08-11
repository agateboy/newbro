#include <PID_v1.h>
#include <Arduino.h>
#include "esp_task_wdt.h"
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <QMC5883LCompass.h>
#include <ESP32Servo.h>

const int in_ch1 = 36; // switch auto manual
const int in_ch2 = 39; // speed manual
const int in_ch3 = 34; // belok manual
const int in_ch4 = 35; // log gps
const int in_ch5 = 32; // +log
const int in_ch6 = 33;
const int in_ch7 = 26;

int loggps;

Servo myservo;
const int pinservo = 14;
const int posisi = 90;

Servo ESC_a;
Servo ESC_b;
Servo ESC_c;

int sw_auto;
int set_target;

static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;
TinyGPSPlus gps2;

double currentLat;
double currentLng;
double currentHeading;
double targetBearing;

SoftwareSerial g(18, 19);
SoftwareSerial g2(17, 16);

QMC5883LCompass compass;

double setpoint, input, output;
double Kp = 1.0, Ki = 0.0, Kd = 0.0;
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

struct Target {
  double latitude;
  double longitude;
};

Target targets[] = {
  { -7.774178, 110.393027},
  { -7.776804, 110.385661}
};

int currentTargetIndex = 0;
const double targetRadius = 3.0; // Radius in meters to consider the target reached

bool logPrinted = false; // State variable to track if the log has been printed

void setup() {
  Serial.begin(115200);
  g.begin(GPSBaud);
  g2.begin(GPSBaud);
  myservo.attach(pinservo);
  myservo.write(posisi);

  // Compass setup
  compass.init();
  compass.setMode(0x01, 0x0C, 0x00, 0x00);
  compass.setSmoothing(5, true);
  compass.setCalibrationOffsets(-28.00, 6.00, 0.00);
  compass.setCalibrationScales(1.00, 1.00, 1.00);

  // PID setup
  setpoint = 0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-90, 90);

  // Pin setup
  pinMode(in_ch1, INPUT);
  pinMode(in_ch2, INPUT);
  pinMode(in_ch3, INPUT);
  pinMode(in_ch4, INPUT);
  pinMode(in_ch5, INPUT);
  pinMode(in_ch6, INPUT);
  pinMode(in_ch7, INPUT);

  delay(500);

  ESC_a.attach(27, 988, 2011);
  ESC_b.attach(12, 988, 2011);
  ESC_c.attach(13, 988, 2011);

  xTaskCreatePinnedToCore(
    gpsCompassTask, // Task function
    "GPSCompassTask", // Name of the task
    10000, // Stack size in words
    NULL, // Task input parameter
    1, // Priority of the task
    NULL, // Task handle
    1 // Core where the task should run
  );
}

void loop() {
  sw_auto = pulseIn(in_ch1, HIGH);
  set_target = pulseIn(in_ch4, HIGH);
  loggps = pulseIn(in_ch5, HIGH);
  int inputSpeed = pulseIn(in_ch2, HIGH);
  int baseSpeed = map(inputSpeed, 1000, 2000, 1000, 2000);

  // Set the target based on the value of set_target
  if (set_target > 1800) {
    currentTargetIndex = 0; // First target
    Serial.println(currentHeading);
  } else if (set_target < 1300) {
    if (loggps > 1500 && !logPrinted) {
      Serial.print(currentLat , 45);
      Serial.print(",");
      Serial.print(currentLng, 45);
      Serial.println(",");
      logPrinted = true; // Mark as printed
    } else if (loggps < 1500) {
      logPrinted = false; // Reset the state when loggps drops below 1500
    }
  } else {
    currentTargetIndex = 1; // Second target
    Serial.println(currentHeading);
  }

  if (sw_auto > 1500) { // Assuming a threshold for auto mode
    // Check if the current position is close enough to the target
    double distanceToTarget = calculateDistance(currentLat, currentLng, targets[currentTargetIndex].latitude, targets[currentTargetIndex].longitude);
    if (distanceToTarget < targetRadius) {
      // Move to the next target
      Serial.println("Sampai titik");
      currentTargetIndex++;
      if (currentTargetIndex >= sizeof(targets) / sizeof(targets[0])) {
        Serial.println("Finish");
      }
    }

    // Normalize the heading to 0-360 degrees
    if (currentHeading < 0) {
      currentHeading += 360;
    } else if (currentHeading >= 360) {
      currentHeading -= 360;
    }

    currentHeading = map(currentHeading, 0, 360, 360, 0);
    input = currentHeading - targetBearing;

    if (input < -180) input += 360;
    if (input > 180) input -= 360;

    myPID.Compute();

    // Adjust thruster speeds based on PID output
//    int baseSpeed = 1500; // Base speed for the thrusters
    int thrusterA = baseSpeed + output;
    int thrusterB = baseSpeed - output;
    int thrusterC = baseSpeed; // Adjust as needed

    // Constrain the thruster signals to be within the valid range
    thrusterA = constrain(thrusterA, 1000, 2000);
    thrusterB = constrain(thrusterB, 1000, 2000);
    thrusterC = constrain(thrusterC, 1000, 2000);

    // Write the thruster signals
    ESC_a.writeMicroseconds(thrusterA);
    ESC_b.writeMicroseconds(thrusterB);
    ESC_c.writeMicroseconds(thrusterC);

    myservo.write(posisi + output);
  } else {
    Manual();
  }
}

void Manual() {
  int inputSpeed = pulseIn(in_ch2, HIGH);
  int inputServo = pulseIn(in_ch3, HIGH);

  // Map the input values to thruster control signals
  int speed = map(inputSpeed, 1000, 2000, 1000, 2000); // Adjust the range as needed
  int turn = map(inputServo, 1000, 2000, 1000, 2000); // Adjust the range as needed

  // Calculate the thruster signals
  // int thrusterA = 1500 + speed + turn;
  // int thrusterB = 1500 + speed - turn;
  // int thrusterC = 1500 - speed;
  int thrusterA = 1500;
  int thrusterB = 1500;
  int thrusterC = 1500;

  // Constrain the thruster signals to be within the valid range
  thrusterA = constrain(thrusterA, 1000, 2000);
  thrusterB = constrain(thrusterB, 1000, 2000);
  thrusterC = constrain(thrusterC, 1000, 2000);

  // Write the thruster signals
  ESC_a.writeMicroseconds(thrusterA);
  ESC_b.writeMicroseconds(thrusterB);
  ESC_c.writeMicroseconds(thrusterC);
}

void gpsCompassTask(void *pvParameters) {
  while (true) {
    while (g.available() > 0) {
      gps.encode(g.read());
    }
    while (g2.available() > 0) {
      gps2.encode(g2.read());
    }
    if (gps.location.isUpdated()) {
      currentLat = gps.location.lat();
      currentLng = gps.location.lng();
    }
    targetBearing = calculateBearing(currentLat, currentLng, targets[currentTargetIndex].latitude, targets[currentTargetIndex].longitude);
    compass.read();
    currentHeading = compass.getAzimuth();

    // Add a delay to avoid overwhelming the CPU
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

double calculateBearing(double lat1, double lng1, double lat2, double lng2) {
  double dLon = radians(lng2 - lng1);
  double y = sin(dLon) * cos(radians(lat2));
  double x = cos(radians(lat1)) * sin(radians(lat2)) - sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon);
  double brng = atan2(y, x);
  brng = degrees(brng);
  brng = fmod((brng + 360), 360); // Ensure bearing is within 0-359 degrees
  return brng;
}

double calculateDistance(double lat1, double lng1, double lat2, double lng2) {
  const double R = 6371000; // Radius of the Earth in meters
  double dLat = radians(lat2 - lat1);
  double dLng = radians(lng2 - lng1);
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(radians(lat1)) * cos(radians(lat2)) *
             sin(dLng / 2) * sin(dLng / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  double distance = R * c;
  return distance;
}
