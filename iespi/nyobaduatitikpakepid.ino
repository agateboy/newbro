// // //#include <PID_v1.h>
// // //#include <Arduino.h>
// // //#include "esp_task_wdt.h"
// // //#include <TinyGPS++.h>
// // //#include <SoftwareSerial.h>
// // //#include <Wire.h>
// // //#include <QMC5883LCompass.h>
// // //#include <ESP32Servo.h>
// // //
// // //const int in_ch1 = 36; //switch auto manual
// // //const int in_ch2 = 39; //speed manual
// // //const int in_ch3 = 34; //belok manual
// // //const int in_ch4 = 35; //log gps
// // //const int in_ch5 = 32; // +log
// // //const int in_ch6 = 33;
// // //const int in_ch7 = 26;
// // //
// // //Servo myservo;
// // //const int pinservo = 14;
// // //const int posisi = 90;
// // //
// // //int sw_auto;
// // //
// // //static const uint32_t GPSBaud = 9600;
// // //
// // //TinyGPSPlus gps;
// // //TinyGPSPlus gps2;
// // //
// // //SoftwareSerial g(18, 19);
// // //SoftwareSerial g2(17, 16);
// // //
// // //QMC5883LCompass compass;
// // //
// // //double setpoint, input, output;
// // //double Kp = 1.0, Ki = 0.0, Kd = 0.0;
// // //PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
// // //
// // //double targetLat = -7.774178;
// // //double targetLng = 110.393027;
// // //double target[targetLat][targetLng] = { -7.774178, 110.393027,
// // //                                        -7.776804, 110.385661
// // //                                      }
// // //
// // //void setup() {
// // //  Serial.begin(115200);
// // //  g.begin(GPSBaud);
// // //  g2.begin(GPSBaud);
// // //  myservo.attach(pinservo);
// // //  myservo.write(posisi);
// // //
// // //  // Compass setup
// // //  compass.init();
// // //  compass.setMode(0x01, 0x0C, 0x00, 0x00);
// // //  compass.setSmoothing(5, true);
// // //  compass.setCalibrationOffsets(-28.00, 6.00, 0.00);
// // //  compass.setCalibrationScales(1.00, 1.00, 1.00);
// // //
// // //  // PID setup
// // //  setpoint = 0;
// // //  myPID.SetMode(AUTOMATIC);
// // //  myPID.SetOutputLimits(-90, 90);
// // //
// // //  // Pin setup
// // //  pinMode(in_ch1, INPUT);
// // //  pinMode(in_ch2, INPUT);
// // //  pinMode(in_ch3, INPUT);
// // //  pinMode(in_ch4, INPUT);
// // //  pinMode(in_ch5, INPUT);
// // //  pinMode(in_ch6, INPUT);
// // //  pinMode(in_ch7, INPUT);
// // //
// // //  delay(500);
// // //}
// // //
// // //void loop() {
// // //  sw_auto = pulseIn(in_ch1, HIGH);
// // //
// // //  while (g.available() > 0) {
// // //    gps.encode(g.read());
// // //  }
// // //
// // //  while (g2.available() > 0) {
// // //    gps2.encode(g2.read());
// // //  }
// // //
// // //  if (gps.location.isUpdated()) {
// // //    double currentLat = gps.location.lat();
// // //    double currentLng = gps.location.lng();
// // //    double targetBearing = calculateBearing(currentLat, currentLng, targetLat, targetLng);
// // //
// // //    compass.read();
// // //    double currentHeading = compass.getAzimuth();
// // //
// // //    // Normalize the heading to 0-360 degrees
// // //    if (currentHeading < 0) {
// // //      currentHeading += 360;
// // //    } else if (currentHeading >= 360) {
// // //      currentHeading -= 360;
// // //    }
// // //    currentHeading = map(currentHeading, 0, 360, 360, 0);
// // //    Serial.println(currentHeading);
// // //
// // //    input = currentHeading - targetBearing;
// // //
// // //    if (input < -180) input += 360;
// // //    if (input > 180) input -= 360;
// // //
// // //    myPID.Compute();
// // //
// // //    double servoPosition = posisi + output;
// // //    servoPosition = constrain(servoPosition, 0, 180); // Ensure servo position is within 0-180 degrees
// // //
// // //    if (sw_auto > 1500) { // Assuming a threshold for auto mode
// // //      myservo.write(servoPosition);
// // //    } else {
// // //      // Manual control logic here
// // //      // For example, you can read in_ch2 and in_ch3 for manual control
// // //    }
// // //  }
// // //}
// // //
// // //double calculateBearing(double lat1, double lng1, double lat2, double lng2) {
// // //  double dLon = radians(lng2 - lng1);
// // //  double y = sin(dLon) * cos(radians(lat2));
// // //  double x = cos(radians(lat1)) * sin(radians(lat2)) - sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon);
// // //  double brng = atan2(y, x);
// // //  brng = degrees(brng);
// // //  brng = fmod((brng + 360), 360); // Ensure bearing is within 0-359 degrees
// // //  return brng;
// // //}

// // #include <PID_v1.h>
// // #include <Arduino.h>
// // #include "esp_task_wdt.h"
// // #include <TinyGPS++.h>
// // #include <SoftwareSerial.h>
// // #include <Wire.h>
// // #include <QMC5883LCompass.h>
// // #include <ESP32Servo.h>

// // const int in_ch1 = 36; //switch auto manual
// // const int in_ch2 = 39; //speed manual
// // const int in_ch3 = 34; //belok manual
// // const int in_ch4 = 35; //log gps
// // const int in_ch5 = 32; // +log
// // const int in_ch6 = 33;
// // const int in_ch7 = 26;

// // Servo myservo;
// // const int pinservo = 14;
// // const int posisi = 90;
// // int sw_auto;
// // int set_target;

// // static const uint32_t GPSBaud = 9600;

// // TinyGPSPlus gps;
// // TinyGPSPlus gps2;
// // SoftwareSerial g(18, 19);
// // SoftwareSerial g2(17, 16);

// // QMC5883LCompass compass;

// // double setpoint, input, output;
// // double Kp = 1.0, Ki = 0.0, Kd = 0.0;
// // PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// // double targetLat[] = {-7.774178, -7.776804};
// // double targetLng[] = {110.393027, 110.385661};
// // int currentTargetIndex = 0;
// // const double targetThreshold = 0.0001; // Define a threshold to consider the target reached

// // void setup() {
// //     Serial.begin(115200);
// //     g.begin(GPSBaud);
// //     g2.begin(GPSBaud);
// //     myservo.attach(pinservo);
// //     myservo.write(posisi);

// //     // Compass setup
// //     compass.init();
// //     compass.setMode(0x01, 0x0C, 0x00, 0x00);
// //     compass.setSmoothing(5, true);
// //     compass.setCalibrationOffsets(-28.00, 6.00, 0.00);
// //     compass.setCalibrationScales(1.00, 1.00, 1.00);

// //     // PID setup
// //     setpoint = 0;
// //     myPID.SetMode(AUTOMATIC);
// //     myPID.SetOutputLimits(-90, 90);

// //     // Pin setup
// //     pinMode(in_ch1, INPUT);
// //     pinMode(in_ch2, INPUT);
// //     pinMode(in_ch3, INPUT);
// //     pinMode(in_ch4, INPUT);
// //     pinMode(in_ch5, INPUT);
// //     pinMode(in_ch6, INPUT);
// //     pinMode(in_ch7, INPUT);

// //     delay(500);
// // }

// // void loop() {
// //     sw_auto = pulseIn(in_ch1, HIGH);
// //     set_target = pulseIn(in_ch4, HIGH);
// //     while (g.available() > 0) {
// //         gps.encode(g.read());
// //     }

// //     while (g2.available() > 0) {
// //         gps2.encode(g2.read());
// //     }

// //     if (gps.location.isUpdated()) {
// //         double currentLat = gps.location.lat();
// //         double currentLng = gps.location.lng();

// //         // Check if the current position is close enough to the target
// //         if (abs(currentLat - targetLat[currentTargetIndex]) < targetThreshold &&
// //             abs(currentLng - targetLng[currentTargetIndex]) < targetThreshold) {
// //             // Move to the next target
// //             currentTargetIndex++;
// //             if (currentTargetIndex >= sizeof(targetLat) / sizeof(targetLat[0])) {
// //                 currentTargetIndex = 0; // Reset to the first target if all targets are reached
// //             }
// //         }

// //         double targetBearing = calculateBearing(currentLat, currentLng, targetLat[currentTargetIndex], targetLng[currentTargetIndex]);

// //         compass.read();
// //         double currentHeading = compass.getAzimuth();

// //         // Normalize the heading to 0-360 degrees
// //         if (currentHeading < 0) {
// //             currentHeading += 360;
// //         } else if (currentHeading >= 360) {
// //             currentHeading -= 360;
// //         }

// //         currentHeading = map(currentHeading, 0, 360, 360, 0);
// //         Serial.println(currentHeading);

// //         input = currentHeading - targetBearing;

// //         if (input < -180) input += 360;
// //         if (input > 180) input -= 360;

// //         myPID.Compute();

// //         double servoPosition = posisi + output;
// //         servoPosition = constrain(servoPosition, 0, 180); // Ensure servo position is within 0-180 degrees

// //         if (sw_auto > 1500) { // Assuming a threshold for auto mode
// //             myservo.write(servoPosition);
// //         } else {
// //             // Manual control logic here
// //             // For example, you can read in_ch2 and in_ch3 for manual control
// //         }
// //     }
// // }

// // double calculateBearing(double lat1, double lng1, double lat2, double lng2) {
// //     double dLon = radians(lng2 - lng1);
// //     double y = sin(dLon) * cos(radians(lat2));
// //     double x = cos(radians(lat1)) * sin(radians(lat2)) - sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon);
// //     double brng = atan2(y, x);
// //     brng = degrees(brng);
// //     brng = fmod((brng + 360), 360); // Ensure bearing is within 0-359 degrees
// //     return brng;
// // }

// #include <PID_v1.h>
// #include <Arduino.h>
// #include "esp_task_wdt.h"
// #include <TinyGPS++.h>
// #include <SoftwareSerial.h>
// #include <Wire.h>
// #include <QMC5883LCompass.h>
// #include <ESP32Servo.h>

// const int in_ch1 = 36; // switch auto manual
// const int in_ch2 = 39; // speed manual
// const int in_ch3 = 34; // belok manual
// const int in_ch4 = 35; // log gps
// const int in_ch5 = 32; // +log
// const int in_ch6 = 33;
// const int in_ch7 = 26;

// Servo myservo;
// const int pinservo = 14;
// const int posisi = 90;

// int sw_auto;
// int set_target;

// static const uint32_t GPSBaud = 9600;

// TinyGPSPlus gps;
// TinyGPSPlus gps2;

// SoftwareSerial g(18, 19);
// SoftwareSerial g2(17, 16);

// QMC5883LCompass compass;

// double setpoint, input, output;
// double Kp = 1.0, Ki = 0.0, Kd = 0.0;
// PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// double targetLat[] = {-7.774178, -7.776804};
// double targetLng[] = {110.393027, 110.385661};
// int currentTargetIndex = 0;

// const double targetThreshold = 0.0001; // Define a threshold to consider the target reached

// void setup() {
//     Serial.begin(115200);
//     g.begin(GPSBaud);
//     g2.begin(GPSBaud);
//     myservo.attach(pinservo);
//     myservo.write(posisi);

//     // Compass setup
//     compass.init();
//     compass.setMode(0x01, 0x0C, 0x00, 0x00);
//     compass.setSmoothing(5, true);
//     compass.setCalibrationOffsets(-28.00, 6.00, 0.00);
//     compass.setCalibrationScales(1.00, 1.00, 1.00);

//     // PID setup
//     setpoint = 0;
//     myPID.SetMode(AUTOMATIC);
//     myPID.SetOutputLimits(-90, 90);

//     // Pin setup
//     pinMode(in_ch1, INPUT);
//     pinMode(in_ch2, INPUT);
//     pinMode(in_ch3, INPUT);
//     pinMode(in_ch4, INPUT);
//     pinMode(in_ch5, INPUT);
//     pinMode(in_ch6, INPUT);
//     pinMode(in_ch7, INPUT);

//     delay(500);
// }

// void loop() {
//     sw_auto = pulseIn(in_ch1, HIGH);
//     set_target = pulseIn(in_ch4, HIGH);

//     // Set the target based on the value of set_target
//     if (set_target > 1500) {
//         currentTargetIndex = 0; // First target
//     } else {
//         currentTargetIndex = 1; // Second target
//     }

//     while (g.available() > 0) {
//         gps.encode(g.read());
//     }

//     while (g2.available() > 0) {
//         gps2.encode(g2.read());
//     }

//     if (gps.location.isUpdated()) {
//         double currentLat = gps.location.lat();
//         double currentLng = gps.location.lng();

//         // Check if the current position is close enough to the target
//         if (abs(currentLat - targetLat[currentTargetIndex]) < targetThreshold &&
//             abs(currentLng - targetLng[currentTargetIndex]) < targetThreshold) {
//             // Move to the next target
//             currentTargetIndex++;
//             if (currentTargetIndex >= sizeof(targetLat) / sizeof(targetLat[0])) {
//                 currentTargetIndex = 0; // Reset to the first target if all targets are reached
//             }
//         }

//         double targetBearing = calculateBearing(currentLat, currentLng, targetLat[currentTargetIndex], targetLng[currentTargetIndex]);
//         compass.read();
//         double currentHeading = compass.getAzimuth();

//         // Normalize the heading to 0-360 degrees
//         if (currentHeading < 0) {
//             currentHeading += 360;
//         } else if (currentHeading >= 360) {
//             currentHeading -= 360;
//         }
//         currentHeading = map(currentHeading, 0, 360, 360, 0);

//         Serial.println(currentHeading);

//         input = currentHeading - targetBearing;

//         if (input < -180) input += 360;
//         if (input > 180) input -= 360;

//         myPID.Compute();

//         double servoPosition = posisi + output;
//         servoPosition = constrain(servoPosition, 0, 180); // Ensure servo position is within 0-180 degrees

//         if (sw_auto > 1500) { // Assuming a threshold for auto mode
//             myservo.write(servoPosition);
//         } else {
//             // Manual control logic here
//             // For example, you can read in_ch2 and in_ch3 for manual control
//         }
//     }
// }

// double calculateBearing(double lat1, double lng1, double lat2, double lng2) {
//     double dLon = radians(lng2 - lng1);
//     double y = sin(dLon) * cos(radians(lat2));
//     double x = cos(radians(lat1)) * sin(radians(lat2)) - sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon);
//     double brng = atan2(y, x);
//     brng = degrees(brng);
//     brng = fmod((brng + 360), 360); // Ensure bearing is within 0-359 degrees
//     return brng;
// }

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

Servo myservo;
const int pinservo = 14;
const int posisi = 90;

int sw_auto;
int set_target;

static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;
TinyGPSPlus gps2;

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
    {-7.774178, 110.393027},
    {-7.776804, 110.385661}
};

int currentTargetIndex = 0;
const double targetThreshold = 0.0001; // Define a threshold to consider the target reached

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
}

void loop() {
    sw_auto = pulseIn(in_ch1, HIGH);
    set_target = pulseIn(in_ch4, HIGH);

    // Set the target based on the value of set_target
    if (set_target > 1500) {
        currentTargetIndex = 0; // First target
    } else {
        currentTargetIndex = 1; // Second target
    }

    while (g.available() > 0) {
        gps.encode(g.read());
    }

    while (g2.available() > 0) {
        gps2.encode(g2.read());
    }

    if (gps.location.isUpdated()) {
        double currentLat = gps.location.lat();
        double currentLng = gps.location.lng();

        // Check if the current position is close enough to the target
        if (abs(currentLat - targets[currentTargetIndex].latitude) < targetThreshold &&
            abs(currentLng - targets[currentTargetIndex].longitude) < targetThreshold) {
            // Move to the next target
            currentTargetIndex++;
            if (currentTargetIndex >= sizeof(targets) / sizeof(targets[0])) {
                currentTargetIndex = 0; // Reset to the first target if all targets are reached
            }
        }

        double targetBearing = calculateBearing(currentLat, currentLng, targets[currentTargetIndex].latitude, targets[currentTargetIndex].longitude);
        compass.read();
        double currentHeading = compass.getAzimuth();

        // Normalize the heading to 0-360 degrees
        if (currentHeading < 0) {
            currentHeading += 360;
        } else if (currentHeading >= 360) {
            currentHeading -= 360;
        }

        currentHeading = map(currentHeading, 0, 360, 360, 0);
        Serial.println(currentHeading);

        input = currentHeading - targetBearing;
        if (input < -180) input += 360;
        if (input > 180) input -= 360;

        myPID.Compute();

        double servoPosition = posisi + output;
        servoPosition = constrain(servoPosition, 0, 180); // Ensure servo position is within 0-180 degrees

        if (sw_auto > 1500) { // Assuming a threshold for auto mode
            myservo.write(servoPosition);
        } else {
            // Manual control logic here
            // For example, you can read in_ch2 and in_ch3 for manual control
        }
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
