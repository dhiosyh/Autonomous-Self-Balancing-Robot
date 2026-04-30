#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
 #include "Wire.h"
#endif

#define MIN_ABS_SPEED 30

MPU6050 mpu;

// MPU control/status vars
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// orientation/motion vars
Quaternion q;
VectorFloat gravity;
float ypr[3];

// PID control configuration
double originalSetpoint = 1.16; // Titik keseimbangan fisik [cite: 110]
double setpoint = originalSetpoint;
double input, output;

// Konstanta PID hasil tuning [cite: 113, 114, 115]
double Kp = 30;
double Ki = 100;
double Kd = 1.5;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Motor control pins [cite: 120 - 125]
double motorSpeedFactorLeft = 0.5;
double motorSpeedFactorRight = 0.5;
int ENA = 11;
int IN1 = 7;
int IN2 = 6;
int IN3 = 5;
int IN4 = 4;
int ENB = 10;

LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

void setup() {
    Serial.begin(115200); // Baud rate tinggi untuk mencegah delay pada kontrol [cite: 206]
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock [cite: 208]
    
    mpu.initialize();
    devStatus = mpu.dmpInitialize();

    // Offset sensor (diatur ke 0 atau disesuaikan hasil kalibrasi) [cite: 139]
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setZAccelOffset(0);

    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        packetSize = mpu.dmpGetFIFOPacketSize();
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10); // Kalkulasi setiap 10ms [cite: 213]
        pid.SetOutputLimits(-255, 255);
    } else {
        Serial.print(F("DMP Initialization failed"));
        while (1); 
    }
}

void loop() {
    fifoCount = mpu.getFIFOCount();
    if (fifoCount >= packetSize) {
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        input = ypr[1] * 180 / M_PI; // Konversi radian ke derajat [cite: 168]
        pid.Compute(); // Hitung output PID [cite: 169]

        // Kirim data ke Serial Plotter untuk monitoring
        Serial.print(input);
        Serial.print(",");
        Serial.println(setpoint);

        // Gerakkan motor berdasarkan output PID
        motorController.move(-output, MIN_ABS_SPEED); [cite: 186, 228]
    }
}
