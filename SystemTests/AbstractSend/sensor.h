#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <pigpio.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

#define MPU6050_ADDRESS 0x68 // Project one is 0x68, GDP one is 0x69

#define MPU6050_X_ACCEL_OFFS_H 0x06
#define MPU6050_X_ACCEL_OFFS_L 0x07
#define MPU6050_Y_ACCEL_OFFS_H 0x08
#define MPU6050_Y_ACCEL_OFFS_L 0x09
#define MPU6050_Z_ACCEL_OFFS_H 0x0a
#define MPU6050_Z_ACCEL_OFFS_L 0x0b
#define MPU6050_X_GYRO_OFFS_H 0x13
#define MPU6050_X_GYRO_OFFS_L 0x14
#define MPU6050_Y_GYRO_OFFS_H 0x15
#define MPU6050_Y_GYRO_OFFS_L 0x16
#define MPU6050_Z_GYRO_OFFS_H 0x17
#define MPU6050_Z_GYRO_OFFS_L 0x18

#define MPU6050_SELF_TEST_X 0x0d
#define MPU6050_SELF_TEST_Y 0x0e
#define MPU6050_SELF_TEST_Z 0x0f
#define MPU6050_SELF_TEST_A 0x10
#define MPU6050_GYRO_CONFIG 0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_REG_DATA_START 0x3b
#define MPU6050_REG_PWR_MGMT_1 0x6b
#define MPU6050_REG_WHO_AM_I 0x75

#define ACCEL_SCALE 16384.0
#define GYRO_SCALE 131.0

#define LPF_VAL 0.1

#define PI  3.14159265

struct SensorData {
    SensorData() : x(0), y(0), z(0) {}
    float x;
    float y;
    float z;
    void lpf(SensorData current) {
        x = (1.0 - LPF_VAL) * x + LPF_VAL * current.x;
        y = (1.0 - LPF_VAL) * y + LPF_VAL * current.y;
        z = (1.0 - LPF_VAL) * z + LPF_VAL * current.z;
    }

    SensorData normalise() {
        float total = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
        SensorData n;
        n.x = x / total;
        n.y = y / total;
        n.z = z / total;
        return n;
    }
};

class Sensor {
    private:
        uint8_t handle;
        float calculateAccelFT(uint8_t val);
        float calculateGyroFT(uint8_t val);
        void selfTest(float *finalResults);
        bool checkDeviceID();
        void setXAccelOffset(int16_t val);
        void setYAccelOffset(int16_t val);
        void setZAccelOffset(int16_t val);
        void setXGyroOffset(int16_t val);
        void setYGyroOffset(int16_t val);
        void setZGyroOffset(int16_t val);
        SensorData accelTotal;
        SensorData gyroTotal;
        SensorData _getAccelData();
        SensorData _getGyroData();

        SensorData accel;
    public:
        Sensor();
        ~Sensor();
        SensorData getAccelData();
        SensorData getGyroData();
};

#endif // SENSOR_H
