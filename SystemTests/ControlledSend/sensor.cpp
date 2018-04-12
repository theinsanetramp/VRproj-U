#include "sensor.h"

Sensor::Sensor() {
    // Initialises connection
    printf("Initialising MPU6050 connection\n");
    handle = i2cOpen(1, MPU6050_ADDRESS, 0); //TODO: Check i2cBus
    if (handle < 0) {
        printf("I2C Error: %d", handle);
        exit(-1);
    }

    if (!checkDeviceID()) {
        printf("Device ID is not what is expected....\n");
        exit(-1);
    }

    i2cWriteByteData(handle, MPU6050_REG_PWR_MGMT_1, 0x08); // Disables everything and sets internal 8MHz as osc

    // Perform self test
    float selfTestResults[6];
    selfTest(selfTestResults);
    printf("\nSelf Test results within %% of factory value:\n");
    printf("\tAcceleromoter:\n");
    printf("\t\tX:\t%f\tY:\t%f\tZ:\t%f\n", selfTestResults[0], selfTestResults[1], selfTestResults[2]);
    printf("\tGyroscope:\n");
    printf("\t\tX:\t%f\tY:\t%f\tZ:\t%f\n", selfTestResults[3], selfTestResults[4], selfTestResults[5]);

    // Sets range of accel and gyrp
    i2cWriteByteData(handle, MPU6050_ACCEL_CONFIG, 0); // Sets accel to +- 2g
    i2cWriteByteData(handle, MPU6050_GYRO_CONFIG, 0);  // Sets gyro to +- 250 deg/sec

    // Adds offsets
    setXAccelOffset(-2984);
    setYAccelOffset(1010);
    setZAccelOffset(1429);
    setXGyroOffset(-59);
    setYGyroOffset(50);
    setZGyroOffset(-90);
    printf("\n----------------------------------\n\n");
    accelTotal = _getAccelData();
    gyroTotal = _getGyroData();
}

Sensor::~Sensor() {
    i2cClose(handle);
}

bool Sensor::checkDeviceID() {
    uint8_t byte = i2cReadByteData(handle, MPU6050_REG_WHO_AM_I);
    printf("Device ID: 0x%02x\n", byte);
    return (byte == 0x68);
}

SensorData Sensor::_getAccelData() {
    // Reads bytes from sensor
    uint8_t x_msb = i2cReadByteData(handle, MPU6050_REG_DATA_START);
    uint8_t x_lsb = i2cReadByteData(handle, MPU6050_REG_DATA_START + 1);
    uint8_t y_msb = i2cReadByteData(handle, MPU6050_REG_DATA_START + 2);
    uint8_t y_lsb = i2cReadByteData(handle, MPU6050_REG_DATA_START + 3);
    uint8_t z_msb = i2cReadByteData(handle, MPU6050_REG_DATA_START + 4);
    uint8_t z_lsb = i2cReadByteData(handle, MPU6050_REG_DATA_START + 5);
    
    // Puts together bytes
    short x = x_msb << 8 | x_lsb;
    short y = y_msb << 8 | y_lsb;
    short z = z_msb << 8 | z_lsb;

    //printf("\rRaw Accel Data: X: %d\tY: %d\tZ: %d\n", x, y, z);
    //fflush(stdout);

    // Makes data struct and returns it
    SensorData accel;
    accel.x = x / ACCEL_SCALE;
    accel.y = y / ACCEL_SCALE;
    accel.z = z / ACCEL_SCALE;
    return accel;
}

SensorData Sensor::getAccelData() {
    SensorData accel = _getAccelData();
    accelTotal.lpf(accel);
    return accelTotal;
}

SensorData Sensor::_getGyroData() {
    // Reads bytes from sensor
    uint8_t x_msb = i2cReadByteData(handle, MPU6050_REG_DATA_START + 8);
    uint8_t x_lsb = i2cReadByteData(handle, MPU6050_REG_DATA_START + 9);
    uint8_t y_msb = i2cReadByteData(handle, MPU6050_REG_DATA_START + 10);
    uint8_t y_lsb = i2cReadByteData(handle, MPU6050_REG_DATA_START + 11);
    uint8_t z_msb = i2cReadByteData(handle, MPU6050_REG_DATA_START + 12);
    uint8_t z_lsb = i2cReadByteData(handle, MPU6050_REG_DATA_START + 13);

    // Puts together bytes 
    short x = x_msb << 8 | x_lsb;
    short y = y_msb << 8 | y_lsb;
    short z = z_msb << 8 | z_lsb;

    //printf("\rRaw Gyro Data: X: %d     Y: %d     Z: %d              ", x, y, z);
    //fflush(stdout);
    //
    // Makes data struct and returns it
    SensorData gyro;
    gyro.x = x / GYRO_SCALE;
    gyro.y = y / GYRO_SCALE;
    gyro.z = z / GYRO_SCALE;
    return gyro;
}

SensorData Sensor::getGyroData() {
    accel = getAccelData();
    SensorData gyro = _getGyroData();
    gyroTotal.lpf(gyro);
    return gyroTotal;
}

void Sensor::selfTest(float *finalResults) {
    // Start Self Tests
    i2cWriteByteData(handle, MPU6050_ACCEL_CONFIG, 0xf0); // Enables self test on all three accel axes (note changes to +-8g)
    i2cWriteByteData(handle, MPU6050_GYRO_CONFIG, 0xe0); // Enables self test on all three gyro axes
    usleep(1000000); // Sleep to let self test take place
    
    uint8_t rawData[4];
    uint8_t selfTest[6];
    float factoryTrim[6];
    // Gather raw results
    rawData[0] = i2cReadByteData(handle, MPU6050_SELF_TEST_X);
    rawData[1] = i2cReadByteData(handle, MPU6050_SELF_TEST_Y);
    rawData[2] = i2cReadByteData(handle, MPU6050_SELF_TEST_Z);
    rawData[3] = i2cReadByteData(handle, MPU6050_SELF_TEST_A);

    // Extract results
    selfTest[0] = ((rawData[0] >> 3) & 0x1c) | ((rawData[3] & 0x30) >> 4); //XA_TEST
    selfTest[1] = ((rawData[1] >> 3) & 0x1c) | ((rawData[3] & 0x0c) >> 4); //YA_TEST
    selfTest[2] = ((rawData[2] >> 3) & 0x1c) | ((rawData[3] & 0x03) >> 4); //ZA_TEST
    selfTest[3] = rawData[0] & 0x1f; //XG_TEST
    selfTest[4] = rawData[1] & 0x1f; //YG_TEST
    selfTest[5] = rawData[2] & 0x1f; //ZG_TEST

    printf("\nSelf test results:\n");
    printf("\tAcceleromoter:\n");
    printf("\t\tX:\t%d\tY:\t%d\tZ:\t%d\n", selfTest[0], selfTest[1], selfTest[2]);
    printf("\tGyroscope:\n");
    printf("\t\tX:\t%d\tY:\t%d\tZ:\t%d\n", selfTest[3], selfTest[4], selfTest[5]);

    // Calculate Factory Trim
    // Accelerometer
    for (int i = 0; i < 3; i++)
        factoryTrim[i] = calculateAccelFT(selfTest[i]);
    // Gyroscope
    for (int i = 3; i < 6; i++)
        factoryTrim[i] = calculateGyroFT(selfTest[i]);
    factoryTrim[4] = -factoryTrim[4]; // Corrects Gyro Y which is -ve to X and Z

    printf("\nFactory Trims:\n");
    printf("\tAcceleromoter:\n");
    printf("\t\tX:\t%f\tY:\t%f\tZ:\t%f\n", factoryTrim[0], factoryTrim[1], factoryTrim[2]);
    printf("\tGyroscope:\n");
    printf("\t\tX:\t%f\tY:\t%f\tZ:\t%f\n", factoryTrim[3], factoryTrim[4], factoryTrim[5]);

    for(int i = 0; i < 6; i++)
        finalResults[i] = 100.0 + 100.0 * ((float)selfTest[i] - factoryTrim[i]) / factoryTrim[i];
}

float Sensor::calculateAccelFT(uint8_t val) {
    if (val == 0)
        return 0.0;
    else
        return 4096.0 * 0.34 * pow(0.92 / 0.34, (((float)val - 1.0) / 30.0));
}

float Sensor::calculateGyroFT(uint8_t val) {
    if (val == 0)
        return 0.0;
    else
        return 25.0 * 131.0 * pow(1.046, (float)val - 1);
}

void Sensor::setXAccelOffset(int16_t val) {
    i2cWriteByteData(handle, MPU6050_X_ACCEL_OFFS_H, (val >> 8) & 0xff);
    i2cWriteByteData(handle, MPU6050_X_ACCEL_OFFS_L, val & 0xff);
}

void Sensor::setYAccelOffset(int16_t val) {
    i2cWriteByteData(handle, MPU6050_Y_ACCEL_OFFS_H, (val >> 8) & 0xff);
    i2cWriteByteData(handle, MPU6050_Y_ACCEL_OFFS_L, val & 0xff);
}

void Sensor::setZAccelOffset(int16_t val) {
    i2cWriteByteData(handle, MPU6050_Z_ACCEL_OFFS_H, (val >> 8) & 0xff);
    i2cWriteByteData(handle, MPU6050_Z_ACCEL_OFFS_L, val & 0xff);
}

void Sensor::setXGyroOffset(int16_t val) {
    i2cWriteByteData(handle, MPU6050_X_GYRO_OFFS_H, (val >> 8) & 0xff);
    i2cWriteByteData(handle, MPU6050_X_GYRO_OFFS_L, val & 0xff);
}

void Sensor::setYGyroOffset(int16_t val) {
    i2cWriteByteData(handle, MPU6050_Y_GYRO_OFFS_H, (val >> 8) & 0xff);
    i2cWriteByteData(handle, MPU6050_Y_GYRO_OFFS_L, val & 0xff);
}

void Sensor::setZGyroOffset(int16_t val) {
    i2cWriteByteData(handle, MPU6050_Z_GYRO_OFFS_H, (val >> 8) & 0xff);
    i2cWriteByteData(handle, MPU6050_Z_GYRO_OFFS_L, val & 0xff);
}
