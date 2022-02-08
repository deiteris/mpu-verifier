#include "Wire.h"
//#include "SPI.h"
#include <i2cscan.h>
#include <I2Cdev.h>

#define SERIAL_BAUDRATE 115200

/*#define PIN_IMU_MISO 12
#define PIN_IMU_MOSI 13
#define PIN_IMU_SCLK 14
#define PIN_IMU_NCS 22
#define SPI_SETUP_FREQ 1000000UL
#define SPI_DATA_FREQ 20000000UL*/

#ifdef ESP8266
#define PIN_IMU_SDA D2
#define PIN_IMU_SCL D1
#else
#define PIN_IMU_SDA 21
#define PIN_IMU_SCL 22
#endif

#define I2C_SPEED   400000

#define MPU_ADDRESS_AD0_LOW     0x68 // address pin low (GND)
#define MPU_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)

#define MPU9255_GENUINE_WHOAMI  115
#define MPU9250_GENUINE_WHOAMI  113
#define MPU6500_GENUINE_WHOAMI  112
#define MPU6050_GENUINE_WHOAMI  104

#define MPU_RA_MAG_ADDRESS      0x0C
#define MPU_RA_MAG_ST1_REG      0x02
#define MPU_RA_MAG_START_REG    0x03

#define MPU9250_RA_ACCEL_XOUT_H 0x3B
#define MPU9250_RA_GYRO_XOUT_H  0x43

#define MPU_RA_INT_PIN_CFG          0x37
#define MPU_RA_WHO_AM_I             0x75
#define MPU_RA_SIGNAL_PATH_RESET    0x68
#define MPU_RA_PWR_MGMT_1           0x6B

uint8_t buffer[14] = {0};

float offset[2][3] = {0};
float scale[2][3] = {1};

bool readMag(int16_t *readings, uint8_t addr) {
    I2Cdev::writeBit(addr, MPU_RA_INT_PIN_CFG, 1, true); // Enable I2C bypass
    if (!I2CSCAN::isI2CExist(MPU_RA_MAG_ADDRESS)) {
        I2Cdev::writeBit(addr, MPU_RA_INT_PIN_CFG, 1, false); // Disable I2C bypass
        return false;
    }
    /*uint8_t ST1;
    do {
        if (!I2Cdev::readBit(MPU_RA_MAG_ADDRESS, MPU_RA_MAG_ST1_REG, 1, &ST1)) return false;
    } while (!(ST1 & 0x01));*/  // Check ST1 for data readiness
    bool status = I2Cdev::readBytes(MPU_RA_MAG_ADDRESS, MPU_RA_MAG_START_REG, 7, buffer); // Read a sequence of 6 Hxx registers + ST2 register
    if (!(buffer[6] & 0x8)) { // Check ST2 for sensor overflow
        readings[0] = (int16_t)(buffer[0] | buffer[1] << 8);
        readings[1] = (int16_t)(buffer[2] | buffer[3] << 8);
        readings[2] = (int16_t)(buffer[4] | buffer[5] << 8);
    }
    I2Cdev::writeBit(addr, MPU_RA_INT_PIN_CFG, 1, false); // Disable I2C bypass
    return status;
}

bool readGyro(int16_t *readings, uint8_t addr) {
    bool status = I2Cdev::readBytes(addr, MPU9250_RA_GYRO_XOUT_H, 6, buffer);
    readings[0] = (((int16_t)buffer[0]) << 8) | buffer[1];
    readings[1] = (((int16_t)buffer[2]) << 8) | buffer[3];
    readings[2] = (((int16_t)buffer[4]) << 8) | buffer[5];
    return status;
}

bool readAccel(int16_t *readings, uint8_t addr) {
    bool status = I2Cdev::readBytes(addr, MPU9250_RA_ACCEL_XOUT_H, 6, buffer);
    readings[0] = (((int16_t)buffer[0]) << 8) | buffer[1];
    readings[1] = (((int16_t)buffer[2]) << 8) | buffer[3];
    readings[2] = (((int16_t)buffer[4]) << 8) | buffer[5];
    return status;
}

void calibrate(uint8_t addr) {
    uint8_t sensor = MPU_ADDRESS_AD0_LOW - addr;
    Serial.println("Calibrating magnetometer...");
    int16_t min1[3] = {0}, max1[3] = {0}, xyz[3] = {0};
    float avg_delta_xyz[3], avg_delta;
    int samples = 500;
    for (int i = 0; i < samples; i++) {
        readMag(xyz, addr);
        Serial.printf("{\"X\":\"%d\",\"Y\":\"%d\",\"Z\":\"%d\"}\n", xyz[0], xyz[1], xyz[2]);
        for (int j = 0; j < 3; j++) {
            min1[j] = min(min1[j], xyz[j]);
            max1[j] = max(max1[j], xyz[j]);
        }
        delay(100);
    }
    Serial.printf("Min: X = %d Y = %d Z = %d\n", min1[0], min1[1], min1[2]);
    Serial.printf("Max: X = %d Y = %d Z = %d\n", max1[0], max1[1], max1[2]);
    for (int i = 0; i < 3; i++) {
        offset[sensor][i] = (max1[i] + min1[i]) / 2.f;
        avg_delta_xyz[i] = (max1[i] - min1[i]) / 2.f;
    }
    avg_delta = (avg_delta_xyz[0] + avg_delta_xyz[1] + avg_delta_xyz[2]) / 3;
    for (int i = 0; i < 3; i++) {
        scale[sensor][i] = avg_delta / avg_delta_xyz[i];
    }
    Serial.printf("Calculated deltas: X = %f Y = %f Z = %f\n", avg_delta_xyz[0], avg_delta_xyz[1], avg_delta_xyz[2]);
    Serial.printf("Calculated offsets: X = %f Y = %f Z = %f\n", offset[sensor][0], offset[sensor][1], offset[sensor][2]);
    Serial.printf("Calculated scales: X = %f Y = %f Z = %f\n", scale[sensor][0], scale[sensor][1], scale[sensor][2]);
}

void checkMag(uint8_t addr) {
    I2Cdev::writeBit(addr, MPU_RA_INT_PIN_CFG, 1, true); // Enable I2C bypass
    if (!I2CSCAN::isI2CExist(MPU_RA_MAG_ADDRESS)) {
        Serial.printf("[ERR] No magnetometer on addr 0x%02x\n", MPU_RA_MAG_ADDRESS);
    } else {
        Serial.printf("[INFO] Magnetometer was found on addr 0x%02x\n", MPU_RA_MAG_ADDRESS);
        I2Cdev::writeByte(MPU_RA_MAG_ADDRESS, 0x0A, 0x16); //enable the magnetometer in continuous mode 16-bit 100hz
        // For QMC5883L
        //I2Cdev::writeByte(MPU_RA_MAG_ADDRESS, 0x0B, 0x01);
        //I2Cdev::writeByte(MPU_RA_MAG_ADDRESS, 0x09, 0x1D);
        delay(10);
        calibrate(addr);
    }
    I2Cdev::writeBit(addr, MPU_RA_INT_PIN_CFG, 1, false); // Disable I2C bypass
}

void checkMPU(uint8_t addr) {
    if (!I2CSCAN::isI2CExist(addr)) {
        Serial.printf("[ERR] Can't find I2C device on addr 0x%02x\n", addr);
        return;
    }
    Serial.printf("[INFO] Found I2C device on addr 0x%02x\n", addr);

    I2Cdev::writeBit(addr, MPU_RA_PWR_MGMT_1, 6, false); // Set sleep disabled

    I2Cdev::readByte(addr, MPU_RA_WHO_AM_I, buffer); // Read WHOAMI register
    switch (buffer[0]) {
        case MPU9255_GENUINE_WHOAMI: {
            Serial.printf("Connected MPU is MPU-9255 with device ID %d\n", MPU9250_GENUINE_WHOAMI);
            break;
        }
        case MPU9250_GENUINE_WHOAMI: {
            Serial.printf("Connected MPU is MPU-9250 with device ID %d\n", MPU9250_GENUINE_WHOAMI);
            break;
        }
        case MPU6500_GENUINE_WHOAMI: {
            Serial.printf("Connected MPU is MPU-6500 with device ID %d\n", MPU6500_GENUINE_WHOAMI);
            break;
        }
        case MPU6050_GENUINE_WHOAMI: {
            Serial.printf("Connected MPU is MPU-6050 with device ID %d\n", MPU6050_GENUINE_WHOAMI);
            break;
        }
        default: {
            Serial.printf("Connected MPU is unknown with device ID %d\n", buffer[0]);
            break;
        }
    }
    checkMag(addr);
}

/*void spiWriteRegister(uint8_t addr, uint8_t val) {
    SPI.beginTransaction(SPISettings(SPI_SETUP_FREQ, MSBFIRST, SPI_MODE3));
    digitalWrite(PIN_IMU_NCS, LOW);
    SPI.transfer16(((0x00 | addr) << 8) | val);
    digitalWrite(PIN_IMU_NCS, HIGH);
    SPI.endTransaction();
}*/

void setup() {
    Serial.begin(SERIAL_BAUDRATE);
    I2CSCAN::clearBus(PIN_IMU_SDA, PIN_IMU_SCL);
    Wire.begin(PIN_IMU_SDA, PIN_IMU_SCL);
    //pinMode(PIN_IMU_NCS, OUTPUT);
    //digitalWrite(PIN_IMU_NCS, HIGH);
    //SPI.begin(PIN_IMU_SCLK, PIN_IMU_MISO, PIN_IMU_MOSI, PIN_IMU_NCS);
#ifdef ESP8266
    Wire.setClockStretchLimit(150000L); // Default stretch limit 150mS
#endif
    Wire.setClock(I2C_SPEED);
    delay(500);

    /*spiWriteRegister(MPU_RA_PWR_MGMT_1, 0x81);
    delay(100);
    spiWriteRegister(MPU_RA_SIGNAL_PATH_RESET, 0x07);
    delay(100);*/

    Serial.println();
    checkMPU(MPU_ADDRESS_AD0_LOW);
    Serial.println("-----------------------");
    checkMPU(MPU_ADDRESS_AD0_HIGH);
    Serial.println("-----------------------");
    Serial.println("Waiting 5 seconds...");
    delay(5000);
    Serial.println();
    Serial.println("-----------------------");
    Serial.println();
}

void loop() {
    /*buffer[0] = 0x80 | MPU9250_RA_GYRO_XOUT_H;
    SPI.beginTransaction(SPISettings(SPI_DATA_FREQ, MSBFIRST, SPI_MODE3));
    digitalWrite(PIN_IMU_NCS, LOW);
    SPI.transfer(buffer, 7);
    digitalWrite(PIN_IMU_NCS, HIGH);
    SPI.endTransaction();

    int16_t readings[3];
    readings[0] = (int16_t)(buffer[2] | buffer[1] << 8);
    readings[1] = (int16_t)(buffer[4] | buffer[3] << 8);
    readings[2] = (int16_t)(buffer[6] | buffer[5] << 8);
    Serial.printf("X=%d Y=%d Z=%d\n", readings[0], readings[1], readings[2]);

    delay(100);*/

    bool res;
    for (int i = 0; i < 2; i++) {
        uint8_t addr = MPU_ADDRESS_AD0_LOW + i;
        int16_t gxyz[3] = {0}, axyz[3] = {0};

        res = readGyro(gxyz, addr);
        if (!res) {
            continue;
        }

        // Show Values
        Serial.printf("Gyro X Value for 0x%02x: ", addr);
        Serial.println(gxyz[0]);
        Serial.printf("Gyro Y Value for 0x%02x: ", addr);
        Serial.println(gxyz[1]);
        Serial.printf("Gyro Z Value for 0x%02x: ", addr);
        Serial.println(gxyz[2]);
        Serial.println();

        res = readAccel(axyz, addr);
        if (!res) {
            continue;
        }

        // Show Values
        Serial.printf("Accel X Value for 0x%02x: ", addr);
        Serial.println(axyz[0]);
        Serial.printf("Accel Y Value for 0x%02x: ", addr);
        Serial.println(axyz[1]);
        Serial.printf("Accel Z Value for 0x%02x: ", addr);
        Serial.println(axyz[2]);
        Serial.println();

        int16_t mxyz[3] = {0};

        bool res = readMag(mxyz, addr);
        if (!res) {
            continue;
        }

        uint8_t sensor = addr - MPU_ADDRESS_AD0_LOW;
        float calibrated_mxyz[3];
        // Apply calibration
        for (int i = 0; i < 3; i++) {
            calibrated_mxyz[i] = (mxyz[i] - offset[sensor][i]) * scale[sensor][i];
        }

        // Show Values
        Serial.printf("Magnetometer X Value for 0x%02x: ", addr);
        Serial.println(calibrated_mxyz[0]);
        Serial.printf("Magnetometer Y Value for 0x%02x: ", addr);
        Serial.println(calibrated_mxyz[1]);
        Serial.printf("Magnetometer Z Value for 0x%02x: ", addr);
        Serial.println(calibrated_mxyz[2]);

        int a = atan2(calibrated_mxyz[1], calibrated_mxyz[0]) * 180.0 / PI;
        Serial.printf("Azimuth: %d\n", a < 0 ? 360 + a : a);
        Serial.println();
    }

    delay(250);
}
