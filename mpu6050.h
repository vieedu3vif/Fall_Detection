#include "stm32f10x.h"
#include "i2c.h"
#include <math.h>

#define RAD_TO_DEG 57.295779513082320876798154814105

#define MPU6050_ADDR          0x68

#define WHO_AM_I_REG          0x75
#define PWR_MGMT_1_REG        0x6b
#define SMPLRT_DIV_REG        0x19
#define ACCEL_CONFIG_REG      0x1c
#define GYRO_CONFIG_REG       0x1b
#define MPU_CONFIG_REG        0x1a
#define ACCEL_XOUT_H_REG      0x3b
#define ACCEL_XOUT_L_REG      0x3c
#define ACCEL_YOUT_H_REG      0x3d
#define ACCEL_YOUT_L_REG      0x3e
#define ACCEL_ZOUT_H_REG      0x3f
#define ACCEL_ZOUT_L_REG      0x40
#define GYRO_XOUT_H_REG       0x43
#define GYRO_XOUT_L_REG       0x44
#define GYRO_YOUT_H_REG       0x45
#define GYRO_YOUT_L_REG       0x46
#define GYRO_ZOUT_H_REG       0x47
#define GYRO_ZOUT_L_REG       0x48
#define TEMP_OUT_H_REG        0x41
#define TEMP_OUT_L_REG        0x42

extern const uint16_t i2c_timeout;
extern const double Accel_Z_corrector;

extern uint32_t timer;

#define I2C_Direction_Transmitter 0
#define I2C_Direction_Receiver 1

 typedef struct {
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

typedef struct {
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Ax;
    double Ay;
    double Az;
    double Gx;
    double Gy;
    double Gz;
    double Temperature;
    double KalmanAngleX;
    double KalmanAngleY;
} MPU6050;

extern Kalman_t KalmanX ;
extern Kalman_t KalmanY ;

uint8_t MPU6050_Init(uint8_t i2c);
void MPU6050_Read_Accel(uint8_t i2c, MPU6050 *DataStruct);
void MPU6050_Read_Gyro(uint8_t i2c, MPU6050 *DataStruct);
void MPU6050_Read_Temp(uint8_t i2c, MPU6050 *DataStruct);
double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);
void MPU6050_Read_All(uint8_t i2c, MPU6050 *DataStruct);
void MPU6050_WriteReg(uint8_t i2c, uint8_t reg, uint8_t value);
uint8_t MPU6050_ReadReg(uint8_t i2c, uint8_t reg);
