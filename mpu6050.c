/*#include "mpu6050.h"
const uint16_t i2c_timeout = 100;
const double Accel_Z_corrector = 14418.0;

Kalman_t KalmanX = {
    .Q_angle = (double) 0.001f,
    .Q_bias = (double) 0.003f,
    .R_measure = (double) 0.03f
};

Kalman_t KalmanY = {
    .Q_angle = (double) 0.001f,
    .Q_bias = (double) 0.003f,
    .R_measure = (double) 0.03f
};

uint8_t MPU6050_Init(uint8_t i2c) {
    uint8_t check;
    uint8_t Data;

    // Check device ID WHO_AM_I
    i2c_start(i2c);
    i2c_add(i2c, MPU6050_ADDR, I2C_Direction_Transmitter);
    i2c_data(i2c, WHO_AM_I_REG);
    i2c_start(i2c);
    i2c_add(i2c, MPU6050_ADDR, I2C_Direction_Receiver);
    check = i2c_data(i2c, 0xFF);
    i2c_stop(i2c);

    if (check == 104) { // 0x68 will be returned by the sensor if everything goes well
        // Power management register 0X6B we should write all 0's to wake the sensor up
        Data = 0;
        i2c_start(i2c);
        i2c_add(i2c, MPU6050_ADDR, I2C_Direction_Transmitter);
        i2c_data(i2c, PWR_MGMT_1_REG);
        i2c_data(i2c, Data);
        i2c_stop(i2c);

        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        Data = 0x07;
        i2c_start(i2c);
        i2c_add(i2c, MPU6050_ADDR, I2C_Direction_Transmitter);
        i2c_data(i2c, SMPLRT_DIV_REG);
        i2c_data(i2c, Data);
        i2c_stop(i2c);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ± 2g
        Data = 0x00;
        i2c_start(i2c);
        i2c_add(i2c, MPU6050_ADDR, I2C_Direction_Transmitter);
        i2c_data(i2c, ACCEL_CONFIG_REG);
        i2c_data(i2c, Data);
        i2c_stop(i2c);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ± 250 °/s
        Data = 0x00;
        i2c_start(i2c);
        i2c_add(i2c, MPU6050_ADDR, I2C_Direction_Transmitter);
        i2c_data(i2c, GYRO_CONFIG_REG);
        i2c_data(i2c, Data);
        i2c_stop(i2c);
        
        return 0;
    }
    return 1;
}

void MPU6050_Read_Accel(uint8_t i2c, MPU6050 *DataStruct) {
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from ACCEL_XOUT_H register
    i2c_start(i2c);
    i2c_add(i2c, MPU6050_ADDR, I2C_Direction_Transmitter);
    i2c_data(i2c, ACCEL_XOUT_H_REG);
    i2c_start(i2c);
    i2c_add(i2c, MPU6050_ADDR, I2C_Direction_Receiver);
    
    for (int i = 0; i < 6; i++) {
        Rec_Data[i] = i2c_data(i2c, 0xFF);
    }
    
    i2c_stop(i2c);

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
}

void MPU6050_Read_Gyro(uint8_t i2c, MPU6050 *DataStruct) {
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from GYRO_XOUT_H register
    i2c_start(i2c);
    i2c_add(i2c, MPU6050_ADDR, I2C_Direction_Transmitter);
    i2c_data(i2c, GYRO_XOUT_H_REG);
    i2c_start(i2c);
    i2c_add(i2c, MPU6050_ADDR, I2C_Direction_Receiver);

    for (int i = 0; i < 6; i++) {
        Rec_Data[i] = i2c_data(i2c, 0xFF);
    }
    
    i2c_stop(i2c);

    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;
}

void MPU6050_Read_Temp(uint8_t i2c, MPU6050 *DataStruct) {
    uint8_t Rec_Data[2];
    int16_t temp;

    // Read 2 BYTES of data starting from TEMP_OUT_H_REG register
    i2c_start(i2c);
    i2c_add(i2c, MPU6050_ADDR, I2C_Direction_Transmitter);
    i2c_data(i2c, TEMP_OUT_H_REG);
    i2c_start(i2c);
    i2c_add(i2c, MPU6050_ADDR, I2C_Direction_Receiver);

    for (int i = 0; i < 2; i++) {
        Rec_Data[i] = i2c_data(i2c, 0xFF);
    }
    
    i2c_stop(i2c);

    temp = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Temperature = (double)(temp / 340.0 + 36.53);
}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt) {
    // Predict
    Kalman->angle += dt * (newRate - Kalman->bias);
    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    // Update
    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
}

void MPU6050_Read_All(uint8_t i2c, MPU6050 *DataStruct) {
    double dt = 0.005; // 5 ms

    MPU6050_Read_Accel(i2c, DataStruct);
    MPU6050_Read_Gyro(i2c, DataStruct);
    MPU6050_Read_Temp(i2c, DataStruct);

    double roll = atan(DataStruct->Accel_Y_RAW / sqrt(pow(DataStruct->Accel_X_RAW, 2) + pow(DataStruct->Accel_Z_RAW, 2))) * RAD_TO_DEG;
    double pitch =  atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;

    if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90)) {
        KalmanY.angle = pitch;
        DataStruct->KalmanAngleY = pitch;
    } else {
        DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
    }

    if (fabs(DataStruct->KalmanAngleY) > 90) {
        DataStruct->Gx = -DataStruct->Gx;
    }

    DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gx, dt);
} */
