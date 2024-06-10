#include "mpu6050.h"
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
	//  i2c_init(i2c, I2C_FM);
	
    uint8_t check;
	  check = MPU6050_ReadReg(i2c, WHO_AM_I_REG);
//	if(check == 0x68) {
        MPU6050_WriteReg(i2c, PWR_MGMT_1_REG, 0x01);    
        MPU6050_WriteReg(i2c, SMPLRT_DIV_REG, 0x00);    
        MPU6050_WriteReg(i2c, MPU_CONFIG_REG, 0x00);    
        MPU6050_WriteReg(i2c, GYRO_CONFIG_REG, 0x08);   
        MPU6050_WriteReg(i2c, ACCEL_CONFIG_REG, 0x00); 
		  
//	}

    return check;
}
void MPU6050_WriteReg(uint8_t i2c, uint8_t reg, uint8_t value) {
    i2c_start(i2c);
    i2c_add(i2c, MPU6050_ADDR << 1, 0); // Write mode
    i2c_data(i2c, reg);
    i2c_data(i2c, value);
    i2c_stop(i2c);
}

void MPU6050_Read_Accel(uint8_t i2c, MPU6050 *DataStruct) {
    uint8_t Rec_Data[6];

	  Rec_Data[0] = MPU6050_ReadReg(i2c, ACCEL_XOUT_H_REG);
    Rec_Data[1] = MPU6050_ReadReg(i2c, ACCEL_XOUT_L_REG);
    Rec_Data[2] = MPU6050_ReadReg(i2c, ACCEL_YOUT_H_REG);
    Rec_Data[3] = MPU6050_ReadReg(i2c, ACCEL_YOUT_L_REG);
    Rec_Data[4] = MPU6050_ReadReg(i2c, ACCEL_ZOUT_H_REG);
    Rec_Data[5] = MPU6050_ReadReg(i2c, ACCEL_ZOUT_L_REG);


    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);


    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
}

void MPU6050_Read_Gyro(uint8_t i2c, MPU6050 *DataStruct) {
    uint8_t Rec_Data[6];

	  Rec_Data[0] = MPU6050_ReadReg(i2c, GYRO_XOUT_H_REG);
    Rec_Data[1] = MPU6050_ReadReg(i2c, GYRO_XOUT_L_REG);
    Rec_Data[2] = MPU6050_ReadReg(i2c, GYRO_YOUT_H_REG);
    Rec_Data[3] = MPU6050_ReadReg(i2c, GYRO_YOUT_L_REG);
    Rec_Data[4] = MPU6050_ReadReg(i2c, GYRO_ZOUT_H_REG);
    Rec_Data[5] = MPU6050_ReadReg(i2c, GYRO_ZOUT_L_REG);

    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    DataStruct->Gx = DataStruct->Gyro_X_RAW / 65.5;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 65.5;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 65.5;
}
uint8_t MPU6050_ReadReg(uint8_t i2c, uint8_t reg) {
    uint8_t value;
    
    i2c_start(i2c);
    i2c_add(i2c, MPU6050_ADDR << 1, 0); // Write mode
    i2c_data(i2c, reg);
    i2c_start(i2c);
    i2c_add(i2c, MPU6050_ADDR << 1, 1); // Read mode
    value = i2c_read(i2c, 0);
    i2c_stop(i2c);
    
    return value;
}

void MPU6050_Read_Temp(uint8_t i2c, MPU6050 *DataStruct) {
    uint8_t Rec_Data[2];
    int16_t temp;

  
    Rec_Data[0] = MPU6050_ReadReg(i2c, TEMP_OUT_H_REG);
    Rec_Data[1] = MPU6050_ReadReg(i2c, TEMP_OUT_L_REG);

    temp = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Temperature = (double)(temp / 340.0 + 36.53);
}

