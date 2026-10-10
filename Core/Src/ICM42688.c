#include "ICM42688.h"
#include "i2c.h"
#include "stm32h7xx_hal_i2c.h"
#include <stdint.h>
#include <stdio.h>

static float accSensitivity   = 0.244f;   //加速度的最小分辨率 mg/LSB
static float gyroSensitivity  = 32.8f;    //陀螺仪的最小分辨率
float bsp_Icm42688GetAres(uint8_t Ascale)
{
    switch(Ascale)
    {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (11), 4 Gs (10), 8 Gs (01), and 16 Gs  (00).
    case AFS_2G:
        accSensitivity = 2000 / 32768.0f;
        break;
    case AFS_4G:
        accSensitivity = 4000 / 32768.0f;
        break;
    case AFS_8G:
        accSensitivity = 8000 / 32768.0f;
        break;
    case AFS_16G:
        accSensitivity = 16000 / 32768.0f;
        break;
    }

    return accSensitivity;
}

float bsp_Icm42688GetGres(uint8_t Gscale)
{
    switch(Gscale)
    {
    case GFS_15_125DPS:
        gyroSensitivity = 15.125f / 32768.0f;
        break;
    case GFS_31_25DPS:
        gyroSensitivity = 31.25f / 32768.0f;
        break;
    case GFS_62_5DPS:
        gyroSensitivity = 62.5f / 32768.0f;
        break;
    case GFS_125DPS:
        gyroSensitivity = 125.0f / 32768.0f;
        break;
    case GFS_250DPS:
        gyroSensitivity = 250.0f / 32768.0f;
        break;
    case GFS_500DPS:
        gyroSensitivity = 500.0f / 32768.0f;
        break;
    case GFS_1000DPS:
        gyroSensitivity = 1000.0f / 32768.0f;
        break;
    case GFS_2000DPS:
        gyroSensitivity = 2000.0f / 32768.0f;
        break;
    }
    return gyroSensitivity;
}
int ICM42688_Init(void)
{
    uint8_t data, reg_val, id;
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, ICM42688_ADDRESS, ICM42688_REG_BANK_SEL, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    data = 0x01;
    HAL_I2C_Mem_Write(&hi2c1, ICM42688_ADDRESS, ICM42688_DEVICE_CONFIG, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    HAL_Delay(100);
    HAL_I2C_Mem_Read(&hi2c1, ICM42688_ADDRESS, ICM42688_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &id, 1, 100);
    printf("reg_val:%d\n",id);
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, ICM42688_ADDRESS, ICM42688_REG_BANK_SEL, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    data = 0x01;
    HAL_I2C_Mem_Write(&hi2c1, ICM42688_ADDRESS, ICM42688_DEVICE_CONFIG, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    HAL_Delay(100);


    if(id==ICM42688_ID)
    {
        bsp_Icm42688GetAres(AFS_2G);
        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, ICM42688_ADDRESS, ICM42688_REG_BANK_SEL, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
        reg_val = (AFS_2G << 5);   //量程 ±2g
        reg_val |= (AODR_1000Hz);     //输出速率 1kHz
        HAL_I2C_Mem_Write(&hi2c1, ICM42688_ADDRESS, ICM42688_ACCEL_CONFIG0, I2C_MEMADD_SIZE_8BIT, &reg_val, 1, 100);

        bsp_Icm42688GetGres(GFS_250DPS);
        HAL_I2C_Mem_Write(&hi2c1, ICM42688_ADDRESS, ICM42688_REG_BANK_SEL, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
        reg_val = (GFS_250DPS << 5);   //量程 ±250dps
        reg_val |= (GODR_1000Hz);     //输出速率 1kHz
        HAL_I2C_Mem_Write(&hi2c1, ICM42688_ADDRESS, ICM42688_GYRO_CONFIG0, I2C_MEMADD_SIZE_8BIT, &reg_val, 1, 100);

        HAL_I2C_Mem_Write(&hi2c1, ICM42688_ADDRESS, ICM42688_REG_BANK_SEL, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
         HAL_I2C_Mem_Read(&hi2c1, ICM42688_ADDRESS, ICM42688_PWR_MGMT0, I2C_MEMADD_SIZE_8BIT, &reg_val, 1, 100); //读取PWR—MGMT0当前寄存器的值(page72)
        reg_val &= ~(1 << 5);//使能温度测量
        reg_val |= ((3) << 2);//设置GYRO_MODE  0:关闭 1:待机 2:预留 3:低噪声
        reg_val |= (3);//设置ACCEL_MODE 0:关闭 1:关闭 2:低功耗 3:低噪声
        HAL_I2C_Mem_Write(&hi2c1, ICM42688_ADDRESS, ICM42688_PWR_MGMT0, I2C_MEMADD_SIZE_8BIT, &reg_val, 1, 100);
        HAL_Delay(1); //操作完PWR—MGMT0寄存器后 200us内不能有任何读写寄存器的操作

        return 0;
    }
    return -1;
}

uint8_t ICM42688_ReadAct( ICM42688_Data *rawdata)
{
    uint8_t buffer[15] = {0};
    ICM42688_Rawdata RawData;

    if (HAL_I2C_Mem_Read(&hi2c1, ICM42688_ADDRESS, ICM42688_ACCEL_DATA_X1,
                         I2C_MEMADD_SIZE_8BIT, buffer, 14, 100) != HAL_OK) {
        return 1;
    }

    RawData.ax = ((uint16_t)buffer[0] << 8)  | buffer[1];
    RawData.ay = ((uint16_t)buffer[2] << 8)  | buffer[3];
    RawData.az = ((uint16_t)buffer[4] << 8)  | buffer[5];
    RawData.gx = ((uint16_t)buffer[6] << 8)  | buffer[7];
    RawData.gy = ((uint16_t)buffer[8] << 8)  | buffer[9];
    RawData.gz = ((uint16_t)buffer[10] << 8) | buffer[11];
    RawData.temp = ((uint16_t)buffer[12] << 8) | buffer[13];
    
    if ((RawData.ax == -1 && RawData.ay == -1 && RawData.az == -1) || RawData.ax == -32768)
    {
        return 1;
    }

    rawdata->accel_x = (float)(RawData.ax * accSensitivity);
    rawdata->accel_y = (float)(RawData.ay * accSensitivity);
    rawdata->accel_z = (float)(RawData.az * accSensitivity);

    rawdata->gyro_x = (float)(RawData.gx * gyroSensitivity);
    rawdata->gyro_y = (float)(RawData.gy * gyroSensitivity);
    rawdata->gyro_z = (float)(RawData.gz * gyroSensitivity);

    return 0;
}

