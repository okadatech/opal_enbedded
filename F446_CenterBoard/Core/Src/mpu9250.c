/*
 * mpu9250.c
 *
 *  Created on: Dec 31, 2019
 *      Author: okadatech
 */


#include "mpu9250.h"
#define MPU_InitRegNum 17

int mpu9250_init(SPI_HandleTypeDef *handler){
	mpu9250 = handler;

	 mpu9250_calib_acc();
	 mpu9250_calib_mag();

	 mpu9250_calibrate(g_bias, a_bias);

	   uint8_t i = 0;
	    uint8_t MPU_Init_Data[MPU_InitRegNum][2] = {
	        {BIT_H_RESET, MPUREG_PWR_MGMT_1},        // Reset Device
	        {0x01, MPUREG_PWR_MGMT_1},               // Clock Source
	        {0x00, MPUREG_PWR_MGMT_2},               // Enable Acc & Gyro
	        {my_low_pass_filter, MPUREG_CONFIG},     // Use DLPF set Gyroscope bandwidth 184Hz, temperature bandwidth 188Hz
	        {BITS_FS_2000DPS, MPUREG_GYRO_CONFIG},    // +-250dps
	        {BITS_FS_16G, MPUREG_ACCEL_CONFIG},       // +-2G
	        {my_low_pass_filter_acc, MPUREG_ACCEL_CONFIG_2}, // Set Acc Data Rates, Enable Acc LPF , Bandwidth 184Hz
	        {0x12, MPUREG_INT_PIN_CFG},      //
	        //{0x40, MPUREG_I2C_MST_CTRL},   // I2C Speed 348 kHz
	        //{0x20, MPUREG_USER_CTRL},      // Enable AUX
	        {0x30, MPUREG_USER_CTRL},        // I2C Master mode and set I2C_IF_DIS to disable slave mode I2C bus
	        {0x0D, MPUREG_I2C_MST_CTRL},     // I2C configuration multi-master  IIC 400KHz

	        {AK8963_I2C_ADDR, MPUREG_I2C_SLV0_ADDR},  // Set the I2C slave addres of AK8963 and set for write.
	        //{0x09, MPUREG_I2C_SLV4_CTRL},
	        //{0x81, MPUREG_I2C_MST_DELAY_CTRL}, // Enable I2C delay

	        {AK8963_CNTL2, MPUREG_I2C_SLV0_REG}, // I2C slave 0 register address from where to begin data transfer
	        {0x01, MPUREG_I2C_SLV0_DO},   // Reset AK8963
	        {0x81, MPUREG_I2C_SLV0_CTRL}, // Enable I2C and set 1 byte

	        {AK8963_CNTL1, MPUREG_I2C_SLV0_REG}, // I2C slave 0 register address from where to begin data transfer
	#ifdef AK8963FASTMODE
	        {0x16, MPUREG_I2C_SLV0_DO},   // Register value to 100Hz continuous measurement in 16bit
	#else
	        {0x12, MPUREG_I2C_SLV0_DO},   // Register value to 8Hz continuous measurement in 16bit
	#endif
	        {0x81, MPUREG_I2C_SLV0_CTRL}  //Enable I2C and set 1 byte

	    };

	    for(i = 0; i < MPU_InitRegNum; i++) {
	    	mpu9250_WriteReg(MPU_Init_Data[i][1], MPU_Init_Data[i][0]);
	    	HAL_Delay(1);
	    }

	    mpu9250_set_acc_scale(BITS_FS_16G);
	    mpu9250_set_gyro_scale(BITS_FS_2000DPS);

	    //mpu9250_calib_mag();  // If experiencing problems here, just comment it out. Should still be somewhat functional.
	    return HAL_OK;
}


unsigned int mpu9250_WriteReg( uint8_t WriteAddr, uint8_t WriteData )
{
    unsigned int temp_val;

	 uint8_t TxData[1];
	 uint8_t RxData[1];
	 mpu9250_select();

	 TxData[0]=WriteAddr;
	 HAL_SPI_TransmitReceive(mpu9250, (uint8_t *)TxData, (uint8_t *)RxData,1,0xFF);
	 TxData[0]=WriteData;
	 HAL_SPI_TransmitReceive(mpu9250, (uint8_t *)TxData, (uint8_t *)RxData,1,0xFF);

    temp_val=RxData[0];
    mpu9250_deselect();

    return temp_val;
}

unsigned int  mpu9250_ReadReg( uint8_t WriteAddr, uint8_t WriteData )
{
    return mpu9250_WriteReg(WriteAddr | READ_FLAG,WriteData);
}
void mpu9250_ReadRegs( uint8_t ReadAddr, uint8_t *ReadBuf, unsigned int Bytes )
{
    unsigned int  i = 0;
    uint8_t TxData[1];
    uint8_t RxData[1];
    mpu9250_select();
    TxData[0]=ReadAddr | READ_FLAG;
    HAL_SPI_TransmitReceive(mpu9250, (uint8_t *)TxData, (uint8_t *)RxData,1,0xFF);
    for(i = 0; i < Bytes; i++){
    	 TxData[0]=0x00;
    	 HAL_SPI_TransmitReceive(mpu9250, (uint8_t *)TxData, (uint8_t *)RxData,1,0xFF);
    	 ReadBuf[i] = RxData[0];
    }

    mpu9250_deselect();

    //delayMicroseconds(50);
}


/*                                     INITIALIZATION
 * usage: call this function at startup, giving the sample rate divider (raging from 0 to 255) and
 * low pass filter value; suitable values are:
 * BITS_DLPF_CFG_256HZ_NOLPF2
 * BITS_DLPF_CFG_188HZ
 * BITS_DLPF_CFG_98HZ
 * BITS_DLPF_CFG_42HZ
 * BITS_DLPF_CFG_20HZ
 * BITS_DLPF_CFG_10HZ
 * BITS_DLPF_CFG_5HZ
 * BITS_DLPF_CFG_2100HZ_NOLPF
 * returns 1 if an error occurred
 */

#define MPU_InitRegNum 17


unsigned int mpu9250_set_acc_scale(int scale){
    unsigned int temp_scale;
    mpu9250_WriteReg(MPUREG_ACCEL_CONFIG, scale);

    switch (scale){
        case BITS_FS_2G:
            acc_divider=16384;
        break;
        case BITS_FS_4G:
            acc_divider=8192;
        break;
        case BITS_FS_8G:
            acc_divider=4096;
        break;
        case BITS_FS_16G:
            acc_divider=2048;
        break;
    }
    temp_scale = mpu9250_WriteReg(MPUREG_ACCEL_CONFIG|READ_FLAG, 0x00);

    switch (temp_scale){
        case BITS_FS_2G:
            temp_scale=2;
        break;
        case BITS_FS_4G:
            temp_scale=4;
        break;
        case BITS_FS_8G:
            temp_scale=8;
        break;
        case BITS_FS_16G:
            temp_scale=16;
        break;
    }
    return temp_scale;
}



/*                                 GYROSCOPE SCALE
 * usage: call this function at startup, after initialization, to set the right range for the
 * gyroscopes. Suitable ranges are:
 * BITS_FS_250DPS
 * BITS_FS_500DPS
 * BITS_FS_1000DPS
 * BITS_FS_2000DPS
 * returns the range set (250,500,1000 or 2000)
 */

unsigned int mpu9250_set_gyro_scale(int scale){
    unsigned int temp_scale;
    mpu9250_WriteReg(MPUREG_GYRO_CONFIG, scale);

    switch (scale){
        case BITS_FS_250DPS:   gyro_divider = 131;  break;
        case BITS_FS_500DPS:   gyro_divider = 65.5; break;
        case BITS_FS_1000DPS:  gyro_divider = 32.8; break;
        case BITS_FS_2000DPS:  gyro_divider = 16.4; break;
    }

    temp_scale = mpu9250_WriteReg(MPUREG_GYRO_CONFIG|READ_FLAG, 0x00);

    switch (temp_scale){
        case BITS_FS_250DPS:   temp_scale = 250;    break;
        case BITS_FS_500DPS:   temp_scale = 500;    break;
        case BITS_FS_1000DPS:  temp_scale = 1000;   break;
        case BITS_FS_2000DPS:  temp_scale = 2000;   break;
    }
    return temp_scale;
}



/*                                 WHO AM I?
 * usage: call this function to know if SPI is working correctly. It checks the I2C address of the
 * mpu9250 which should be 0x71
 */

unsigned int mpu9250_whoami(){
    unsigned int response;
    response = mpu9250_WriteReg(0xF5, 0x00);
    return response;
}



/*                                 READ ACCELEROMETER
 * usage: call this function to read accelerometer data. Axis represents selected axis:
 * 0 -> X axis
 * 1 -> Y axis
 * 2 -> Z axis
 */

void mpu9250_read_acc()
{
    uint8_t response_H[1],response_L[1];
    int16_t bit_data;
    float data;
    mpu9250_ReadRegs(MPUREG_ACCEL_XOUT_H,response_H,1);
    mpu9250_ReadRegs(MPUREG_ACCEL_XOUT_L,response_L,1);
    bit_data = ((int16_t)response_H[0]<<8)|response_L[0];
    data = (float)bit_data;
    accel_data[0] = data/acc_divider - a_bias[0];

    mpu9250_ReadRegs(MPUREG_ACCEL_YOUT_H,response_H,1);
    mpu9250_ReadRegs(MPUREG_ACCEL_YOUT_L,response_L,1);
    bit_data = ((int16_t)response_H[0]<<8)|response_L[0];
    data = (float)bit_data;
    accel_data[1] = data/acc_divider - a_bias[1];

    mpu9250_ReadRegs(MPUREG_ACCEL_ZOUT_H,response_H,1);
    mpu9250_ReadRegs(MPUREG_ACCEL_ZOUT_L,response_L,1);
    bit_data = ((int16_t)response_H[0]<<8)|response_L[0];
    data = (float)bit_data;
    accel_data[2] = data/acc_divider - a_bias[2];


}

/*                                 READ GYROSCOPE
 * usage: call this function to read gyroscope data. Axis represents selected axis:
 * 0 -> X axis
 * 1 -> Y axis
 * 2 -> Z axis
 */

void mpu9250_read_gyro()
{
    uint8_t response_H[1],response_L[1];
    int16_t bit_data;
    float data;
    mpu9250_ReadRegs(MPUREG_GYRO_XOUT_H,response_H,1);
    mpu9250_ReadRegs(MPUREG_GYRO_XOUT_L,response_L,1);
    bit_data = ((int16_t)response_H[0]<<8)|response_L[0];
    data = (float)bit_data;
    gyro_data[0] = data/gyro_divider - g_bias[0];


    mpu9250_ReadRegs(MPUREG_GYRO_YOUT_H,response_H,1);
    mpu9250_ReadRegs(MPUREG_GYRO_YOUT_L,response_L,1);
    bit_data = ((int16_t)response_H[0]<<8)|response_L[0];
    data = (float)bit_data;
    gyro_data[1] = data/gyro_divider - g_bias[1];


    mpu9250_ReadRegs(MPUREG_GYRO_ZOUT_H,response_H,1);
    mpu9250_ReadRegs(MPUREG_GYRO_ZOUT_L,response_L,1);
    bit_data = ((int16_t)response_H[0]<<8)|response_L[0];
    data = (float)bit_data;
    gyro_data[2] = data/gyro_divider - g_bias[2];

}


/*                                 READ temperature
 * usage: call this function to read temperature data.
 * returns the value in °C
 */

void mpu9250_read_temp(){
	uint8_t response_H[1],response_L[1];
    int16_t bit_data;
    float data;
    mpu9250_ReadRegs(MPUREG_TEMP_OUT_H,response_H,1);
    mpu9250_ReadRegs(MPUREG_TEMP_OUT_L,response_L,1);

    bit_data = ((int16_t)response_H[0]<<8)|response_L[0];
    data = (float)bit_data;
    temperature = (data/340)+36.53;
    //mpu9250_deselect();
}

/*                                 READ ACCELEROMETER CALIBRATION
 * usage: call this function to read accelerometer data. Axis represents selected axis:
 * 0 -> X axis
 * 1 -> Y axis
 * 2 -> Z axis
 * returns Factory Trim value
 */

void mpu9250_calib_acc()
{
    uint8_t response[4];
    int temp_scale;
    //READ CURRENT ACC SCALE
    temp_scale=mpu9250_WriteReg(MPUREG_ACCEL_CONFIG|READ_FLAG, 0x00);
    mpu9250_set_acc_scale(BITS_FS_8G);
    //ENABLE SELF TEST need modify
    //temp_scale=WriteReg(MPUREG_ACCEL_CONFIG, 0x80>>axis);

    mpu9250_ReadRegs(MPUREG_SELF_TEST_X,response,4);
    calib_data[0] = ((response[0]&11100000)>>3) | ((response[3]&00110000)>>4);
    calib_data[1] = ((response[1]&11100000)>>3) | ((response[3]&00001100)>>2);
    calib_data[2] = ((response[2]&11100000)>>3) | ((response[3]&00000011));

    mpu9250_set_acc_scale(temp_scale);
}

uint8_t mpu9250_AK8963_whoami(){
    uint8_t response;
    mpu9250_WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
    mpu9250_WriteReg(MPUREG_I2C_SLV0_REG, AK8963_WIA); //I2C slave 0 register address from where to begin data transfer
    mpu9250_WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81); //Read 1 byte from the magnetometer

    //WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81);    //Enable I2C and set bytes
    delayUs(100);
    response = mpu9250_WriteReg(MPUREG_EXT_SENS_DATA_00|READ_FLAG, 0x00);    //Read I2C
    //ReadRegs(MPUREG_EXT_SENS_DATA_00,response,1);
    //response=WriteReg(MPUREG_I2C_SLV0_DO, 0x00);    //Read I2C

    return response;
}

void mpu9250_calib_mag(){
    uint8_t response[3];
    float data;
    int i;
    // Choose either 14-bit or 16-bit magnetometer resolution
    //uint8_t MFS_14BITS = 0; // 0.6 mG per LSB
    uint8_t MFS_16BITS =1; // 0.15 mG per LSB
    // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
    uint8_t M_8HZ = 0x02; // 8 Hz update
    //uint8_t M_100HZ = 0x06; // 100 Hz continuous magnetometer

    /* get the magnetometer calibration */

    mpu9250_WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG);   // Set the I2C slave    addres of AK8963 and set for read.
    mpu9250_WriteReg(MPUREG_I2C_SLV0_REG, AK8963_ASAX);                 // I2C slave 0 register address from where to begin data transfer
    mpu9250_WriteReg(MPUREG_I2C_SLV0_CTRL, 0x83);                       // Read 3 bytes from the magnetometer

    //WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81);   // Enable I2C and set bytes
    HAL_Delay(100);
    //response[0]=WriteReg(MPUREG_EXT_SENS_DATA_01|READ_FLAG, 0x00); //Read I2C

    mpu9250_WriteReg(AK8963_CNTL1, 0x00);                               // set AK8963 to Power Down
    delayUs(50000);                                                 // long wait between AK8963 mode changes
    mpu9250_WriteReg(AK8963_CNTL1, 0x0F);                               // set AK8963 to FUSE ROM access
    delayUs(50000);                                                // long wait between AK8963 mode changes

    mpu9250_ReadRegs(MPUREG_EXT_SENS_DATA_00,response,3);
    //response=WriteReg(MPUREG_I2C_SLV0_DO, 0x00);              // Read I2C
    for(i = 0; i < 3; i++) {
        data=response[i];
        Magnetometer_ASA[i] = ((data-128)/256+1)*Magnetometer_Sensitivity_Scale_Factor;
    }
    mpu9250_WriteReg(AK8963_CNTL1, 0x00); // set AK8963 to Power Down
    delayUs(50000);
    // Configure the magnetometer for continuous read and highest resolution.
    // Set bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL
    // register, and enable continuous mode data acquisition (bits [3:0]),
    // 0010 for 8 Hz and 0110 for 100 Hz sample rates.
    mpu9250_WriteReg(AK8963_CNTL1, MFS_16BITS << 4 | M_8HZ);            // Set magnetometer data resolution and sample ODR
    delayUs(50000);
}

void mpu9250_read_mag(){
    uint8_t response[7];
    float data;
    int i;

    mpu9250_WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG);  // Set the I2C slave addres of AK8963 and set for read.
    mpu9250_WriteReg(MPUREG_I2C_SLV0_REG, AK8963_HXL);                 // I2C slave 0 register address from where to begin data transfer
    mpu9250_WriteReg(MPUREG_I2C_SLV0_CTRL, 0x87);                      // Read 6 bytes from the magnetometer

    // delayMicroseconds(1000);
    mpu9250_ReadRegs(MPUREG_EXT_SENS_DATA_00,response,7);
    // must start your read from AK8963A register 0x03 and read seven bytes so that upon read of ST2 register 0x09 the AK8963A will unlatch the data registers for the next measurement.
    for(i = 0; i < 3; i++) {
        mag_data_raw[i] = ((int16_t)response[i*2+1]<<8)|response[i*2];
        data = (float)mag_data_raw[i];
        mag_data[i] = data*Magnetometer_ASA[i];
    }
}

uint8_t mpu9250_get_CNTL1(){
	mpu9250_WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); // Set the I2C slave addres of AK8963 and set for read.
	mpu9250_WriteReg(MPUREG_I2C_SLV0_REG, AK8963_CNTL1);              // I2C slave 0 register address from where to begin data transfer
	mpu9250_WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81); //Read 1 byte from the magnetometer

    // delayMicroseconds(1000);
    return mpu9250_WriteReg(MPUREG_EXT_SENS_DATA_00|READ_FLAG, 0x00);    //Read I2C
}

void mpu9250_read_all(){
    uint8_t response[21];
    int16_t bit_data;
    float data;
    int i;

    // Send I2C command at first
    mpu9250_WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); // Set the I2C slave addres of AK8963 and set for read.
    mpu9250_WriteReg(MPUREG_I2C_SLV0_REG, AK8963_HXL);                // I2C slave 0 register address from where to begin data transfer
    mpu9250_WriteReg(MPUREG_I2C_SLV0_CTRL, 0x87);                     // Read 7 bytes from the magnetometer
    // must start your read from AK8963A register 0x03 and read seven bytes so that upon read of ST2 register 0x09 the AK8963A will unlatch the data registers for the next measurement.

    mpu9250_ReadRegs(MPUREG_ACCEL_XOUT_H,response,21);
    // Get accelerometer value
    for(i = 0; i < 3; i++) {
        bit_data = ((int16_t)response[i*2]<<8) | response[i*2+1];
        data = (float)bit_data;
        accel_data[i] = data/acc_divider - a_bias[i];
    }
    // Get temperature
    bit_data = ((int16_t)response[i*2]<<8) | response[i*2+1];
    data = (float)bit_data;
    temperature = ((data-21)/333.87)+21;
    // Get gyroscope value
    for(i=4; i < 7; i++) {
        bit_data = ((int16_t)response[i*2]<<8) | response[i*2+1];
        data = (float)bit_data;
        gyro_data[i-4] = data/gyro_divider - g_bias[i-4];
    }
    // Get Magnetometer value
    for(i=7; i < 10; i++) {
        mag_data_raw[i-7] = ((int16_t)response[i*2+1]<<8) | response[i*2];
        data = (float)mag_data_raw[i-7];
        mag_data[i-7] = data * Magnetometer_ASA[i-7];
    }
}

void mpu9250_calibrate(float *dest1, float *dest2){
    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

    // reset device
    mpu9250_WriteReg(MPUREG_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
    HAL_Delay(100);

    // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
    // else use the internal oscillator, bits 2:0 = 001
    mpu9250_WriteReg(MPUREG_PWR_MGMT_1, 0x01);
    mpu9250_WriteReg(MPUREG_PWR_MGMT_2, 0x00);
    HAL_Delay(200);

    // Configure device for bias calculation
    mpu9250_WriteReg(MPUREG_INT_ENABLE, 0x00);   // Disable all interrupts
    mpu9250_WriteReg(MPUREG_FIFO_EN, 0x00);      // Disable FIFO
    mpu9250_WriteReg(MPUREG_PWR_MGMT_1, 0x00);   // Turn on internal clock source
    mpu9250_WriteReg(MPUREG_I2C_MST_CTRL, 0x00); // Disable I2C master
    mpu9250_WriteReg(MPUREG_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
    mpu9250_WriteReg(MPUREG_USER_CTRL, 0x0C);    // Reset FIFO and DMP
    HAL_Delay(15);

    // Configure MPU6050 gyro and accelerometer for bias calculation
    mpu9250_WriteReg(MPUREG_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
    mpu9250_WriteReg(MPUREG_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
    mpu9250_WriteReg(MPUREG_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    mpu9250_WriteReg(MPUREG_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

    uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
    uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    mpu9250_WriteReg(MPUREG_USER_CTRL, 0x40);   // Enable FIFO
    mpu9250_WriteReg(MPUREG_FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
    HAL_Delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    mpu9250_WriteReg(MPUREG_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
    mpu9250_ReadRegs(MPUREG_FIFO_COUNTH, data, 2); // read FIFO sample count
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

    for (ii = 0; ii < packet_count; ii++) {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        mpu9250_ReadRegs(MPUREG_FIFO_R_W, data, 12); // read data for averaging
        accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
        accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
        gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
        gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
        gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

        accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[1] += (int32_t) accel_temp[1];
        accel_bias[2] += (int32_t) accel_temp[2];
        gyro_bias[0]  += (int32_t) gyro_temp[0];
        gyro_bias[1]  += (int32_t) gyro_temp[1];
        gyro_bias[2]  += (int32_t) gyro_temp[2];

    }
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;

    if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
    else {accel_bias[2] += (int32_t) accelsensitivity;}

    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
    data[3] = (-gyro_bias[1]/4)       & 0xFF;
    data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
    data[5] = (-gyro_bias[2]/4)       & 0xFF;

    // Push gyro biases to hardware registers
    mpu9250_WriteReg(MPUREG_XG_OFFS_USRH, data[0]);
    mpu9250_WriteReg(MPUREG_XG_OFFS_USRL, data[1]);
    mpu9250_WriteReg(MPUREG_YG_OFFS_USRH, data[2]);
    mpu9250_WriteReg(MPUREG_YG_OFFS_USRL, data[3]);
    mpu9250_WriteReg(MPUREG_ZG_OFFS_USRH, data[4]);
    mpu9250_WriteReg(MPUREG_ZG_OFFS_USRL, data[5]);

    // Output scaled gyro biases for display in the main program
    dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
    dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
    dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

    // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.

    int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
    mpu9250_ReadRegs(MPUREG_XA_OFFSET_H, data, 2); // Read factory accelerometer trim values
    accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
    mpu9250_ReadRegs(MPUREG_YA_OFFSET_H, data, 2);
    accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
    mpu9250_ReadRegs(MPUREG_ZA_OFFSET_H, data, 2);
    accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

    uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
    uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

    for(ii = 0; ii < 3; ii++) {
      if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
    }

    // Construct total accelerometer bias, including calculated average accelerometer bias from above
    accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
    accel_bias_reg[1] -= (accel_bias[1]/8);
    accel_bias_reg[2] -= (accel_bias[2]/8);

    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0])      & 0xFF;
    data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1])      & 0xFF;
    data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2])      & 0xFF;
    data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

// Apparently this is not working for the acceleration biases in the MPU-9250
// Are we handling the temperature correction bit properly?
// Push accelerometer biases to hardware registers
    mpu9250_WriteReg(MPUREG_XA_OFFSET_H, data[0]);
    mpu9250_WriteReg(MPUREG_XA_OFFSET_L, data[1]);
    mpu9250_WriteReg(MPUREG_YA_OFFSET_H, data[2]);
    mpu9250_WriteReg(MPUREG_YA_OFFSET_L, data[3]);
    mpu9250_WriteReg(MPUREG_ZA_OFFSET_H, data[4]);
    mpu9250_WriteReg(MPUREG_ZA_OFFSET_L, data[5]);

// Output scaled accelerometer biases for display in the main program
    dest2[0] = (float)accel_bias[0]/(float)accelsensitivity;
    dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
    dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}

void mpu9250_select() {
    //Set CS low to start transmission (interrupts conversion)
#ifdef CORE_TEENSY
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,0);
#else
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,0);
#endif
}

void mpu9250_deselect() {
    //Set CS high to stop transmission (restarts conversion)
#ifdef CORE_TEENSY
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,1);
#else
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,1);
#endif
}
