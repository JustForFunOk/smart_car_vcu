#include "mpu9250.h"
#include <stdio.h>  // printf

const uint8_t READWRITE_CMD = 0x80;
const uint8_t MULTIPLEBYTE_CMD = 0x40;
const uint8_t DUMMY_BYTE = 0x00;

// 400 kHz
const uint32_t _i2cRate = 400000;

// MPU9250 registers address
// Power Management 1
const uint8_t PWR_MGMNT_1 = 0x6B;
const uint8_t CLOCK_SEL_PLL = 0x01;  // Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
const uint8_t PWR_RESET = 0x80;  // 1 – Reset the internal registers and restores the default settings. Write a 1 to set the reset, the bit will auto clear.
// Power Management 2
const uint8_t PWR_MGMNT_2 = 0x6C;
const uint8_t SEN_ENABLE = 0x00;  // open XYZ axis of gyro and accelerometer
// User Control
const uint8_t USER_CTRL = 0x6A;
// I2C Master Control
const uint8_t I2C_MST_CTRL = 0x24;
const uint8_t I2C_MST_EN = 0x20;  // Enable the I2C Master I/F module; pins ES_DA and ES_SCL are isolated from pins SDA/SDI and SCL/ SCLK.
const uint8_t I2C_MST_CLK = 0x0D;  // I2C clock speed 400kHz
// Accelerometer Configuration
const uint8_t ACCEL_CONFIG = 0x1C;
const uint8_t ACCEL_FS_SEL_2G = 0x00;  // Accel Full Scale Select: +-2g
const uint8_t ACCEL_FS_SEL_4G = 0x08;  // Accel Full Scale Select: +-4g
const uint8_t ACCEL_FS_SEL_8G = 0x10;  // Accel Full Scale Select: +-8g
const uint8_t ACCEL_FS_SEL_16G = 0x18; // Accel Full Scale Select: +-16g
// Accelerometer Configuration 2
const uint8_t ACCEL_CONFIG2 = 0x1D;
const uint8_t ACCEL_DLPF_460 = 0x00;  // Data Low Pass Filter Bandwidth: 460Hz [default]
const uint8_t ACCEL_DLPF_184 = 0x01;  // Data Low Pass Filter Bandwidth: 184Hz
const uint8_t ACCEL_DLPF_92 = 0x02;   // Data Low Pass Filter Bandwidth: 92Hz
const uint8_t ACCEL_DLPF_41 = 0x03;   // Data Low Pass Filter Bandwidth: 41Hz
const uint8_t ACCEL_DLPF_20 = 0x04;   // Data Low Pass Filter Bandwidth: 20Hz
const uint8_t ACCEL_DLPF_10 = 0x05;   // Data Low Pass Filter Bandwidth: 10Hz
const uint8_t ACCEL_DLPF_5 = 0x06;    // Data Low Pass Filter Bandwidth: 5Hz
// Gyroscope Configuration
const uint8_t GYRO_CONFIG = 0x1B;
const uint8_t GYRO_FS_SEL_250DPS = 0x00;  // Gyro Full Scale Select: +-250dps(degrees per second) and FCHOICE[1:0]=11
const uint8_t GYRO_FS_SEL_500DPS = 0x08;  // Gyro Full Scale Select: +-500dps
const uint8_t GYRO_FS_SEL_1000DPS = 0x10; // Gyro Full Scale Select: +-1000dps
const uint8_t GYRO_FS_SEL_2000DPS = 0x18; // Gyro Full Scale Select: +-2000dps
// Configuration
const uint8_t CONFIG = 0x1A;
const uint8_t GYRO_DLPF_250 = 0x00;  // Data Low Pass Filter Bandwidth: 250Hz
const uint8_t GYRO_DLPF_184 = 0x01;  // Data Low Pass Filter Bandwidth: 184Hz
const uint8_t GYRO_DLPF_92 = 0x02;   // Data Low Pass Filter Bandwidth: 92Hz
const uint8_t GYRO_DLPF_41 = 0x03;   // Data Low Pass Filter Bandwidth: 41Hz
const uint8_t GYRO_DLPF_20 = 0x04;   // Data Low Pass Filter Bandwidth: 20Hz
const uint8_t GYRO_DLPF_10 = 0x05;   // Data Low Pass Filter Bandwidth: 10Hz
const uint8_t GYRO_DLPF_5 = 0x06;    // Data Low Pass Filter Bandwidth: 5Hz
// Sample Rate Divider
const uint8_t SMPDIV = 0x19;
// INT Pin / Bypass Enable Configuration
const uint8_t INT_PIN_CFG = 0x37;
const uint8_t BYPASS_EN = 0x02;


const uint8_t ACCEL_OUT = 0x3B;
const uint8_t GYRO_OUT = 0x43;
const uint8_t TEMP_OUT = 0x41;
const uint8_t EXT_SENS_DATA_00 = 0x49;


const uint8_t INT_ENABLE = 0x38;
const uint8_t INT_DISABLE = 0x00;
const uint8_t INT_PULSE_50US = 0x00;
const uint8_t INT_WOM_EN = 0x40;
const uint8_t INT_RAW_RDY_EN = 0x01;
const uint8_t PWR_CYCLE = 0x20;



const uint8_t DIS_GYRO = 0x07;
const uint8_t I2C_SLV0_ADDR = 0x25;
const uint8_t I2C_SLV0_REG = 0x26;
const uint8_t I2C_SLV0_DO = 0x63;
const uint8_t I2C_SLV0_CTRL = 0x27;
const uint8_t I2C_SLV0_EN = 0x80;
const uint8_t I2C_READ_FLAG = 0x80;
const uint8_t MOT_DETECT_CTRL = 0x69;
const uint8_t ACCEL_INTEL_EN = 0x80;
const uint8_t ACCEL_INTEL_MODE = 0x40;
const uint8_t LP_ACCEL_ODR = 0x1E;
const uint8_t WOM_THR = 0x1F;
const uint8_t WHO_AM_I = 0x75;
const uint8_t WHO_AM_I_VALUE_1 = 0x71;
const uint8_t WHO_AM_I_VALUE_2 = 0x73;
const uint8_t FIFO_EN = 0x23;
const uint8_t FIFO_TEMP = 0x80;
const uint8_t FIFO_GYRO = 0x70;
const uint8_t FIFO_ACCEL = 0x08;
const uint8_t FIFO_MAG = 0x01;
const uint8_t FIFO_COUNT = 0x72;
const uint8_t FIFO_READ = 0x74;

// AK8963 registers address and value
// Who Am I
const uint8_t AK8963_WHO_AM_I = 0x00;
const uint8_t AK8963_WHO_AM_I_VALUE = 0x48;
// Control 1
const uint8_t AK8963_CNTL1 = 0x0A;
const uint8_t AK8963_PWR_DOWN = 0x00;  // Power-down mode
// Control 2
const uint8_t AK8963_CNTL2 = 0x0B;
const uint8_t AK8963_RESET = 0x01;  // When “1” is set, all registers are initialized. After reset, SRST bit turns to “0” automatically.


const uint8_t AK8963_I2C_ADDR = 0x0C;
const uint8_t AK8963_HXL = 0x03;


const uint8_t AK8963_CNT_MEAS1 = 0x12;
const uint8_t AK8963_CNT_MEAS2 = 0x16;
const uint8_t AK8963_FUSE_ROM = 0x0F;


const uint8_t AK8963_ASA = 0x10;


static uint8_t _buffer[21];
static uint8_t _mag_adjust[3];

#ifdef MPU9250_USE_IIC
static bool MPU9250_IsConnected()
{
    if(HAL_I2C_IsDeviceReady(&MPU9250_I2C_CHANNEL,DEVICE_ADDRESS,1,HAL_MAX_DELAY)==HAL_OK)
        return true;
    else
        return false;
}

static void MPU_I2C_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&MPU9250_I2C_CHANNEL,DEVICE_ADDRESS,WriteAddr,I2C_MEMADD_SIZE_8BIT,pBuffer,NumByteToWrite,HAL_MAX_DELAY);
    if(HAL_OK != status)
    {
        printf("I2C write error: %d", status);
    }
}

static void MPU_I2C_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
    uint8_t data = ReadAddr | READWRITE_CMD;
    HAL_StatusTypeDef status;
    status = HAL_I2C_Master_Transmit(&MPU9250_I2C_CHANNEL,DEVICE_ADDRESS,&data,1,HAL_MAX_DELAY);
    if(HAL_OK != status)
    {
        printf("HAL_I2C_Master_Transmit error: %d", status);
    }
    status = HAL_I2C_Master_Receive(&MPU9250_I2C_CHANNEL,DEVICE_ADDRESS,pBuffer,NumByteToRead,HAL_MAX_DELAY);
    if(HAL_OK != status)
    {
        printf("HAL_I2C_Master_Receive error: %d", status);
    }
}
#else  // SPI
__weak void MPU9250_OnActivate()
{
}

static inline void MPU9250_Activate()
{
    MPU9250_OnActivate();
    HAL_GPIO_WritePin(MPU9250_CS_GPIO, MPU9250_CS_PIN, GPIO_PIN_RESET);
}

static inline void MPU9250_Deactivate()
{
    HAL_GPIO_WritePin(MPU9250_CS_GPIO, MPU9250_CS_PIN, GPIO_PIN_SET);
}

uint8_t SPIx_WriteRead(uint8_t Byte)
{
    uint8_t receivedbyte = 0;
    if(HAL_SPI_TransmitReceive(&hspi1,(uint8_t*) &Byte,(uint8_t*) &receivedbyte,1,0x1000)!=HAL_OK)
    {
        return -1;
    }
    else
    {
    }
    return receivedbyte;
}

void MPU_SPI_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
    MPU9250_Activate();
    SPIx_WriteRead(WriteAddr);
    while(NumByteToWrite>=0x01)
    {
        SPIx_WriteRead(*pBuffer);
        NumByteToWrite--;
        pBuffer++;
    }
    MPU9250_Deactivate();
}

void MPU_SPI_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
    MPU9250_Activate();
    uint8_t data = ReadAddr | READWRITE_CMD;
    HAL_SPI_Transmit(&MPU9250_SPI, &data, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&MPU9250_SPI, pBuffer, NumByteToRead, HAL_MAX_DELAY);
    MPU9250_Deactivate();
}
#endif

/* writes a byte to MPU9250 register given a register address and data */
static void writeRegister(uint8_t subAddress, uint8_t data)
{
    #ifdef MPU9250_USE_IIC
    MPU_I2C_Write(&data, subAddress, 1);
    #else
    MPU_SPI_Write(&data, subAddress, 1);
    #endif
    HAL_Delay(10);
}

/* reads registers from MPU9250 given a starting register address, number of bytes, and a pointer to store data */
static void readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest){
    #ifdef MPU9250_USE_IIC
    MPU_I2C_Read(dest, subAddress, count);
    #else
    MPU_SPI_Read(dest, subAddress, count);
    #endif
}

/* writes a register to the AK8963 given a register address and data */
static void writeAK8963Register(uint8_t subAddress, uint8_t data)
{
    // set slave 0 to the AK8963 and set for write
    writeRegister(I2C_SLV0_ADDR,AK8963_I2C_ADDR);

    // set the register to the desired AK8963 sub address
    writeRegister(I2C_SLV0_REG,subAddress);

    // store the data for write
    writeRegister(I2C_SLV0_DO,data);

    // enable I2C and send 1 byte
    writeRegister(I2C_SLV0_CTRL,I2C_SLV0_EN | (uint8_t)1);
}

/* reads registers from the AK8963 */
static void readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest)
{
    // set slave 0 to the AK8963 and set for read
    writeRegister(I2C_SLV0_ADDR, AK8963_I2C_ADDR | I2C_READ_FLAG);

    // set the register to the desired AK8963 sub address
    writeRegister(I2C_SLV0_REG,subAddress);

    // enable I2C and request the bytes
    writeRegister(I2C_SLV0_CTRL,I2C_SLV0_EN | count);

    // takes some time for these registers to fill
    HAL_Delay(1);

    // read the bytes off the MPU9250 EXT_SENS_DATA registers
    readRegisters(EXT_SENS_DATA_00,count,dest);
}

/* gets the MPU9250 WHO_AM_I register value, expected to be 0x71 */
static uint8_t whoAmI(){
    // read the WHO_AM_I register
    readRegisters(WHO_AM_I,1,_buffer);

    // return the register value
    return _buffer[0];
}

/* gets the AK8963 WHO_AM_I register value, expected to be 0x48 */
static int whoAmIAK8963(){
    _buffer[0] = 0xff;
    // read the WHO AM I register
    readAK8963Registers(AK8963_WHO_AM_I,1,_buffer);
    // return the register value
    return _buffer[0];
}

// uint8_t initGMeterAndGyro()
// {

// }

/* starts communication with the MPU-9250 */
uint8_t MPU9250_Init()
{
    #ifdef MPU9250_USE_IIC
    while(!MPU9250_IsConnected())
    {
        HAL_Delay(100);
    }
    #endif
    // select PLL as clock source
    writeRegister(PWR_MGMNT_1, CLOCK_SEL_PLL);
    // enable I2C master mode
    writeRegister(USER_CTRL, I2C_MST_EN);
    // set the I2C bus speed to 400 kHz
    writeRegister(I2C_MST_CTRL, I2C_MST_CLK);

    // set AK8963 to Power-down
    writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);
    // reset the MPU9250
    writeRegister(PWR_MGMNT_1,PWR_RESET);  //???reset? restore default setting
    // wait for MPU9250 ready
    HAL_Delay(10);

    // select PLL as clock source
    writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL);  //???

    // check the WHO AM I byte, expected value is 0x71 (decimal 113) or 0x73 (decimal 115)
    uint8_t who = whoAmI();
    if((who != WHO_AM_I_VALUE_1) && ( who != WHO_AM_I_VALUE_2))
    {
        return 1;
    }

    // enable accelerometer and gyro
    writeRegister(PWR_MGMNT_2,SEN_ENABLE);

    // setting accel range to 4G as default
    writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_4G);

    // setting the gyro range to 250DPS as default
    writeRegister(GYRO_CONFIG,GYRO_FS_SEL_250DPS);

    // setting bandwidth to 184Hz as default
    writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_184);

    // setting gyro bandwidth to 184Hz
    writeRegister(CONFIG,GYRO_DLPF_184);

    // setting the sample rate divider to 0 as default
    writeRegister(SMPDIV,0x00);

    // enable I2C master mode
    writeRegister(USER_CTRL,I2C_MST_EN);

    // set the I2C bus speed to 400 kHz
    writeRegister(I2C_MST_CTRL,I2C_MST_CLK);


    // reset the AK8963
    writeAK8963Register(AK8963_CNTL2,AK8963_RESET);
    // wait for AK8963 ready
    HAL_Delay(1000);

    // writeRegister(INT_PIN_CFG,BYPASS_EN);  // enable bypass ????

    // check AK8963 WHO AM I register, expected value is 0x48 (decimal 72)
    if( whoAmIAK8963() != AK8963_WHO_AM_I_VALUE )
    {
        return 1;
    }

    /* get the magnetometer calibration */
    // set AK8963 to Power Down
    writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);

    HAL_Delay(100); // long wait between AK8963 mode changes

    // set AK8963 to FUSE ROM access
    writeAK8963Register(AK8963_CNTL1,AK8963_FUSE_ROM);

    // long wait between AK8963 mode changes
    HAL_Delay(100);

    // read the AK8963 ASA registers and compute magnetometer scale factors
    readAK8963Registers(AK8963_ASA, 3, _mag_adjust);

    // set AK8963 to Power Down
    writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);

    // long wait between AK8963 mode changes
    HAL_Delay(100);

    // set AK8963 to 16 bit resolution, 100 Hz update rate
    writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS2);

    // long wait between AK8963 mode changes
    HAL_Delay(100);

    // select clock source to gyro
    writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL);

    // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
    readAK8963Registers(AK8963_HXL,7,_buffer);

    // successful init, return 0
    return 0;
}

/* sets the accelerometer full scale range to values other than default */
void MPU9250_SetAccelRange(AccelRange range)
{
    writeRegister(ACCEL_CONFIG, range);
}

/* sets the gyro full scale range to values other than default */
void MPU9250_SetGyroRange(GyroRange range)
{
    writeRegister(GYRO_CONFIG, range);
}

/* sets the DLPF bandwidth to values other than default */
void MPU9250_SetDLPFBandwidth(DLPFBandwidth bandwidth)
{
    writeRegister(ACCEL_CONFIG2,bandwidth);
    writeRegister(CONFIG,bandwidth);
}

/* sets the sample rate divider to values other than default */
void MPU9250_SetSampleRateDivider(SampleRateDivider srd)
{
    /* setting the sample rate divider to 19 to facilitate setting up magnetometer */
    writeRegister(SMPDIV,19);

    if(srd > 9)
    {
        // set AK8963 to Power Down
        writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);

        // long wait between AK8963 mode changes
        HAL_Delay(100);

        // set AK8963 to 16 bit resolution, 8 Hz update rate
        writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS1);

        // long wait between AK8963 mode changes
        HAL_Delay(100);

        // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
        readAK8963Registers(AK8963_HXL,7,_buffer);

    }
    else
    {
        // set AK8963 to Power Down
        writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);
        // long wait between AK8963 mode changes
        HAL_Delay(100);
        // set AK8963 to 16 bit resolution, 100 Hz update rate
        writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS2);

        // long wait between AK8963 mode changes
        HAL_Delay(100);

        // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
        readAK8963Registers(AK8963_HXL,7,_buffer);
    }

    writeRegister(SMPDIV, srd);
}

/* read the data, each argument should point to a array for x, y, and x */
void MPU9250_GetData(int16_t* AccData, int16_t* MagData, int16_t* GyroData)
{
    // grab the data from the MPU9250
    readRegisters(ACCEL_OUT, 21, _buffer);

    // combine into 16 bit values
    AccData[0] = (((int16_t)_buffer[0]) << 8) | _buffer[1];
    AccData[1] = (((int16_t)_buffer[2]) << 8) | _buffer[3];
    AccData[2] = (((int16_t)_buffer[4]) << 8) | _buffer[5];
    GyroData[0] = (((int16_t)_buffer[8]) << 8) | _buffer[9];
    GyroData[1] = (((int16_t)_buffer[10]) << 8) | _buffer[11];
    GyroData[2] = (((int16_t)_buffer[12]) << 8) | _buffer[13];

    int16_t magx = (((int16_t)_buffer[15]) << 8) | _buffer[14];
    int16_t magy = (((int16_t)_buffer[17]) << 8) | _buffer[16];
    int16_t magz = (((int16_t)_buffer[19]) << 8) | _buffer[18];

    MagData[0] = (int16_t)((float)magx * ((float)(_mag_adjust[0] - 128) / 256.0f + 1.0f));
    MagData[1] = (int16_t)((float)magy * ((float)(_mag_adjust[1] - 128) / 256.0f + 1.0f));
    MagData[2] = (int16_t)((float)magz * ((float)(_mag_adjust[2] - 128) / 256.0f + 1.0f));
}