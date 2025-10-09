#include "BQ769x2_protocol.h"
#include <stdio.h>
#define DELAY (32e6)

//******************************************************************************
// BQ Parameters ***************************************************************
//******************************************************************************
// Global Variables for cell voltages, temperatures, Stack voltage, PACK Pin voltage, LD Pin voltage, CC2 current
uint16_t CellVoltage[16] = {0x01, 0x02, 0x03, 0x04, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
float Temperature[3]     = {0, 0, 0};
uint16_t Stack_Voltage   = 0x00;
uint16_t Pack_Voltage    = 0x00;
uint16_t LD_Voltage      = 0x00;
int16_t Pack_Current     = 0x00;
uint16_t AlarmBits       = 0x00;

uint8_t value_SafetyStatusA;  // Safety Status Register A
uint8_t value_SafetyStatusB;  // Safety Status Register B
uint8_t value_SafetyStatusC;  // Safety Status Register C
uint8_t value_PFStatusA;      // Permanent Fail Status Register A
uint8_t value_PFStatusB;      // Permanent Fail Status Register B
uint8_t value_PFStatusC;      // Permanent Fail Status Register C
uint8_t FET_Status;  // FET Status register contents  - Shows states of FETs
uint16_t CB_ActiveCells;  // Cell Balancing Active Cells

uint8_t UV_Fault             = 0;  // under-voltage fault state
uint8_t OV_Fault             = 0;  // over-voltage fault state
uint8_t SCD_Fault            = 0;  // short-circuit fault state
uint8_t OCD_Fault            = 0;  // over-current fault state
uint8_t ProtectionsTriggered = 0;  // Set to 1 if any protection triggers

uint8_t LD_ON = 0;  // Load Detect status bit
uint8_t DSG   = 0;  // discharge FET state
uint8_t CHG   = 0;  // charge FET state
uint8_t PCHG  = 0;  // pre-charge FET state
uint8_t PDSG  = 0;  // pre-discharge FET state
uint8_t DCHG_pin = 0;  // DCHG pin asserted (digital input state)
uint8_t DDSG_pin = 0;  // DDSG pin asserted (digital input state)

uint32_t AccumulatedCharge_Int;   // in AFE_READPASSQ func
uint32_t AccumulatedCharge_Frac;  // in AFE_READPASSQ func
uint32_t AccumulatedCharge_Time;  // in AFE_READPASSQ func

// Arrays
//uint8_t RX_data [2] = {0x00, 0x00};// used for DirectCommand func, used in several functions to store data read from BQ769x2
//uint8_t RX_32Byte [32] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};//used in Subcommands read function
unsigned char RX_32Byte[32] = {0x00};
unsigned char RX_data[2]    = {0x00};

//**********************************BQ Parameters *********************************

//**********************************Function prototypes **********************************
void delayUS(uint16_t us)
{  // Sets the delay in microseconds.
    uint16_t ms;
    char i;
    ms = us / 1000;
    for (i = 0; i < ms; i++) delay_cycles(32000);
}

unsigned char Checksum(unsigned char *ptr, unsigned char len)
// Calculates the checksum when writing to a RAM register. The checksum is the inverse of the sum of the bytes.
{
    unsigned char i;
    unsigned char checksum = 0;

    for (i = 0; i < len; i++) checksum += ptr[i];

    checksum = 0xff & ~checksum;

    return (checksum);
}

unsigned char CRC8(unsigned char *ptr, unsigned char len)
//Calculates CRC8 for passed bytes. Used in i2c read and write functions
{
    unsigned char i;
    unsigned char crc = 0;
    while (len-- != 0) {
        for (i = 0x80; i != 0; i /= 2) {
            if ((crc & 0x80) != 0) {
                crc *= 2;
                crc ^= 0x107;
            } else
                crc *= 2;

            if ((*ptr & i) != 0) crc ^= 0x107;
        }
        ptr++;
    }
    return (crc);
}

void DirectCommands(uint8_t command, uint16_t data, uint8_t type)
// See the TRM or the BQ76952 header file for a full list of Direct Commands
{  //type: R = read, W = write
    uint8_t TX_data[2] = {0x00, 0x00};

    //little endian format
    TX_data[0] = data & 0xff;
    TX_data[1] = (data >> 8) & 0xff;

    if (type == R) {                       //Read
        I2C_ReadReg(command, RX_data, 2);  //RX_data is a global variable
        //        delay_cycles(64000);delay_cycles(64000);
        //        delay_cycles(40000); // for 400k Test
        delayUS(2000);
        delayUS(2000);  //success in 100k
    }
    if (type == W) {  //write
        //Control_status, alarm_status, alarm_enable all 2 bytes long
        I2C_WriteReg(command, TX_data, 2);
        delayUS(2000);
        delayUS(2000);
    }
}

void CommandSubcommands(uint16_t command)  //For Command only Subcommands
// See the TRM or the BQ76952 header file for a full list of Command-only subcommands
{  //For DEEPSLEEP/SHUTDOWN subcommand you will need to call this function twice consecutively

    uint8_t TX_Reg[2] = {0x00, 0x00};

    //TX_Reg in little endian format
    TX_Reg[0] = command & 0xff;
    TX_Reg[1] = (command >> 8) & 0xff;

    I2C_WriteReg(0x3E, TX_Reg, 2);
    delayUS(2000);
}

void Subcommands(uint16_t command, uint16_t data, uint8_t type)
// See the TRM or the BQ76952 header file for a full list of Subcommands
{
    //security keys and Manu_data writes dont work with this function (reading these commands works)
    //max readback size is 32 bytes i.e. DASTATUS, CUV/COV snapshot
    uint8_t TX_Reg[4]    = {0x00, 0x00, 0x00, 0x00};
    uint8_t TX_Buffer[2] = {0x00, 0x00};

    //TX_Reg in little endian format
    TX_Reg[0] = command & 0xff;
    TX_Reg[1] = (command >> 8) & 0xff;

    if (type == R) {  //read
        I2C_WriteReg(0x3E, TX_Reg, 2);
        delayUS(2000);
        I2C_ReadReg(0x40, RX_32Byte, 32);  //RX_32Byte is a global variable
    } else if (type == W) {
        //FET_Control, REG12_Control
        TX_Reg[2] = data & 0xff;
        I2C_WriteReg(0x3E, TX_Reg, 3);
        delayUS(1000);
        TX_Buffer[0] = Checksum(TX_Reg, 3);
        TX_Buffer[1] = 0x05;  //combined length of registers address and data
        I2C_WriteReg(0x60, TX_Buffer, 2);
        delayUS(1000);
    } else if (type == W2) {  //write data with 2 bytes
        //CB_Active_Cells, CB_SET_LVL
        TX_Reg[2] = data & 0xff;
        TX_Reg[3] = (data >> 8) & 0xff;
        I2C_WriteReg(0x3E, TX_Reg, 4);
        delayUS(1000);
        TX_Buffer[0] = Checksum(TX_Reg, 4);
        TX_Buffer[1] = 0x06;  //combined length of registers address and data
        I2C_WriteReg(0x60, TX_Buffer, 2);
        delayUS(1000);
    }
}

void BQ769x2_SetRegister(uint16_t reg_addr, uint32_t reg_data, uint8_t datalen)
{
    uint8_t TX_Buffer[2]  = {0x00, 0x00};
    uint8_t TX_RegData[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    //TX_RegData in little endian format
    TX_RegData[0] = reg_addr & 0xff;
    TX_RegData[1] = (reg_addr >> 8) & 0xff;
    TX_RegData[2] = reg_data & 0xff;  //1st byte of data

    switch (datalen) {
        case 1:  //1 byte datalength
            I2C_WriteReg(0x3E, TX_RegData, 3);
            delayUS(2000);
            TX_Buffer[0] = Checksum(TX_RegData, 3);
            TX_Buffer[1] =
                0x05;  //combined length of register address and data
            I2C_WriteReg(0x60, TX_Buffer, 2);  // Write the checksum and length
            delayUS(2000);
            break;
        case 2:  //2 byte datalength
            TX_RegData[3] = (reg_data >> 8) & 0xff;
            I2C_WriteReg(0x3E, TX_RegData, 4);
            delayUS(2000);
            TX_Buffer[0] = Checksum(TX_RegData, 4);
            TX_Buffer[1] =
                0x06;  //combined length of register address and data
            I2C_WriteReg(0x60, TX_Buffer, 2);  // Write the checksum and length
            delayUS(2000);
            break;
        case 4:  //4 byte datalength, Only used for CCGain and Capacity Gain
            TX_RegData[3] = (reg_data >> 8) & 0xff;
            TX_RegData[4] = (reg_data >> 16) & 0xff;
            TX_RegData[5] = (reg_data >> 24) & 0xff;
            I2C_WriteReg(0x3E, TX_RegData, 6);
            delayUS(2000);
            TX_Buffer[0] = Checksum(TX_RegData, 6);
            TX_Buffer[1] =
                0x08;  //combined length of register address and data
            I2C_WriteReg(0x60, TX_Buffer, 2);  // Write the checksum and length
            delayUS(2000);
            break;
    }
}
//************************************BQ769X2 Functions*********************************
void BQ769x2_Init()
{
    // Configures all parameters in device RAM
    // Enter CONFIGUPDATE mode (Subcommand 0x0090) - It is required to be in CONFIG_UPDATE mode to program the device RAM settings
    // See TRM for full description of CONFIG_UPDATE mode
    CommandSubcommands(SET_CFGUPDATE);
    delayUS(8000);
    CommandSubcommands(SWAP_COMM_MODE);
    delayUS(8000);
    BQ769x2_SetRegister(CommType,0x07,1);
    delayUS(8000);
    CommandSubcommands(SWAP_TO_I2C);
    delayUS(9000);
    CommandSubcommands(SLEEP_DISABLE);
    delayUS(8000);
    
    
    // After entering CONFIG_UPDATE mode, RAM registers can be programmed. When programming RAM, checksum and length must also be
    // programmed for the change to take effect. All of the RAM registers are described in detail in the BQ769x2 TRM.
    // An easier way to find the descriptions is in the BQStudio Data Memory screen. When you move the mouse over the register name,
    // a full description of the register and the bits will pop up on the screen.
//-----------------------CALIBRATION START--------------------------//
//CELLGAIN
    BQ769x2_SetRegister(Cell1Gain, 0x2F41, 2); //cell 1 gain
    BQ769x2_SetRegister(Cell2Gain, 0x2F6C, 2); //cell 2 gain
    BQ769x2_SetRegister(Cell3Gain, 0x2F59, 2); //cell 3 gain
    BQ769x2_SetRegister(Cell4Gain, 0x2F56, 2); //cell 4 gain
    BQ769x2_SetRegister(Cell5Gain, 0x2F46, 2); //cell 5 gain
    BQ769x2_SetRegister(Cell6Gain, 0x2F2E, 2); //cell 6 gain
    BQ769x2_SetRegister(Cell7Gain, 0x2F35, 2); //cell 7 gain
    BQ769x2_SetRegister(Cell8Gain, 0x0000, 2); //cell 8 gain
    BQ769x2_SetRegister(Cell9Gain, 0x0000, 2); //cell 9 gain
    BQ769x2_SetRegister(Cell10Gain, 0x2F4C, 2); //cell 10 gain
//PACK GAIN
    BQ769x2_SetRegister(PackGain, 0x878A, 2); //PACK GAIN
    BQ769x2_SetRegister(TOSGain, 0x83FC, 2); //TOS GAIN
    BQ769x2_SetRegister(LDGain, 0x8214, 2); //LG Gain

//CURRENT
    float gain = .63f;
    uint32_t bitpat;
    memcpy(&bitpat, &gain, sizeof(bitpat));
    BQ769x2_SetRegister(CCGain, bitpat, 4);

    //BQ769x2_SetRegister(CapacityGain, 0x40DC28F6, 4); //Capacity Gain 7.501

    BQ769x2_SetRegister(CoulombCounterOffsetSamples, 0x0040, 2); //Coulumb COunter Offset (64)
    BQ769x2_SetRegister(BoardOffset, 0x03C0, 2); //Coulumb COunter Offset (0) 960

//Temperature OFFSET Default
    BQ769x2_SetRegister(IntGain, 0x632E, 2); //int gain Default (25390)
    BQ769x2_SetRegister(Intbaseoffset, 0x0BD8, 2); //int base default (23032)
    BQ769x2_SetRegister(IntMaximumAD, 0x3FFF, 2); //IntMaximumAD default (16383)
    BQ769x2_SetRegister(IntMaximumTemp, 0x18EB, 2); //IntMaximumtemp default (6379)

//COulumb COunter Deadband
    BQ769x2_SetRegister(CoulombCounterDeadband, 0x0040, 2); //Coulumb COunter 64
//-------------------------SETTINGS START----------------------------//
//MIN FUSE VOLTAGE DISABLE
    BQ769x2_SetRegister(PowerConfig, 0x2D81, 2); //
//REG 12
    BQ769x2_SetRegister(REG12Config, 0x0D, 1); //REG1 = 3.3V
//REG0
    BQ769x2_SetRegister(REG0Config, 0x01, 1); // REG Enable
//HWD Regultator Options disabled 
//Comm Type
    BQ769x2_SetRegister(CommType, 0x0007, 2); // 07 = I2C (for use up to 100 kHz bus speed)
//I2C ADDRESS = Default (0x08) 7 bit
//SPI ADDRESS Default
//Comm Idle Time
    BQ769x2_SetRegister(CommIdleTime, 0x0001, 2); // 1s Comm Idle TIme
//CFETOFF DEFAULT
//DFETOFF DEFAULT
//ALERT Pin Config
    BQ769x2_SetRegister(ALERTPinConfig, 0x00, 1);
// Set TS1 and TS2 to measure Cell Temperature - 0x07
    BQ769x2_SetRegister(TS1Config, 0x0B, 1); //CELL TEMP 0x07
    BQ769x2_SetRegister(TS2Config, 0x0B, 1); //CELL TEMP 0x07
// Set TS3 and HDQto measure FET Temperature - 0x0F
    BQ769x2_SetRegister(TS3Config, 0x0B, 1); // FET TEMP 0F
    BQ769x2_SetRegister(HDQPinConfig, 0x0B,1);  // FET Temp 0F
 // Set DCHG, and DDSG Measure Temp ONLY
    BQ769x2_SetRegister(DCHGPinConfig, 0x0B,1);  // Measure Temp ONLY 0B
    BQ769x2_SetRegister(DDSGPinConfig, 0x0B,1); //  Measure Temp ONLY 0B
 //DA CONFIGURATION
    BQ769x2_SetRegister(DAConfiguration, 0x06,1); //
// 'VCell Mode'
    BQ769x2_SetRegister(VCellMode, 0x027F, 2); // 8 Cells only, 8 and 9 not 
// CC3 Samples
    BQ769x2_SetRegister(CC3Samples, 0x0064, 2); // 100 samples to get the Average

//PROTECTION CONFIGURATION
    BQ769x2_SetRegister(ProtectionConfiguration, 0x0602, 2); // 100 samples to get the Average
//Enabled Protections A,B,C
    BQ769x2_SetRegister(EnabledProtectionsA, 0x9F, 1); // ALL Protections
    BQ769x2_SetRegister(EnabledProtectionsB, 0xFF, 1);  // ALL Protections
    BQ769x2_SetRegister(EnabledProtectionsC, 0x00, 1);  // ALL Protections except PTO, and HWD F0
//Charge Protections A,B,C
    BQ769x2_SetRegister(CHGFETProtectionsA, 0x98, 1); // ALL Protections
    BQ769x2_SetRegister(CHGFETProtectionsB, 0xFD, 1); // ALL Protections D5
    BQ769x2_SetRegister(CHGFETProtectionsC, 0x00, 1); // ALL Protections 50
//DISCharge Protections A,B,C
    BQ769x2_SetRegister(DSGFETProtectionsA, 0x84, 1); // ALL Protections
    BQ769x2_SetRegister(DSGFETProtectionsB, 0xE6, 1); // ALL Protections
    BQ769x2_SetRegister(DSGFETProtectionsC, 0x80, 1); // ALL Protections E0
//BODY DIODE THRESHOLD  100mA
    BQ769x2_SetRegister(BodyDiodeThreshold, 0x0064, 2); // ALL Protections
// 'Default Alarm Mask' - 
    BQ769x2_SetRegister(DefaultAlarmMask, 0xF882, 2);
    BQ769x2_SetRegister(SFAlertMaskA, 0xFC, 1); //SFAlertMaskA Default
    BQ769x2_SetRegister(SFAlertMaskB, 0xF7, 1); //SFAlertMaskB Default
    BQ769x2_SetRegister(SFAlertMaskC, 0xF4, 1); //SFAlertMaskC Default
    BQ769x2_SetRegister(PFAlertMaskA, 0xDF, 1); //PFAlertMaskA 
    BQ769x2_SetRegister(PFAlertMaskB, 0x9F, 1); //PFAlertMaskB Default
    BQ769x2_SetRegister(PFAlertMaskC, 0x00, 1); //PFAlertMaskC Default
    BQ769x2_SetRegister(PFAlertMaskD, 0x00, 1); //PFAlertMaskD Default
//Enabled PF A
    BQ769x2_SetRegister(EnabledPFA, 0xDF, 1); //Enabled PFA 
    BQ769x2_SetRegister(EnabledPFB, 0x1B, 1); //Enabled PFB
    BQ769x2_SetRegister(EnabledPFD, 0x01, 1); //Enabled PFD
//FET_OPTIONS
    BQ769x2_SetRegister(FETOptions, 0x1D, 1); //FET OPTIONS
//Charge Pump Control
    BQ769x2_SetRegister(ChgPumpControl, 0x01, 1); //DEFAULT
//Pre charge start and stop disabled
//Pre Discharge Timeout
    BQ769x2_SetRegister(PredischargeTimeout, 0xC8, 1); //DEFAULT
    BQ769x2_SetRegister(PredischargeStopDelta, 0x32, 1); //DEFAULT
//CURRENT THRESHOLDS
    BQ769x2_SetRegister(DsgCurrentThreshold , 0x64, 1); //System Knows its discharging when below 100mA
    BQ769x2_SetRegister(ChgCurrentThreshold , 0x32, 1); //System Knows its charging when above 50mA
//check time
    BQ769x2_SetRegister(CheckTime , 0x1E, 1); //CHECK CELL CONNECTIONS EVERY 30S 
//CELL INTERCONNECT RESISTABCE DEFAULT

//CELL BALANCING
    BQ769x2_SetRegister(BalancingConfiguration,0xF0,1); //CELL BALANCING
    BQ769x2_SetRegister(MinCellTemp,0xFFEC,2); //Min Cell Temp for Cell Balancing -20C
    BQ769x2_SetRegister(MaxCellTemp,0x003C,2); //Max Cell Temp for Cell Balancing 60C
    BQ769x2_SetRegister(MaxInternalTemp,0x0046,2); //Max internal Temp for Cell Balancing 
    BQ769x2_SetRegister(CellBalanceInterval,0x0014,2); //Cell Balancing every 20S
    BQ769x2_SetRegister(CellBalanceMaxCells,0x0002,2); //Cell Balance 2 cells max
    BQ769x2_SetRegister(CellBalanceMinCellVCharge,0x0D48,2); //Cell Balance only activate after > 3400mV
    BQ769x2_SetRegister(CellBalanceMinDeltaCharge,0x0028,2); //Cell Balance activate when 40mV difference
    BQ769x2_SetRegister(CellBalanceStopDeltaCharge,0x0014,2); //Cell Balance stops when difference is at 20mV
//CELL BALANCE RELAX
    BQ769x2_SetRegister(CellBalanceMinCellVRelax,0x0C80,2); //Cell Balance Deactivate relax at 3200MV
    BQ769x2_SetRegister(CellBalanceMinDeltaRelax,0x0028,2); //Cell Balance activate when 4mV difference
    BQ769x2_SetRegister(CellBalanceStopDeltaRelax,0x0014,2); //Cell Balance stops when difference is at 20mV
//MFG STATUS
    BQ769x2_SetRegister(MfgStatusInit,0x00D0,2); //SET OTP ENABLE HERE
//-------------------------PROTECTIONS START-------------------------//
    // Set up CUV (under-voltage) 
    BQ769x2_SetRegister(CUVThreshold, 0x32, 1); //Trigger at 2530mV
    BQ769x2_SetRegister(CUVDelay, 0x025E, 2); // DELAY 1.5 S
    BQ769x2_SetRegister(CUVRecoveryHysteresis, 0x0001, 2); // Recover at 2642mV

    // Set up COV (over-voltage)
    BQ769x2_SetRegister(COVThreshold, 0x49, 1); //Trigger at 3694mV
    BQ769x2_SetRegister(COVDelay, 0x01C7, 2); // DELAY 1.5 S
    BQ769x2_SetRegister(COVRecoveryHysteresis, 0x0001, 2); // REcover at 3.59V

    //Set up COVL Latch
    BQ769x2_SetRegister(COVLLatchLimit, 0x0005, 2); //increment to 3 COVL will be triggered
    BQ769x2_SetRegister(COVLCounterDecDelay, 0x001E, 2); //Counter Decrement Delay 30 secs
    BQ769x2_SetRegister(COVLRecoveryTime, 0xFF, 1); //Recovery Time 5 Minutes

    // Set up OCC (over-current in charge) 
    BQ769x2_SetRegister(OCCThreshold, 0x005A, 2); //7Ah
    BQ769x2_SetRegister(OCCDelay, 0x7F, 1); //0.4S Delay 

    // Set up OCD1 (over-current in discharge) Threshold 
    BQ769x2_SetRegister(OCD1Threshold, 0x000D, 2); //Trigger at 150A 0x4B
    BQ769x2_SetRegister(OCD1Delay, 0x7F, 1); //0.4S Delay 

    BQ769x2_SetRegister(OCD2Threshold, 0x0064, 2); //Trigger at 200A
    BQ769x2_SetRegister(OCD2Delay, 0x7F, 1); //0.4S Delay 
    
    BQ769x2_SetRegister(OCD3Threshold, 0x9E58, 2); //32A should be 200 A -25000 9E58
    BQ769x2_SetRegister(OCD3Delay, 0x0002, 2); //delay 2S

    //OCD RECOVERY THRESHOLD
    BQ769x2_SetRegister(OCDRecoveryThreshold, 0x0032, 2); //32A should be 250 A

    // Set up SCD Threshold 
    BQ769x2_SetRegister(SCDThreshold, 0x0A, 1); //250mV
    BQ769x2_SetRegister(SCDDelay, 0x0002, 2); //30 micro second
    BQ769x2_SetRegister(SCDRecoveryTime, 0x0005, 2); //5s recovery time

    //OCDL
    BQ769x2_SetRegister(OCDLLatchLimit, 0x0005, 2); //increment 5 
    BQ769x2_SetRegister(OCDLCounterDecDelay, 0x001E, 2); //30 s decrement
    BQ769x2_SetRegister(OCDLRecoveryTime, 0x00B4, 2); //3 minutes
    BQ769x2_SetRegister(OCDLRecoveryThreshold, 0x00C8, 2); //200mah recovery 

    //SCDl
    BQ769x2_SetRegister(SCDLLatchLimit, 0x0001, 2); //increment 5 
    BQ769x2_SetRegister(SCDLCounterDecDelay, 0x001E, 2); //30 s decrement
    BQ769x2_SetRegister(SCDLRecoveryTime, 0x00B4, 2); //3 minutes
    BQ769x2_SetRegister(SCDLRecoveryThreshold, 0x00C8, 2); //200mah recovery

    //OTC
    BQ769x2_SetRegister(OTCThreshold, 0x0032, 2); //OTC triggers at 50C During Charging
    BQ769x2_SetRegister(OTCDelay, 0x0002, 2); //Triggers at 2S
    BQ769x2_SetRegister(OTCRecovery, 0x002D, 2); //Recoveers at 45C

    //OTD
    BQ769x2_SetRegister(OTDThreshold, 0x003C, 2); //OTC triggers at 60C During disCharging
    BQ769x2_SetRegister(OTDDelay, 0x0002, 2); //Triggers at 2S
    BQ769x2_SetRegister(OTDRecovery, 0x0037, 2); //Recovers at 55C

    //OTF
    BQ769x2_SetRegister(OTFThreshold, 0x0050, 2); //OTF triggers at 80C 
    BQ769x2_SetRegister(OTFDelay, 0x0002, 2); //OTF triggers at 2s
    BQ769x2_SetRegister(OTFRecovery, 0x0041, 2); //OTF recovers at 65C 

    //OTINT
    BQ769x2_SetRegister(OTINTThreshold, 0x0055, 2); //OTint triggers at 85C 
    BQ769x2_SetRegister(OTINTDelay, 0x0002, 2); //OTint delay at 2s
    BQ769x2_SetRegister(OTINTRecovery, 0x0050, 2); //OTint recovers at 80C 

    //UTC
    BQ769x2_SetRegister(UTCThreshold, 0x0000, 2); //UTC triggers at 0C 
    BQ769x2_SetRegister(UTCDelay, 0x0002, 2); // triggers at 2S 
    BQ769x2_SetRegister(UTCRecovery, 0x0014, 2); //UTC recovers at 20C 

    //UTD
    BQ769x2_SetRegister(UTDThreshold, 0x0000, 2); //UTD triggers at 0C 
    BQ769x2_SetRegister(UTDDelay, 0x0002, 2); // triggers at 2S
    BQ769x2_SetRegister(UTDRecovery, 0x0014, 2); //UTC recovers at 20C 

    //UTINT
    BQ769x2_SetRegister(UTINTThreshold, 0xFFEC, 2); //UTint triggers at -20C 
    BQ769x2_SetRegister(UTINTDelay, 0x0002, 2); // triggers at 2S
    BQ769x2_SetRegister(UTINTRecovery, 0x0014, 2); //UTint Recovers at 20C

    //Recovery Time
    BQ769x2_SetRegister(ProtectionsRecoveryTime, 0x0003, 2); // triggers at 3S

    //HWD Disabled
    //Protections Load Detect Disabled
    //PTO Disabled
//---------------------------PROTECTIONS ENDED--------------------------------------------//

//---------------------------PERMANENT FAIL VALUES START---------------------------------------//
    //CUDEP
    BQ769x2_SetRegister(CUDEPThreshold, 0x0708, 2); //CUDEP at 1800mV
    BQ769x2_SetRegister(CUDEPDelay, 0x0005, 2); // triggers at 5S
    //SUV
    BQ769x2_SetRegister(SUVThreshold, 0x07D0, 2); //SUV at 2000mV
    BQ769x2_SetRegister(SUVDelay, 0x0005, 2); //SUV triggers at 5S
    //SOV
    BQ769x2_SetRegister(SOVThreshold, 0x0ED8, 2); //SOV at 3800mV
    BQ769x2_SetRegister(SOVDelay, 0x0005, 2); //SOV triggers at 5S
    //TOS
    BQ769x2_SetRegister(TOSSThreshold, 0x01F4, 2); //TOS at 500mV
    BQ769x2_SetRegister(TOSSDelay, 0x0005, 2); //TOS at 5S
    //SOCC
    BQ769x2_SetRegister(SOCCThreshold, 0x1194, 2); //SOCC Triggers at 45Ah Charging Current
    BQ769x2_SetRegister(SOCCDelay, 0x0005, 2); //SOCC Triggers at 5S
    //SOCD -250,000 = -25,000
    BQ769x2_SetRegister(SOCDThreshold, 0x9E58, 2); //SOCD
    BQ769x2_SetRegister(SOCDDelay, 0x0005, 2); //SOCD at 5S
    //SOT
    BQ769x2_SetRegister(SOTThreshold, 0x0050, 2); //SOT at 80C
    BQ769x2_SetRegister(SOTDelay, 0x0005, 2); //SOT at5S
    //SOTF
    BQ769x2_SetRegister(SOTFThreshold, 0x0055, 2); //SOTF at 80C
    BQ769x2_SetRegister(SOTFDelay, 0x0005, 2); //SOTF at 5S
    //VIMR - VOltage check imbalance while at Rest
    BQ769x2_SetRegister(VIMRCheckVoltage, 0x0C80, 2); //Check ALl CEll Voltage at 3200mV
    BQ769x2_SetRegister(VIMRMaxRelaxCurrent, 0x0032, 2); //Check when the load is < 50mA 
    BQ769x2_SetRegister(VIMRThreshold, 0x01F4, 2); //Triggers when imbalance is >500mV
    BQ769x2_SetRegister(VIMRDelay, 0x0005, 2); //Triggers only at 5S
    BQ769x2_SetRegister(VIMRRelaxMinDuration, 0x012C, 2); //Checking begin after 5 mins
    // VIMA - VOltage imbalance check while active
    BQ769x2_SetRegister(VIMACheckVoltage, 0x0DAC, 2); //Check ALl CEll Voltage at 3500mV
    BQ769x2_SetRegister(VIMAMinActiveCurrent, 0x0032, 2); //>50ma check the imbalance
    BQ769x2_SetRegister(VIMAThreshold, 0x00C8, 2); //triggers when there are 200mV imbalance
    BQ769x2_SetRegister(VIMADelay, 0x0005, 2); //Triggers only at 5S
    //CFETOFF
    BQ769x2_SetRegister(CFETFOFFThreshold, 0x0014, 2); //Triggers when >= 20mA is still flowing when off
    BQ769x2_SetRegister(CFETFOFFDelay, 0x0005, 2); //Triggersat 5S
    //DFETOFF
    BQ769x2_SetRegister(DFETFOFFThreshold, 0xFFEC, 2); //Triggers when >= -20mA is still flowing when off
    BQ769x2_SetRegister(DFETFOFFDelay, 0x0005, 2); //Triggers when >= -20mA is still flowing when off
    //VSSF Disabled //2LVL Disabled //LFOF Dsiabled //HWMX Disabled
    
//---------------------------PERMANENT FAIL VALUES END---------------------------------------//

    
    CommandSubcommands(FET_ENABLE);
    delayUS(500);
    CommandSubcommands(ALL_FETS_ON);
    delayUS(500);
    CommandSubcommands(SWAP_COMM_MODE);
    delayUS(8000);
    BQ769x2_SetRegister(CommType,0x07,1);
    delayUS(8000);
    //MFG STATUS
    BQ769x2_SetRegister(MfgStatusInit,0x00D0,2); //SET OTP ENABLE HERE
    // Exit CONFIGUPDATE mode  - Subcommand 0x0092
    CommandSubcommands(EXIT_CFGUPDATE);
    delayUS(8000);
}

// ********************************* BQ769x2 Status and Fault Commands   *****************************************

void BQ769x2_ReadAlarmStatus()
{
    // Read this register to find out why the ALERT pin was asserted
    DirectCommands(AlarmStatus, 0x00, R);
    AlarmBits = (uint16_t) RX_data[1] * 256 + (uint16_t) RX_data[0];
    printf("AlarmBits: 0x%04X (%u)\n", AlarmBits, AlarmBits);
}

void BQ769x2_ReadSafetyStatus()
{
    printf("Entered BQ769x2_ReadSafetyStatus()\n");

    // Read Safety Status A
    DirectCommands(SafetyStatusA, 0x00, R);
    value_SafetyStatusA = (RX_data[1] * 256 + RX_data[0]);
    printf("SafetyStatusA: 0x%04X (%u)\n", value_SafetyStatusA, value_SafetyStatusA);

    // Decode Faults from A
    UV_Fault  = ((0x4 & RX_data[0]) >> 2);
    OV_Fault  = ((0x8 & RX_data[0]) >> 3);
    SCD_Fault = ((0x8 & RX_data[1]) >> 3);
    OCD_Fault = ((0x2 & RX_data[1]) >> 1);

    printf("UV Fault: %d\n", UV_Fault);
    printf("OV Fault: %d\n", OV_Fault);
    printf("SCD Fault: %d\n", SCD_Fault);
    printf("OCD Fault: %d\n", OCD_Fault);

    // Read Safety Status B
    DirectCommands(SafetyStatusB, 0x00, R);
    value_SafetyStatusB = (RX_data[1] * 256 + RX_data[0]);
    printf("SafetyStatusB: 0x%04X (%u)\n", value_SafetyStatusB, value_SafetyStatusB);

    // Read Safety Status C
    DirectCommands(SafetyStatusC, 0x00, R);
    value_SafetyStatusC = (RX_data[1] * 256 + RX_data[0]);
    printf("SafetyStatusC: 0x%04X (%u)\n", value_SafetyStatusC, value_SafetyStatusC);

    // Check if any protection was triggered
    if ((value_SafetyStatusA + value_SafetyStatusB + value_SafetyStatusC) > 1) {
        ProtectionsTriggered = 1;
        printf("Protections Triggered!\n");
    } else {
        ProtectionsTriggered = 0;
        printf("No Protections Triggered.\n");
    }
}


void BQ769x2_ReadPFStatus()
{
    printf("PF STATUS START\n");
    // Read Permanent Fail Status A/B/C and find which bits are set
    // This shows which permanent failures have been triggered
    DirectCommands(PFStatusA, 0x00, R);
    printf("Direct ok");
    value_PFStatusA = ((RX_data[1] << 8) + RX_data[0]);
    printf("PFStatusA: 0x%04X (%u)\n", value_PFStatusA, value_PFStatusA);

    DirectCommands(PFStatusB, 0x00, R);
    value_PFStatusB = ((RX_data[1] << 8) + RX_data[0]);
    printf("PFStatusB: 0x%04X (%u)\n", value_PFStatusB, value_PFStatusB);

    DirectCommands(PFStatusC, 0x00, R);
    value_PFStatusC = ((RX_data[1] << 8) + RX_data[0]);
    printf("PFStatusC: 0x%04X (%u)\n", value_PFStatusC, value_PFStatusC);
    printf("PF STATUS END \n");
}


// ********************************* End of BQ769x2 Status and Fault Commands   *****************************************

// ********************************* BQ769x2 Measurement Commands   *****************************************

uint16_t BQ769x2_ReadVoltage(uint8_t command)
// This function can be used to read a specific cell voltage or stack / pack / LD voltage
{
    //RX_data is global var
    DirectCommands(command, 0x00, R);
    if (command >= Cell1Voltage &&
        command <= Cell16Voltage) {  //Cells 1 through 16 (0x14 to 0x32)
        return (RX_data[1] * 256 + RX_data[0]);  //voltage is reported in mV
    } else {                                     //stack, Pack, LD
        return 10 * (RX_data[1] * 256 +
                        RX_data[0]);  //voltage is reported in 0.01V units
    }
}

void BQ769x2_ReadAllVoltages()
// Reads all cell voltages, Stack voltage, PACK pin voltage, and LD pin voltage
{
    unsigned char x;
    int cellvoltageholder = Cell1Voltage;  //Cell1Voltage is 0x14|
    for (x = 0; x < 10; x++) {

        print_timestamp();
        printf("cell voltage", x+1, cellvoltageholder);
        CellVoltage[x] = BQ769x2_ReadVoltage(cellvoltageholder);
        int mv = CellVoltage[x];

        // convert to volts with 3 decimal places (millivolt resolution)
        int volts = mv / 1000;        // integer part, e.g. 3
        int milli = mv % 1000;        // fractional part in mV, e.g. 376
        if (milli < 0) milli = -milli; // safety for negative values

        printf(" %d: %d.%03d V\n", x + 1, volts, milli);
        cellvoltageholder += 2;
                            }

    Stack_Voltage = BQ769x2_ReadVoltage(StackVoltage);
    Pack_Voltage  = BQ769x2_ReadVoltage(PACKPinVoltage);
    LD_Voltage    = BQ769x2_ReadVoltage(LDPinVoltage);

    int volts = Stack_Voltage / 1000;
    int milli = Stack_Voltage % 1000;
    print_timestamp();
    printf("Stack Voltage: %d.%03d V\n", volts, milli);

}

void BQ769x2_ReadCurrent()
// Reads PACK current
{
    DirectCommands(CC2Current, 0x00, R);
    Pack_Current =
        (int16_t)((uint16_t) RX_data[1] * 256 +
                  (uint16_t) RX_data[0]);  // current is reported in mA

        // Convert to A with 2 decimals
    int scaled_mA = Pack_Current * 10;   // keep your *10 scaling

    // Convert to A with 2 decimals
    int amps = scaled_mA / 1000;                 // whole part
    int hundredths = abs(scaled_mA % 1000) / 10; // fractional part
    print_timestamp();
    printf("Current: %d.%02d A\n", amps, hundredths);
}

float BQ769x2_ReadTemperature(uint8_t command)
{
    DirectCommands(command, 0x00, R);
    //RX_data is a global var
    return (0.1 * (float) (RX_data[1] * 256 + RX_data[0])) -
           273.15;  // converts from 0.1K to Celcius
}

void BQ769x2_ReadAllTemperatures()
{
    static const char *labels[6] = {
        "TS1", "TS2", "TS3", "HDQ", "DCHG", "DDSG"
    };

    printf("Temperature reading start\n");

    Temperature[0] = BQ769x2_ReadTemperature(TS1Temperature);
    Temperature[1] = BQ769x2_ReadTemperature(TS2Temperature);
    Temperature[2] = BQ769x2_ReadTemperature(TS3Temperature);
    Temperature[3] = BQ769x2_ReadTemperature(HDQTemperature);
    Temperature[4] = BQ769x2_ReadTemperature(DCHGTemperature);
    Temperature[5] = BQ769x2_ReadTemperature(DDSGTemperature);

    for (int i = 0; i < 6; i++) {
        print_timestamp();
        printf("Temperature %s: %.2f °C\n", labels[i], Temperature[i]);
    }

    printf("Temperature reading end\n");
}


void BQ769x2_ReadPassQ()
{  // Read Accumulated Charge and Time from DASTATUS6
    printf("Read pass starts\n");
    Subcommands(DASTATUS6, 0x00, R);
    AccumulatedCharge_Int  = ((RX_32Byte[3] << 24) + (RX_32Byte[2] << 16) +
                             (RX_32Byte[1] << 8) + RX_32Byte[0]);  //Bytes 0-3
    AccumulatedCharge_Frac = ((RX_32Byte[7] << 24) + (RX_32Byte[6] << 16) +
                              (RX_32Byte[5] << 8) + RX_32Byte[4]);  //Bytes 4-7
    AccumulatedCharge_Time =
        ((RX_32Byte[11] << 24) + (RX_32Byte[10] << 16) + (RX_32Byte[9] << 8) +
            RX_32Byte[8]);  //Bytes 8-11

    printf("Accumulated Charge (Integer Part): %ld Coulombs\n", AccumulatedCharge_Int);
    printf("Accumulated Charge (Fractional Part): %ld (LSB units)\n", AccumulatedCharge_Frac);
    printf("Accumulated Time: %ld ms\n", AccumulatedCharge_Time);
    printf("Read pass eneds\n");
}

void BQ769x2_EnableAllFETs()  // Enable FET and Trun on all FET
{
    CommandSubcommands(FET_ENABLE);
    delayUS(500);
    CommandSubcommands(ALL_FETS_ON);
    delayUS(500);

    printf("BQ769x2: All FETs enabled successfully.\n");
}

void BQ769x2_DisableAllFETs()

{
    CommandSubcommands(ALL_FETS_OFF);
    delayUS(500);
    printf("BQ769x2: All FETs disabled successfully.\n");
}

void BQ769x2_SwapI2C()

{
    CommandSubcommands(SWAP_TO_I2C);
    delayUS(1000);
    printf("BQ769x2: SWAP\n");
}


void BQ769x2_SleepDisable()

{
    CommandSubcommands(SLEEP_DISABLE);
    delayUS(500);
    printf("BQ769x2: SLEEP DISABLED\n");
}

void BQ769x2_ReadFETStatus(void)
{
    // Read 0x7F (FET Status) via your DirectCommands helper which fills RX_data[0..1]
    DirectCommands(FETStatus, 0x00, R);

    // The FET Status is an 8-bit field (returned in RX_data[0]); RX_data[1] may be 0 or reserved.
    uint8_t fet = RX_data[0];
    FET_Status = fet; // raw

    CHG      = (fet & (1 << 0)) ? 1 : 0;  // CHG_FET
    PCHG     = (fet & (1 << 1)) ? 1 : 0;  // PCHG_FET
    DSG      = (fet & (1 << 2)) ? 1 : 0;  // DSG_FET
    PDSG     = (fet & (1 << 3)) ? 1 : 0;  // PDSG_FET
    DCHG_pin = (fet & (1 << 4)) ? 1 : 0;  // DCHG pin asserted
    DDSG_pin = (fet & (1 << 5)) ? 1 : 0;  // DDSG pin asserted
    LD_ON    = (fet & (1 << 6)) ? 1 : 0;  // ALERT/ALRT_PIN state


    // Optional: set a combined human-readable string or flags
    print_timestamp();
    printf("FET Status: 0x%02X  CHG:%d PCHG:%d DSG:%d PDSG:%d DCHG_pin:%d DDSG_pin:%d ALERT:%d\n",
       fet, CHG, PCHG, DSG, PDSG, DCHG_pin, DDSG_pin, LD_ON);

}



//************************************End of BQ769x2 Measurement Commands******************************************