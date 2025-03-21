

/********************** The automatic offset adjustment ************
   The automatic offset adjustment, it is command "oo", works only for the ranges
   0 - 8.192pF (0 - 4.096pF for AD7745/46) as CDC single-ended input configuration
   and +- 8.192pF (+- 4.096pF for AD7745/46) as CDC differential input configuration.
   For other ranges it is necessary to set the offset manually, because a universal
   algorithm would be unnecessarily complex.
   In the case of  AD7746, only the currently active channel is balanced.
   Be sure to make the basic settings in the AD774X_Init section.
   The message "AD774X not responding !" indicates that the AD774X probably not
   connected properly.

********************** Serial Terminal Commands *********************
  - the baud rate is 115200
  - number of digits for a command with parameter must be preserved, excess zeros must not be deleted
  - sent command must be terminated with codes 0x0D + 0x0A ie. CR + LF or \r\n,
    most PC terminals doing this by default (Arduino Serial Monitor must be set to "Both NL&CR")
  - macros can be created if the PC terminal allows it, but each command must be from the next command
    separated by codes 0x0D + 0x0A
  - registers 00 - 06 are read only and registers 15 - 18 are factory calibrated, they are not saved to flash memory
  - the sampling period variable SamplePeriod (command pwdddd) is a program variable. Minimal possible interval is determined
    by conversion time, set in CONFIGURATION registers. The SamplingPeriod variable is also stored in flash memory by the FW command.

          Signs in the following text: h = hexadeximal digit, d = decimal digit

  fw     FLASH WRITE      writes current registers 07-14 and variable SamplePeriod to EEPROM memory as default after RESET / POR
  fr     FLASH RESET      clears the EEPROM memory area in use, registers 07-14 are initialized from PROGMEM after RESET / POR
  nn     NOP              no operation - delay 250 ms
  oo     OFFSET           automatic offset compensation
  or     OFFSET RESET     clear offset compensation (CAPDACs = off, OFFSET = 0x8000 the middle of the interval)
  pr     PERIODE READ     prints the sampling period of the converter, in the decimal form dddd, the unit is time in [ms]
  pwdddd PERIODE WRITE    write setting the AD774X sampling period, dddd are four digits 0000-9999, the unit is time in [ms]
  rr     REGISTERS READ   prints all registers in the form "R00 = hh R01 = hh… R18 = hh"
  rrdd   REGISTER READ    prints one register in the form "Rdd = hh"
  rwddhh REGISTER WRITE   writes to the register dd hexadecimal value hh, dd are two decimal digits in the range 00 - 18
  ss     SAMPLE STOP      disables / enables periodic sampling of AD774X and printing of data, default is OFF !!!
  tt     TEST THEM        reads the data registers once and displays them. Makes meaning only when sampling is stop.
  vv     VERSION          displays the version of this software
  xx     XINDL            restart Arduino with SW reset AD774X including setting its default values

  Example:
  rw10A2<CR+LF>                   write to configuration register number 10 value 0xA2
                                  identical is RW10a2<CR+LF>
  pw0300<CR+LF>                   changes the sampling period to 300ms

  Example makro:
  SS<CR+LF>                       use it separately for stop sampling
  RW10A2<CR+LF>NN<CR+LF>TT<CR+LF> and then this as macro - write to configuration
                                  register number 10 value 0xA2(start single conversion),
                                  set delay 250ms for conversion and display recieved data
*/

#include "libraries.h"

//* Variables

uint8_t StepByStep = 0;                     // auxiliary variables to phase the offset adjustment process
unsigned long TimeTemp;                     // auxiliary variables for timing
uint8_t SxBuff[SxBuffLength + 1];           // input buffer for serial parser
uint8_t RTxBuff[20];                        // I/O buffer for AD774X registers
uint8_t I2C_State = 0;                      // status of I2C bus, 0 = without error
unsigned int SamplePeriod = 1000;           // sample period in [ms]
float C1 = 0, C2 = 0;                       // auxiliary variables for zero correction calculation
float Capacitance = 0.0, Temperature = 0.0; // real data
bool EnablePeriodicSampling = false;        // periodic sampling with output to serial port, is default disabled
bool EnableSerialTerminal = true;           // enable input from serial port
bool EnableOffsetAutomatic = false;         // enable automatic offset, better said automatic zero setting, is stopped as default

//----------------------------------------------------------------------
// declare Arduino reset function at address 0
//----------------------------------------------------------------------
#ifdef ESP32
#warning "Compiling for ESP32"
#else
void (*resetFunc)(void) = 0; // --> compatible with Arduino
#endif
//----------------------------------------------------------------------

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Serial.print(F("\r\nArduino Restart"));
  //--------------------------------------------------------------------
  // SW reset and init AD774X
  // AD774X registers 07-14 are initialized from PROGMEM before using of the command FW - Flash Write
  // for the first time. After using of FW command, registers are initialized only from EEPROM.
  // PROGMEM reuse is possible after application of the FR command. The commands are entered via
  // the serial interface as described in section AD774X_Comment.
  //--------------------------------------------------------------------
  AD774X_Reset();

  if (I2C_State != 0)
    Serial.print(F("\r\nAD774X not responding !"));
  WriteRegistersFromFlash();
  StartNewConversion();
  Serial.print(F("\r\nI'm waiting for commands:"));
}

//----------------------------------------------------------------------
void loop()
{
  if (EnableOffsetAutomatic)
    OffsetAutomaticBody();
  if (EnablePeriodicSampling)
    PeriodicSampling();
  if (EnableSerialTerminal)
    SerialTerminal();
}

/*
 * Functions definitions
 */

//---------------------------------------------------------------------
// SW reset AD774X, registers of AD774X are set to factory default
// Input: none
// Output: I2C_State - returns an error of I2C, 0 = without error
//---------------------------------------------------------------------
void AD774X_Reset()
{
  Wire.beginTransmission(AD774X_ADDRESS);
  Wire.write(0xBF);
  I2C_State = Wire.endTransmission();
  delay(1);
}

//---------------------------------------------------------------------
// Reads one register AD774X
// Input: RegAdres - registry address
// Output: content of the registry
// Output: I2C_State - returns an error of I2C, 0 = without error
//---------------------------------------------------------------------
uint8_t AD774X_Read_Single_Register(uint8_t RegAdres)
{
  Wire.beginTransmission(AD774X_ADDRESS);
  Wire.write(RegAdres);
  I2C_State = Wire.endTransmission(false);
  Wire.requestFrom(AD774X_ADDRESS, OneByte);
  return Wire.read();
}

//---------------------------------------------------------------------
// Reads the specified number of registers to the TxBuff
// INPUT: RegAdres - first registry address
// INPUT: quantity - number of registers read
// Output: TxBuff - data in the TxBuff field are stored at the registry addresses !!!
// Output: I2C_State - returns an error of I2C, 0 = without error
//---------------------------------------------------------------------
void AD774X_Read_Registers(uint8_t RegAdres, uint8_t *TxBuff, uint8_t quantity)
{
  Wire.beginTransmission(AD774X_ADDRESS);
  Wire.write(RegAdres);
  I2C_State = Wire.endTransmission(false);
  Wire.requestFrom(AD774X_ADDRESS, quantity);
  for (uint8_t i = 0; i < quantity; i++)
  {
    TxBuff[RegAdres + i] = Wire.read();
  }
}

//---------------------------------------------------------------------
// Writes one byte to one AD774X register
// Input: RegAdres - registry address
// Input: DataSingl - written data
// Output: I2C_State - returns an error of I2C, 0 = without error
//----------------------------------------------------------------------
uint8_t AD774X_Write_Single_Register(uint8_t RegAdres, uint8_t DataSingl)
{
  Wire.beginTransmission(AD774X_ADDRESS);
  Wire.write(RegAdres);
  Wire.write(DataSingl);
  I2C_State = Wire.endTransmission();

  return I2C_State;
}

//---------------------------------------------------------------------
// Writes to the registry a defined number of bytes that are stored in the RxBuff field
// Input: RegAdres - first registry address
// Input: RxBuff - buffer where the transmitted data are stored,
//         data to RxBuff must always be saved to register addresses !!!
// Input: quantity - number of registers write
// Output: I2C_State - returns an error of I2C, 0 = without error
//---------------------------------------------------------------------
void AD774X_Write_Registers(uint8_t RegAdres, uint8_t *RxBuff, uint8_t quantity)
{
  Wire.beginTransmission(AD774X_ADDRESS);
  Wire.write(RegAdres);
  Wire.write(RxBuff + RegAdres, quantity);
  I2C_State = Wire.endTransmission();
}

//----------------------------------------------------------------------
void WriteRegistersFromFlash(void)
{
  // 0xAA is flag for default setting AD774X from EEPROM
  if (EEPROM.read(EEPROMStart) != 0xAA)
  {
    // read registers from PROGMEM
    for (uint8_t i = ADR_CAP_SETUP; i < ADR_CAP_GAINH; i++)
    {
      RTxBuff[i] = pgm_read_byte_near(DefaultRegisters + i);
    }
  }
  else
  {
    // read registers from EEPROM
    for (uint8_t i = ADR_CAP_SETUP; i < ADR_CAP_GAINH; i++)
    {
      RTxBuff[i] = EEPROM.read(i);
    }
    EEPROM.get(EEPROMAddrSamplePeriod, SamplePeriod);
  }
  AD774X_Write_Registers(ADR_CAP_SETUP, RTxBuff, 8);
}

//----------------------------------------------------------------------
void WriteRegistersToFlash(void)
{
  AD774X_Read_Registers(ADR_CAP_SETUP, RTxBuff, 8);
  for (int8_t i = ADR_CAP_SETUP; i < ADR_CAP_GAINH; i++)
  {
    EEPROM.write(i, RTxBuff[i]); // --> Used incompatible .update()
  }
  EEPROM.put(EEPROMAddrSamplePeriod, SamplePeriod);
  // 0xAA is flag for default setting AD774X from EEPROM
  EEPROM.write(EEPROMStart, 0xAA);
}

void DeleteEEPROM(void)
{
  for (uint8_t i = EEPROMStart; i < 19; i++)
    EEPROM.write(i, 0xFF);
}

//--------------------------------------------------------------------------------------------------
// automatic offset adjustment,
// it uses CAPDAC and OFFSET registers
//--------------------------------------------------------------------------------------------------
void OffsetAutomaticStart(void)
{
  // stops unnecessary processes
  EnablePeriodicSampling = false;
  EnableSerialTerminal = false;

  // CAPDACs ON
  RTxBuff[ADR_CAPDACA] = CAPDAC_ON;
  RTxBuff[ADR_CAPDACB] = CAPDAC_ON;

  // the offset registers to the center
  RTxBuff[ADR_CAP_OFFH] = 0x80;
  RTxBuff[ADR_CAP_OFFL] = 0x00;

  // sets this registers
  AD774X_Write_Registers(ADR_CAPDACA, RTxBuff, 4);
  C1 = 0;
  C2 = 0;
  StepByStep = 0;

  // sets VT_SETUP register for internal reference
  AD774X_Write_Single_Register(ADR_VT_SETUP, AD774X_Read_Single_Register(ADR_VT_SETUP) & REFERENCE);

  // sets continual mode for conversion to speed up the compensation process
  AD774X_Write_Single_Register(ADR_CFG, (AD774X_Read_Single_Register(ADR_CFG) & MODES) | CONTIN);

  // enable offset compensation process
  EnableOffsetAutomatic = true;
  TimeTemp = millis();
}

void OffsetAutomaticBody(void)
{

  // ? is an AD774X respond ? - timeout detection
  if (millis() - AD774XTimeOut > TimeTemp)
  {
    Serial.print(F("\r\nAD774X not responding !"));
    CapdacClear();
    OffsetAutomaticEnd();
    return;
  }
  // waits for flag RDYCAP
  if ((AD774X_Read_Single_Register(ADR_STATUS) & CAP_RDY) == 0)
  {

    TimeTemp = millis();

    // auxiliary variable for CAPDACs range testing
    uint8_t TestScale = 16;
    // reads valid data
    AD774X_Read_Registers(ADR_CAP_DATAH, RTxBuff, 3);
    if (StepByStep < HowManySteps)
    {
      // converts valid CAP data, the sum of the deviation patterns from zero
      C1 += ConvertCapData();
      StepByStep++;

      if (StepByStep >= HowManySteps)
      {
        // to accurately determine the capacity range of the CAPDAC register
        if (C1 >= 0)
          AD774X_Write_Single_Register(ADR_CAPDACA, CAPDAC_ON | TestScale);
        else
          AD774X_Write_Single_Register(ADR_CAPDACB, CAPDAC_ON | TestScale);

        Serial.print(F("\r\nPhase 1. terminated"));
      }
      return;
    }

    if (StepByStep < 2 * HowManySteps)
    {
      // Convert valid CAP data, sum of samples
      C2 += ConvertCapData();
      StepByStep++;
      if (StepByStep >= 2 * HowManySteps)
      {
        // rough CAPDACs correction settings
        if (C1 >= 0)
          AD774X_Write_Single_Register(ADR_CAPDACA, (uint8_t)((float)TestScale * C1 / (C1 - C2)) | CAPDAC_ON);
        else
          AD774X_Write_Single_Register(ADR_CAPDACB, (uint8_t)((float)TestScale * C1 / (C1 - C2)) | CAPDAC_ON);
        C1 = 0;
        C2 = 0;
        Serial.print(F("\r\nPhase 2. terminated"));
      }

      return;
    }
    if (StepByStep < 3 * HowManySteps)
    {
      // Convert valid CAP data, sum of samples
      C1 += ConvertCapData();
      StepByStep++;
      if (StepByStep >= 3 * HowManySteps)
      {
        // to accurately determine the capacity range of the OFFSET register
        if (C1 >= 0)
        {
          AD774X_Write_Single_Register(ADR_CAP_OFFH, 0xff);
          AD774X_Write_Single_Register(ADR_CAP_OFFL, 0xff);
        }
        else
        {
          AD774X_Write_Single_Register(ADR_CAP_OFFH, 0x00);
          AD774X_Write_Single_Register(ADR_CAP_OFFL, 0x00);
        }

        Serial.print(F("\r\nPhase 3. terminated"));
      }
      return;
    }

    if (StepByStep < 4 * HowManySteps)
    {
      // Convert valid CAP data, sum of samples
      C2 += ConvertCapData();
      StepByStep++;
      if (StepByStep >= 4 * HowManySteps)
      {
        // calculation CAP_OFFSET correction settings
        float FineOffset = (32768.0 * C1 / (C1 - C2));
        // an error - offset could not be compensated
        if (abs(FineOffset) > 32767)
        {
          Serial.print(F("\r\nPhase 4. terminated"));
          Serial.print(F("\r\nError - Offset could not be compensated !"));
          CapdacClear();
          OffsetAutomaticEnd();
          return;
        }
        // fine CAP_OFFSET correction settings
        if (C1 >= 0)
          FineOffset = 32768.0 + FineOffset;
        else
          FineOffset = 32768.0 - FineOffset;
        AD774X_Write_Single_Register(ADR_CAP_OFFH, (uint8_t)((uint16_t)FineOffset >> 8));
        AD774X_Write_Single_Register(ADR_CAP_OFFL, (uint8_t)((uint16_t)FineOffset & 0x00FF));
        Serial.print(F("\r\nPhase 4. terminated"));
        Serial.print(F("\r\nOffset setting complete !"));
        Serial.print(F("\r\nThis setting can be permanently saved by the FW command !"));
        OffsetAutomaticEnd();
      }
      return;
    }
  }
}

void CapdacClear(void)
{
  // CAPDACs OFF
  RTxBuff[ADR_CAPDACA] = CAPDAC_OFF;
  RTxBuff[ADR_CAPDACB] = CAPDAC_OFF;
  // the offset registers to the center
  RTxBuff[ADR_CAP_OFFH] = 0x80;
  RTxBuff[ADR_CAP_OFFL] = 0x00;
  // sets this registers
  AD774X_Write_Registers(ADR_CAPDACA, RTxBuff, 4);
}

void OffsetAutomaticEnd(void)
{
  // renewal of periodic processes
  EnableOffsetAutomatic = false;
  EnablePeriodicSampling = true;
  EnableSerialTerminal = true;
  // starts first sample after offset setting
  StartNewConversion();
}

//----------------------------------------------------------------------
// reading six data registers each sampling period,
// specified by the SamplePeriod variable, and start a new conversion
//----------------------------------------------------------------------
void StartNewConversion(void)
{
  TimeTemp = millis();
  AD774X_Write_Single_Register(ADR_CFG, (AD774X_Read_Single_Register(ADR_CFG) & MODES) | SINGLE);
}

void PeriodicSampling(void)
{
  if ((millis() - SamplePeriod) > TimeTemp)
  {
    // reading valid data
    AD774X_Read_Registers(ADR_CAP_DATAH, RTxBuff, 6);
    StartNewConversion();
    // convert and list of acquired data
    SerialPrintData();
  }
}

//----------------------------------------------------------------------
// conversion of the obtained data to real values
//----------------------------------------------------------------------
long ConvertCapRawData(void)
{
  return long((long)RTxBuff[ADR_CAP_DATAH] << 16) + ((long)RTxBuff[ADR_CAP_DATAM] << 8) + (long)RTxBuff[ADR_CAP_DATAL] - 0x800000;
}

float ConvertCapData(void)
{
  long CapacitanceRaw = ConvertCapRawData();
  // different calculation for AD7747 and AD7745/46
  if (AD7747)
    return (float)CapacitanceRaw / 1024000.0;
  else
    return (float)CapacitanceRaw / 2048000.0;
}

float ConvertTempData(void)
{
  long TemperatureRaw = ((long)RTxBuff[ADR_VT_DATAH] << 16) + ((long)RTxBuff[ADR_VT_DATAM] << 8) + (long)RTxBuff[ADR_VT_DATAL];
  return (float)TemperatureRaw / 2048.0 - 4096.0;
}

void SerialPrintData(void)
{
  Capacitance = ConvertCapData();
  Temperature = ConvertTempData();
  Serial.println("");
  if (Capacitance >= 0)
    Serial.print(F(" "));
  Serial.print(Capacitance, 6);
  Serial.print(F("  pF    "));
  Serial.print(Temperature, 2);
  Serial.print(F("  deg.C"));
}

//----------------------------------------------------------------------
// function for communication via serial port,
// it allows to write and read AD774X registers
//----------------------------------------------------------------------
void SerialTerminal(void)
{
  // ? are the data in the serial buffer?
  if (Serial.available())
  {
    // rotate the input serial parse buffer to the left by 1 character
    // and discard first character
    for (uint8_t i = 0; i < SxBuffLength; i++)
    {
      SxBuff[i] = SxBuff[i + 1];
    }
    // read one character into end the buffer
    SxBuff[SxBuffLength - 1] = Serial.read();
    //----------------------------------------------------------------------
    //  ? are the termination characters 0x0d + 0x0a at the end of the buffer ?
    //----------------------------------------------------------------------
    if (SxBuff[SxBuffLength - 2] == 0x0D && SxBuff[SxBuffLength - 1] == 0x0A)
    {

      //----------------------------------------------------------------------
      // write registers to flash
      //----------------------------------------------------------------------
      if ((SxBuff[SxBuffLength - 4] == 'f' || SxBuff[SxBuffLength - 4] == 'F') && (SxBuff[SxBuffLength - 3] == 'w' || SxBuff[SxBuffLength - 3] == 'W'))
      {
        WriteRegistersToFlash();
        Serial.print(F("\r\nRegisters 07-14 and SamplePeriod interval are written to EEPROM memory as default"));
      }
      //----------------------------------------------------------------------
      // delete flash memory and sets PROGMEM as default
      //----------------------------------------------------------------------
      if ((SxBuff[SxBuffLength - 4] == 'f' || SxBuff[SxBuffLength - 4] == 'F') && (SxBuff[SxBuffLength - 3] == 'r' || SxBuff[SxBuffLength - 3] == 'R'))
      {
        DeleteEEPROM();
        WriteRegistersFromFlash();
        Serial.print(F("\r\nDeleted EEPROM memory and set PROGMEM as default for registers"));
        StartNewConversion();
      }
      //----------------------------------------------------------------------
      // Automatic capacity channel offset adjustment
      //----------------------------------------------------------------------
      if ((SxBuff[SxBuffLength - 4] == 'o' || SxBuff[SxBuffLength - 4] == 'O') && (SxBuff[SxBuffLength - 3] == 'o' || SxBuff[SxBuffLength - 3] == 'O'))
      {
        Serial.print(F("\r\n!!! Quiet please !!!"));
        Serial.print(F("\r\nAutomatic of capacity channel offset adjustment starting"));
        OffsetAutomaticStart();
      }
      //----------------------------------------------------------------------
      // offset compensation is deleted
      //----------------------------------------------------------------------
      if ((SxBuff[SxBuffLength - 4] == 'o' || SxBuff[SxBuffLength - 4] == 'O') && (SxBuff[SxBuffLength - 3] == 'r' || SxBuff[SxBuffLength - 3] == 'R'))
      {
        CapdacClear();
        Serial.print(F("\r\nOffset compensation is deleted"));
        Serial.print(F("\r\nThis setting can be permanently saved by the FW command !"));
        StartNewConversion();
      }
      //----------------------------------------------------------------------
      // read the sampling period
      //----------------------------------------------------------------------
      if ((SxBuff[SxBuffLength - 4] == 'p' || SxBuff[SxBuffLength - 4] == 'P') && (SxBuff[SxBuffLength - 3] == 'r' || SxBuff[SxBuffLength - 3] == 'R'))
      {
        Serial.print(F("\r\nSampling period [ms]: "));
        Serial.print(SamplePeriod);
      }
      //----------------------------------------------------------------------
      // write the sampling period in [ms]
      //----------------------------------------------------------------------
      if ((SxBuff[SxBuffLength - 8] == 'p' || SxBuff[SxBuffLength - 8] == 'P') && (SxBuff[SxBuffLength - 7] == 'w' || SxBuff[SxBuffLength - 7] == 'W'))
      {
        SamplePeriod = 0;
        for (uint8_t i = SxBuffLength - 6; i < (SxBuffLength - 2); i++)
        {
          SamplePeriod = 10 * SamplePeriod + map(SxBuff[i], 48, 57, 0, 9);
        }
        if (SamplePeriod > 9999)
        {
          SamplePeriod = 1000;
          Serial.print(F("\r\nThe sampling period is too long!"));
        }
        Serial.print(F("\r\nThe sampling period is written [ms]: "));
        Serial.print(SamplePeriod);
        Serial.print(F("\r\nThis setting can be permanently saved by the FW command !"));
      }
      //----------------------------------------------------------------------
      // listing of all registers
      //----------------------------------------------------------------------
      if ((SxBuff[SxBuffLength - 4] == 'r' || SxBuff[SxBuffLength - 4] == 'R') && (SxBuff[SxBuffLength - 3] == 'r' || SxBuff[SxBuffLength - 3] == 'R'))
      {
        AD774X_Read_Registers(ADR_STATUS, RTxBuff, 19);
        Serial.println(F("\r\nList all registers 00 - 18:"));
        for (uint8_t i = ADR_STATUS; i < 19; i++)
        {
          Serial.print(F("R"));
          if (i < 10)
            Serial.print(F("0"));
          Serial.print(i);
          Serial.print(F("="));
          if (RTxBuff[i] < 0x10)
            Serial.print(F("0"));
          Serial.print(RTxBuff[i], HEX);
          Serial.print(F(" "));
        }
      }
      //----------------------------------------------------------------------
      // read one register AD774X
      //----------------------------------------------------------------------
      if ((SxBuff[SxBuffLength - 6] == 'r' || SxBuff[SxBuffLength - 6] == 'R') && (SxBuff[SxBuffLength - 5] == 'r' || SxBuff[SxBuffLength - 5] == 'R'))
      {
        uint8_t RegDecadic = 10 * map(SxBuff[SxBuffLength - 4], 48, 57, 0, 9) + map(SxBuff[SxBuffLength - 3], 48, 57, 0, 9);
        if (RegDecadic > 18)
        {
          Serial.print(F("\r\nThe registry address is out of range!"));
          return;
        }
        AD774X_Read_Registers(RegDecadic, RTxBuff, 1);
        Serial.print(F("\r\nRegistry listing R"));
        if (RegDecadic < 10)
          Serial.print("0");
        Serial.print(RegDecadic);
        Serial.print(F("="));
        if (RTxBuff[RegDecadic] < 0x10)
          Serial.print(F("0"));
        Serial.print(RTxBuff[RegDecadic], HEX);
      }
      //----------------------------------------------------------------------
      // write one register
      //----------------------------------------------------------------------
      if ((SxBuff[SxBuffLength - 8] == 'r' || SxBuff[SxBuffLength - 8] == 'R') && (SxBuff[SxBuffLength - 7] == 'w' || SxBuff[SxBuffLength - 7] == 'W'))
      {
        // convert two decimal characters to a decimal number for addressing the registry
        uint8_t RegDecadic = 10 * map(SxBuff[SxBuffLength - 6], 48, 57, 0, 9) + map(SxBuff[SxBuffLength - 5], 48, 57, 0, 9);
        if (RegDecadic > 18)
        {
          Serial.print(F("\r\nThe registry address is out of range!"));
          return;
        }
        if (RegDecadic < 7)
        {
          Serial.print(F("\r\nThe register is read only!"));
          return;
        }
        // converting two hexadecimal characters per byte and store it in the output buffer
        const char temp[3] = {(const char)SxBuff[SxBuffLength - 4], (const char)SxBuff[SxBuffLength - 3]};
        RTxBuff[RegDecadic] = (uint8_t)strtol(&temp[0], NULL, 16);
        AD774X_Write_Registers(RegDecadic, RTxBuff, 1);
        AD774X_Read_Registers(RegDecadic, RTxBuff, 1);
        Serial.print(F("\r\nRegister is saved R"));
        if (RegDecadic < 10)
          Serial.print(F("0"));
        Serial.print(RegDecadic);
        Serial.print(F("="));
        if (RTxBuff[RegDecadic] < 0x10)
          Serial.print(F("0"));
        Serial.print(RTxBuff[RegDecadic], HEX);
      }
      //----------------------------------------------------------------------
      //  restart Aruino
      //----------------------------------------------------------------------
      if ((SxBuff[SxBuffLength - 4] == 'x' || SxBuff[SxBuffLength - 4] == 'X') && (SxBuff[SxBuffLength - 3] == 'x' || SxBuff[SxBuffLength - 3] == 'X'))
      {
#ifdef ESP32
        ESP.restart();
#else

        resetFunc(); // --> For Arduino Only

#endif
      }
      //----------------------------------------------------------------------
      // sampling OFF / ON
      //----------------------------------------------------------------------
      if ((SxBuff[SxBuffLength - 4] == 's' || SxBuff[SxBuffLength - 4] == 'S') && (SxBuff[SxBuffLength - 3] == 's' || SxBuff[SxBuffLength - 3] == 'S'))
      {
        EnablePeriodicSampling = !EnablePeriodicSampling;
        if (EnablePeriodicSampling)
          Serial.print(F("\r\nPeriodic sampling started"));
        else
          Serial.print(F("\r\nPeriodic sampling stoped"));
      }
      //----------------------------------------------------------------------
      // reads the data registers once and displays them
      //----------------------------------------------------------------------
      if ((SxBuff[SxBuffLength - 4] == 't' || SxBuff[SxBuffLength - 4] == 'T') && (SxBuff[SxBuffLength - 3] == 't' || SxBuff[SxBuffLength - 3] == 'T'))
      {
        AD774X_Read_Registers(ADR_CAP_DATAH, RTxBuff, 6);
        SerialPrintData();
      }
      //----------------------------------------------------------------------
      // NOP - delay 250ms
      //----------------------------------------------------------------------
      if ((SxBuff[SxBuffLength - 4] == 'n' || SxBuff[SxBuffLength - 4] == 'N') && (SxBuff[SxBuffLength - 3] == 'n' || SxBuff[SxBuffLength - 3] == 'N'))
      {
        Serial.print(F("\r\nDelay 250 ms"));
        delay(250);
      }
      //----------------------------------------------------------------------
      // version of installed firmware
      //----------------------------------------------------------------------
      if ((SxBuff[SxBuffLength - 4] == 'v' || SxBuff[SxBuffLength - 4] == 'V') && (SxBuff[SxBuffLength - 3] == 'v' || SxBuff[SxBuffLength - 3] == 'V'))
      {
        Serial.print(F(VERSION));
      }
    }
  }
}
