/**
 * @file functions.h
 * @author Dr. Daniel Melendrez (github.com/dzalf)
 * @brief
 * @version 0.1
 * @date 15-08-2024
 *
 * @copyright Copyright (c) 2024
 */

#pragma once

#ifndef FUNCTIONS_H
#define FUNCTIONS_H

//---------------------------------------------------------------------
// SW reset AD774X, registers of AD774X are set to factory default
// Input: none
// Output: I2C_State - returns an error of I2C, 0 = without error
//---------------------------------------------------------------------
void AD774X_Reset();

// Reads one register AD774X
// Input: RegAdres - registry address
// Output: content of the registry
// Output: I2C_State - returns an error of I2C, 0 = without error
//---------------------------------------------------------------------
uint8_t AD774X_Read_Single_Register(uint8_t RegAdres);

// Reads the specified number of registers to the TxBuff
// INPUT: RegAdres - first registry address
// INPUT: quantity - number of registers read
// Output: TxBuff - data in the TxBuff field are stored at the registry addresses !!!
// Output: I2C_State - returns an error of I2C, 0 = without error
//---------------------------------------------------------------------
void AD774X_Read_Registers(uint8_t RegAdres, uint8_t *TxBuff, uint8_t quantity);

// Writes one byte to one AD774X register
// Input: RegAdres - registry address
// Input: DataSingl - written data
// Output: I2C_State - returns an error of I2C, 0 = without error
//----------------------------------------------------------------------
uint8_t AD774X_Write_Single_Register(uint8_t RegAdres, uint8_t DataSingl);

// Writes to the registry a defined number of bytes that are stored in the RxBuff field
// Input: RegAdres - first registry address
// Input: RxBuff - buffer where the transmitted data are stored,
//         data to RxBuff must always be saved to register addresses !!!
// Input: quantity - number of registers write
// Output: I2C_State - returns an error of I2C, 0 = without error
//---------------------------------------------------------------------
void AD774X_Write_Registers(uint8_t RegAdres, uint8_t *RxBuff, uint8_t quantity);

void WriteRegistersFromFlash(void);

void WriteRegistersToFlash(void);

void DeleteEEPROM(void);

// automatic offset adjustment,
// it uses CAPDAC and OFFSET registers
//--------------------------------------------------------------------------------------------------
void OffsetAutomaticStart(void);

void OffsetAutomaticBody(void);

void CapdacClear(void);

void OffsetAutomaticEnd(void);

// reading six data registers each sampling period,
// specified by the SamplePeriod variable, and start a new conversion
//----------------------------------------------------------------------
void StartNewConversion(void);

void PeriodicSampling(void);

// conversion of the obtained data to real values
//----------------------------------------------------------------------
long ConvertCapRawData(void);

float ConvertCapData(void);

float ConvertTempData(void);

void SerialPrintData(void);

// function for communication via serial port,
// it allows to write and read AD774X registers
//----------------------------------------------------------------------
void SerialTerminal(void);

#endif // FUNCTIONS_H
