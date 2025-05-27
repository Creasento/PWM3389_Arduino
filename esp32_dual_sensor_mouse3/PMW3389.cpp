/*
  PMW3389 - Library for interfacing PMW3389 motion sensor module, for ESP32 core

  Copyright (c) 2024, Sunjun Kim

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <cstdint>
#include "PMW3389.hpp"
#include "SROM.h"
#include "Arduino.h"
#include "SPI.h"

#define BEGIN_COM digitalWrite(_ss, LOW); delayMicroseconds(T_NCS_SCLK)
#define END_COM   delayMicroseconds(1); digitalWrite(_ss, HIGH)
#define SPI_BEGIN _spi->beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3)); delayMicroseconds(T_NCS_SCLK)
#define SPI_END   delayMicroseconds(1); _spi->endTransaction()

// //================================================================================
// //	PMW3389 Motion Sensor Module

// // bascially do nothing here
PMW3389::PMW3389()
{
}

// public
/*
begin: initalize variables, prepare the sensor to be init.

# parameter
ss_pin: The arduino pin that is connected to slave select on the module.
CPI: initial CPI. optional.
*/
bool PMW3389::begin(int PIN_SCLK, int PIN_MISO, int PIN_MOSI, uint16_t ss_pin, uint16_t CPI, SPIClass *spi)
{
  // _spi.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, -1); // Note: CS pin is not defined here because we manage it manually
  // _spi.setHwCs(false); // Disable hardware CS pin management
  _ss = ss_pin;
  _spi = spi;
  // _inBurst = false;

  // hard reset
  END_COM;
  delay(100);
  BEGIN_COM;
  END_COM;

  // SPI_BEGIN;
  // adns_write_reg(REG_Power_Up_Reset, 0x5a); // force reset
  writeRegister(REG_Power_Up_Reset, 0x5a); // force reset
  delay(50); // wait for it to reboot

    // read registers 0x02 to 0x06 (and discard the data)
  readRegister(REG_Motion);
  readRegister(REG_Delta_X_L);
  readRegister(REG_Delta_X_H);
  readRegister(REG_Delta_Y_L);
  readRegister(REG_Delta_Y_H);

  // upload the firmware
  upload_firmware();
  // END_COM;

  delay(10);
  setCPI(CPI);
  
  return check_signature();
}

// public
/*
setCPI: set CPI level of the motion sensor.

# parameter
cpi: Count per Inch value
*/
void PMW3389::setCPI(unsigned int cpi)
{
  unsigned int cpival = cpi / 50;

  writeRegister(Resolution_L, (cpival & 0xFF));
  writeRegister(Resolution_H, ((cpival >> 8) & 0xFF));
}

// public
/*
getCPI: get CPI level of the motion sensor.

# retrun
cpi: Count per Inch value
*/
unsigned int PMW3389::getCPI()
{
  // Multiply by 50 to get the CPI value, as set by setCPI
  unsigned int low = readRegister(Resolution_L);
  unsigned int high = readRegister(Resolution_H);

  // Combine the low and high byte values into a single integer
  unsigned int cpival = (high << 8) | low;

  return cpival * 50;;
}

// public
/*
readBurst: get one frame of motion data.

# retrun
type: PMW3389_DATA
*/
PMW3389_DATA PMW3389::readBurst()
{
  unsigned long fromLast = micros() - _lastBurst;
  uint8_t burstBuffer[12];

  if(!_inBurst || fromLast > 500*1000)
  {
    writeRegister(REG_Motion_Burst, 0x00);
    _inBurst = true;
  }

  BEGIN_COM;
  _spi->transfer(REG_Motion_Burst);
  delayMicroseconds(35); // waits for tSRAD

  _spi->transfer(burstBuffer, 12); // read burst buffer
  delayMicroseconds(1); // tSCLK-NCS for read operation is 120ns

  END_COM;
  SPI_END;

  if(burstBuffer[0] & 0b111) // panic recovery, sometimes burst mode works weird.
  {
    _inBurst = false;
  }

  _lastBurst = micros();

  PMW3389_DATA data;

  bool motion = (burstBuffer[0] & 0x80) != 0;
  bool surface = (burstBuffer[0] & 0x08) == 0;   // 0 if on surface / 1 if off surface

  uint8_t xl = burstBuffer[2];    // dx LSB
  uint8_t xh = burstBuffer[3];    // dx MSB
  uint8_t yl = burstBuffer[4];    // dy LSB
  uint8_t yh = burstBuffer[5];    // dy MSB
  uint8_t sl = burstBuffer[10];   // shutter LSB
  uint8_t sh = burstBuffer[11];   // shutter MSB

  int16_t x = (int16_t)((xh<<8) | xl);
  int16_t y = (int16_t)((yh<<8) | yl);
  unsigned int shutter = sh<<8 | sl;

  data.isMotion = motion;
  data.isOnSurface = surface;
  data.dx = x;
  data.dy = y;
  data.SQUAL = burstBuffer[6];
  data.rawDataSum = burstBuffer[7];
  data.maxRawData = burstBuffer[8];
  data.minRawData = burstBuffer[9];
  data.shutter = shutter;

  return data;
}

// // public
// /*
// readReg: get one byte value from the given reg_addr.

// # parameter
// reg_addr: the register address
// # retrun
// byte value on the register.
// */
// byte PMW3389::readReg(byte reg_addr)
// {
//   SPI_BEGIN;
//   byte data = adns_read_reg(reg_addr);
//   SPI_END;
//   return data;
// }

// // public
// /*
// writeReg: write one byte value to the given reg_addr.

// # parameter
// reg_addr: the register address
// data: byte value to be pass to the register.
// */
// void PMW3389::writeReg(byte reg_addr, byte data)
// {
//   SPI_BEGIN;
//   adns_write_reg(reg_addr, data);
//   SPI_END;
// }

/*
adns_upload_firmware: load SROM content to the motion sensor
*/
void PMW3389::upload_firmware() {
  //Write 0 to Rest_En bit of Config2 register to disable Rest mode.
  writeRegister(REG_Config2, 0x00);

  // write 0x1d in SROM_enable reg for initializing
  writeRegister(REG_SROM_Enable, 0x1d);

  // wait for more than one frame period
  delay(10); // assume that the frame rate is as low as 100fps... even if it should never be that low

  // write 0x18 to SROM_enable to start SROM download
  writeRegister(REG_SROM_Enable, 0x18);

  // write the SROM file (=firmware data)
  SPI_BEGIN;
  BEGIN_COM;
  delayMicroseconds(T_NCS_SCLK);
  _spi->transfer(REG_SROM_Load_Burst | WRITE_MASK); // write burst destination adress
  delayMicroseconds(15);

  // send all bytes of the firmware
  for (int i = 0; i < firmware_length; i++) {
    _spi->transfer(firmware_data[i]);
    delayMicroseconds(15);
  }

  delayMicroseconds(T_SCLK_NCS_WRITE);
  END_COM;
  SPI_END;
  delayMicroseconds(T_BEXIT);

  delayMicroseconds(200);
  //Read the SROM_ID register to verify the ID before any other register reads or writes.
  readRegister(REG_SROM_ID);

  //Write 0x00 (rest disable) to Config2 register for wired mouse or 0x20 for wireless mouse design.
  writeRegister(REG_Config2, 0x00);
}

/*
check_signature: check whether SROM is successfully loaded

return: true if the rom is loaded correctly.
*/
bool PMW3389::check_signature() {
  byte pid = readRegister(REG_Product_ID);
  byte iv_pid = readRegister(REG_Inverse_Product_ID);
  byte SROM_ver = readRegister(REG_SROM_ID);

  return (pid==0x47 && iv_pid == 0xB8 && SROM_ver == 0xE8); // 47 b8 e8 71 184 232
}


uint8_t PMW3389::readRegister(uint8_t address)
{
    // // reset readingMotion if another register is read
    // if(readingMotion &&
    //         address != REGISTER_MOTION &&
    //         address != REGISTER_DELTA_X_L &&
    //         address != REGISTER_DELTA_X_H &&
    //         address != REGISTER_DELTA_Y_L &&
    //         address != REGISTER_DELTA_Y_H)
    // {
    //     readingMotion = false;
    // }

    uint16_t result;

    address &= READ_MASK;

    SPI_BEGIN;
    BEGIN_COM;

    // TODO Use T_SRAD between write of address byte and read of data byte
    // TODO try SPI.transfer(address)
    /*result = SPI.transfer16((uint16_t)((address << 8) & 0xFF00));*/
    _spi->transfer(address);
    // Delay (T_SRAD) between write of address byte and read of data byte (see
    // Figure 16 / p.20 and Figure 20 / p.21)
    delayMicroseconds(T_SRAD);
    result = _spi->transfer(0);

    delayMicroseconds(T_SCLK_NCS_READ);
    END_COM;
    SPI_END;

    // TODO Maybe add delay (T_SRR/T_SRW) for next read/write operation (see
    // Figure 20 / p.21)
    // This basic approach might unnecessarily stall execution.
    // Be careful with burst mode being active.
    delayMicroseconds(T_SRW);

    return (uint8_t) (result & 0x00FF);
}

void PMW3389::writeRegister(uint8_t address, uint8_t data)
{
    // if(readingMotion) readingMotion = false;
    address |= WRITE_MASK;

    SPI_BEGIN;
    BEGIN_COM;

    _spi->write16((uint16_t)((address << 8) | data));

    delayMicroseconds(T_SCLK_NCS_WRITE);

    END_COM;
    SPI_END;

    // TODO Maybe add delay (T_SWW/T_SWR) for next read/write operation (see
    // Figure 20 / p.21)
    // This basic approach might unnecessarily stall execution.
    // Be careful with burst mode being active.
    delayMicroseconds(T_SWW);
}

// /*
// prepareImage: prepare a raw image capture from the snesor
// */
// void PMW3389::prepareImage()
// {
//   SPI_BEGIN;

//   adns_write_reg(REG_Config2, 0x00);

//   adns_write_reg(REG_Frame_Capture, 0x83);
//   adns_write_reg(REG_Frame_Capture, 0xc5);

//   delay(20);

//   BEGIN_COM;
//   SPI.transfer(REG_Raw_Data_Burst & 0x7f);
//   delayMicroseconds(20);
// }
// /*
// readImagePixel: prepare a raw image capture from the snesor
// */
// byte PMW3389::readImagePixel()
// {
//   byte pixel = SPI.transfer(0);
//   delayMicroseconds(20);

//   return pixel;
// }

// void PMW3389::endImage()
// {
//   END_COM;
//   SPI_END;
// }
