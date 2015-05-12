/* 
    OGN Tracker Client
    Copyright (C) <2015>  <Mike Roberts>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "OGNRadio.h"

#define SCK 13
#define CS  10
#define MISO 12
#define MOSI 11

SPISettings SPISet(SPI_CLOCK_DIV128, MSBFIRST, SPI_MODE0);

OGNRadio::OGNRadio(void)
{
  pinMode(SCK, OUTPUT); digitalWrite(SCK, HIGH);
  pinMode(MOSI, OUTPUT); digitalWrite(MOSI,HIGH);
  pinMode(CS,OUTPUT); digitalWrite(CS,HIGH);
  pinMode(MISO, INPUT);

  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  SPI.setBitOrder(MSBFIRST);  
}

void OGNRadio::Initialise(void)
{
  WriteRegister(REG_OPMODE, RF_DATAMODUL_MODULATIONTYPE_FSK  | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY);
  WriteRegister(REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET  | RF_DATAMODUL_MODULATIONSHAPING_10 | RF_DATAMODUL_MODULATIONTYPE_FSK );
  
  WriteRegister(REG_IRQFLAGS1,0x00); 
  WriteRegister(REG_IRQFLAGS2,0x00);
  
  WriteRegister(REG_BITRATEMSB, RF_BITRATEMSB_100000);    
  WriteRegister(REG_BITRATELSB, RF_BITRATELSB_100000);    
  
  WriteRegister(REG_FDEVMSB,RF_FDEVMSB_50000);   
  WriteRegister(REG_FDEVLSB,RF_FDEVLSB_50000);  
 
  // Frequenct 868.2Hhz  FRF = 14224589 = D90CCD
  WriteRegister(REG_FRFMSB, 0xD9);
  WriteRegister(REG_FRFMID, 0x0C);
  WriteRegister(REG_FRFLSB, 0xCD);

  // Frequenct 868.4Hhz  FRF = 14227866 = D9199A
  //WriteRegister(REG_FRFMSB, 0xD9);
  //WriteRegister(REG_FRFMID, 0x19);
  //WriteRegister(REG_FRFLSB, 0x9A);
  
  WriteRegister(REG_PACKETCONFIG1, RF_PACKET1_FORMAT_FIXED | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_OFF | RF_PACKET1_CRCAUTOCLEAR_ON |  RF_PACKET1_ADRSFILTERING_OFF);
  
  WriteRegister(REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | 63);
  WriteRegister(REG_PAYLOADLENGTH, 64);
  WriteRegister(REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_1BIT | RF_PACKET2_AUTORXRESTART_ON  | RF_PACKET2_AES_OFF);
  
  WriteRegister(REG_AUTOMODES, 0);
  
  WriteRegister(REG_PALEVEL, RF_PALEVEL_PA0_OFF | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON | RF_PALEVEL_OUTPUTPOWER_11111);
  WriteRegister(REG_OCP, RF_OCP_OFF);
  WriteRegister(REG_TESTPA1,0x5D);
  WriteRegister(REG_TESTPA2,0x7C);
}


void OGNRadio::SendPacket(uint8_t *Packet, uint16_t Size, uint16_t freq)
{
  uint8_t i;
  uint32_t s,e;
  
  if(freq == F8682)
  {
    // Frequenct 868.2Hhz  FRF = 14224589 = D90CCD
    WriteRegister(REG_FRFMSB, 0xD9);
    WriteRegister(REG_FRFMID, 0x0C);
    WriteRegister(REG_FRFLSB, 0xCD);
  }
  else
  {
    // Frequenct 868.4Hhz  FRF = 14227866 = D9199A
    WriteRegister(REG_FRFMSB, 0xD9);
    WriteRegister(REG_FRFMID, 0x19);
    WriteRegister(REG_FRFLSB, 0x9A);
  }  

  WriteRegister(REG_PREAMBLEMSB, 0x00);
  WriteRegister(REG_PREAMBLELSB, 0x80);
  
  WriteRegister(REG_IRQFLAGS1,0x00); 
  WriteRegister(REG_IRQFLAGS2,0x00);
  for(i=0;i<Size;i++)
  {
    WriteRegister(REG_FIFO,Packet[i]);
  }
  
  s=micros();
  WriteRegister(REG_OPMODE , RF_DATAMODUL_MODULATIONTYPE_FSK  | RF_OPMODE_LISTEN_OFF | RF_OPMODE_TRANSMITTER);
  
  do {} while( (ReadRegister(REG_IRQFLAGS2) & RF_IRQFLAGS2_PACKETSENT ) != RF_IRQFLAGS2_PACKETSENT);

  e=micros();
  WriteRegister(REG_OPMODE, RF_DATAMODUL_MODULATIONTYPE_FSK  | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY);

}


void OGNRadio::WriteRegister(uint8_t Register, uint8_t Data)
{
  SPI.beginTransaction(SPISet);
  digitalWrite(CS,LOW);
  SPI.transfer(Register | 0x80);
  SPI.transfer(Data);
  digitalWrite(CS,HIGH);
  SPI.endTransaction();
}

uint8_t OGNRadio::ReadRegister(uint8_t Register)
{
  uint8_t result;
  
  SPI.beginTransaction(SPISet);
  digitalWrite(CS,LOW);
  SPI.transfer(Register & 0x7F);
  result = SPI.transfer(0);
  digitalWrite(CS,HIGH);
  SPI.endTransaction();
  return result;
}


