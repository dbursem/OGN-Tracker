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

#include "RecieveQueue.h"

#include <arduino.h>

/* OGN Packet Map
[00:03] [SSSS SSSS] [SSSS SSSS] [SSSS SSSS] [SSSS SSSS]  Sync Field 4 Bytes 0x0AF3656C
[04:07] [ECRR PMTT] [AAAA AAAA] [AAAA AAAA] [AAAA AAAA]  [E]mergency, [C]rypt, [R]elay Count, Even [P]arity, Address [T]ype [A]ddress 32 Bits
[08:0B] [QQTT TTTT] [LLLL LLLL] [LLLL LLLL] [LLLL LLLL]  [Q]uality, [T}ime, [L]atitude 
[0C:0F] [MBDD DDDD] [LLLL LLLL] [LLLL LLLL] [LLLL LLLL]  [F]ix, [B]aro, [D]OP, [L]ongitude
[10:13] [RRRR RRRR] [SSSS SSSS] [SSAA AAAA] [AAAA AAAA]  Turn [R]ate, [S]peed, [A]ltitude
[14:17] [TTTT TTTT] [AAAA PCCC] [CCCC CCDD] [DDDD DDDD]  [T]emperature, [A]ircraft Type, [P]rivate [C]limb, Hea[D]ing
[18:2B] [CCCC CCCC] [CCCC CCCC] [CCCC CCCC] [CCCC CCCC]  FEC Codes
[2C:2F] [CCCC CCCC] [CCCC CCCC] [CCCC CCCC] [CCCC CCCC]
*/

RecieveQueue::RecieveQueue(void)
{
    ReadPtr = 0; WritePtr = 0;
}

void RecieveQueue::AddPacket(uint32_t *Data)
{
  if( ((WritePtr+1)%QUEUESIZE) == ReadPtr )
  {
    ReadPtr ++; ReadPtr = ReadPtr % QUEUESIZE;
  }
  memcpy(Packet[WritePtr], Data, 5*sizeof(uint32_t));
  WritePtr ++; WritePtr = WritePtr % QUEUESIZE;
  //Serial.println(WritePtr); Serial.println(ReadPtr);
} 

uint8_t RecieveQueue::Available(void)
{
  if(ReadPtr == WritePtr)
    return 0;
  else
    return 1;
}

void RecieveQueue::RemovePacket(void)
{
  if(ReadPtr != WritePtr)
  {
    ReadPtr ++;  ReadPtr = ReadPtr % QUEUESIZE;
  }
}

