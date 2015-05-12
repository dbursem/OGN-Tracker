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

#include "Configuration.h"
#include "NVStore.h"
#include <arduino.h>

Configuration::Configuration(void)
{
}

void Configuration::LoadConfiguration(uint32_t TempAddress)
{
  NVStore *Store;
  Configuration_Struct temp;
  
  Store = new NVStore();
  Store->Store_Read((void *)&temp, sizeof(temp));
  delete Store;
  
  if(temp.StoreVersion != 0x0001)
  {
    cs.Address = TempAddress;
    cs.AddressType = ADDRESS_TYPE_OGN ;
    cs.AircraftType = AIRCRAFT_TYPE_UNKNOWN;
    cs.Private = 0;
    cs.SerialBaud = 115200;
    cs.GPSBaud = 9600;
    cs.DataInPin = 4;
    cs.DataOutPin = 5;
  } 
}

uint32_t Configuration::GetAddress(void)
{
  return cs.Address;
}

void Configuration::SetAddress(uint32_t Address)
{
  cs.Address = Address;
}

uint8_t Configuration::GetAddressType(void)
{
  return cs.AddressType;
}

void Configuration::SetAddressType(uint8_t Type)
{
  cs.AddressType = Type;
}

uint8_t Configuration::GetAircraftType(void)
{
  return cs.AircraftType;
}

void Configuration::SetAircraftType(uint8_t Type)
{
  cs.AircraftType = Type;
}

uint8_t Configuration::GetPrivate(void)
{
  return cs.Private;
}

void Configuration::SetPrivate(uint8_t Private)
{
  cs.Private = Private;
}

uint8_t Configuration::GetDataInPin(void)
{
  return cs.DataInPin;
}

void Configuration::SetDataInPin(uint8_t Pin)
{
  cs.DataInPin = Pin;
}

uint8_t Configuration::GetDataOutPin(void)
{
  return cs.DataOutPin;
}

void Configuration::SetDataOutPin(uint8_t Pin)
{
  cs.DataOutPin = Pin;
}


void Configuration::WriteConfiguration(void)
{
  //NONVolatile_Write((void *)&cs,sizeof(cs));
}

uint32_t Configuration::GetSerialBaud(void)
{
  return cs.SerialBaud;
}

void Configuration::SetSerialBaud(uint32_t SerialBaud)
{
  cs.SerialBaud = SerialBaud;
}

uint32_t Configuration::GetGPSBaud(void)
{
  return cs.GPSBaud;
}

void Configuration::SetGPSBaud(uint32_t GPSBaud)
{
  cs.GPSBaud = GPSBaud;
}



