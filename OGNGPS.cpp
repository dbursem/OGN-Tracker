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
 
#include "OGNGPS.h"



OGNGPS::OGNGPS(SoftwareSerial *ser) : Adafruit_GPS(ser)
{
  TurnRate = 0;
  ClimbRate = 0;
  LastAltitude = 0;
  LastHeading = 0;
}
void OGNGPS::startInterrupt() {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    
}

void OGNGPS::CalculateClimbRate(int32_t TimeNow)
{
  static uint32_t LastTime = 0;
  int32_t NewAltitude;
  int32_t DeltaH, DeltaT;
  
  if(( TimeNow - LastTime) < 30000)
  {
     return;
  }
 
  DeltaT = TimeNow - LastTime;
  LastTime = TimeNow; 
  
  NewAltitude = (int32_t)altitude;
  DeltaH = 10*(NewAltitude - LastAltitude);
  LastAltitude = NewAltitude;
  ClimbRate = DeltaH/DeltaT;
}
  
void OGNGPS::CalculateTurnRate(int32_t TimeNow)
{
  static uint32_t LastTime = 0;
  int32_t NewHeading;
  int32_t DeltaH, DeltaT;
 
  if(( TimeNow - LastTime) < 1000)
  {
     return;
  }
 
  DeltaT = TimeNow - LastTime;
  LastTime = TimeNow; 
  
  NewHeading = (int32_t) angle * 1024 / 360;
  DeltaH = 10*(NewHeading - LastHeading);
  LastHeading = NewHeading;
  TurnRate = DeltaH/DeltaT;
}


uint32_t OGNGPS::GetOGNLatitude(void)
{
  uint32_t Latitude;
  
  Latitude = latitudeDegrees;
  Latitude *= 100000;
  Latitude *= 6;
  Latitude /= 8;
  return Latitude;
}

uint32_t OGNGPS::GetOGNLongitude(void)
{
  uint32_t Longitude;
  Longitude = longitudeDegrees;
  Longitude *= 100000;
  Longitude *= 6;
  Longitude /= 16;
  return Longitude;
}

uint32_t OGNGPS::GetOGNAltitude(void)
{
  uint32_t Altitude;
  Altitude = altitude;
  
  if(Altitude <0)
    return 1;
  else if (Altitude < 0x1000)
    return Altitude;
  else if (Altitude < 0x3000)
    return (0x1000 + ((Altitude - 0x1000)/2));
  else if (Altitude < 0x7000)
    return (0x2000 + ((Altitude - 0x3000)/4));
  else if (Altitude < 0xF000)
    return (0x3000 + ((Altitude - 0x7000)/8));
  else return 0x3FFF;  
}

uint32_t OGNGPS::GetOGNSpeed(void)
{
  uint32_t Speed;
  Speed = speed * 5; // OGN speed in 0.2 knots/s 
  
  
  if(Speed <0)
    return 0;
  else if (Speed < 0x100)
    return Speed;
  else if (Speed < 0x300)
    return (0x100 + ((Speed - 0x100)/2));
  else if (Speed < 0x700)
    return (0x300 + ((Speed - 0x300)/4));
  else if (Speed < 0xF00)
    return (0x700 + ((Speed - 0x700)/8));
  else
    return 0x3FF;
}

uint32_t OGNGPS::GetOGNDOP(void)
{
   return(HDOP);
}

uint8_t OGNGPS::GetOGNFixQuality(void)
{
  if(fix)
   return 1;
  else
    return 0;
}
  
uint8_t OGNGPS::GetOGNFixMode(void)
{
  if(satellites>4)
   return 1;
  else
    return 0;
}
    
uint16_t OGNGPS::GetOGNHeading(void)
{
  return angle * 1024 / 360;
}
 
int16_t OGNGPS::GetOGNTurnRate(void)
{
  return TurnRate;
}
    
int16_t OGNGPS::GetOGNClimbRate(void)        
{
  int16_t Rate = 0;
  int16_t UpDown = 0;
  
  Rate = ClimbRate;
  if(Rate < 0)
  {
    Rate = Rate * -1;
    UpDown = 0x100;
  }
  if(Rate < 0x040)
    Rate = Rate;
  else if (Rate < 0x0C0)
    Rate = 0x040 + (Rate - 0x40)/2;
  else if (Rate < 0x1C0)
    Rate = 0x0C0 + (Rate - 0x0C0)/4;
  else if (Rate < 0x3C0)
     Rate = 0x1C0 + (Rate - 0x1C0)/8;
  else
     Rate = 0x0FF;
     
  return (uint16_t)(UpDown | Rate);
}
