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
#include <RunningMedian.h>
RunningMedian tenseconds = RunningMedian(20);
RunningMedian tenminutes = RunningMedian(120);

OGNGPS::OGNGPS(SoftwareSerial *ser) : Adafruit_GPS(ser)
{
  TurnRate = 0;
  ClimbRate = 0;
  LastAltitude = 0;
  LastHeading = 0;
  startInterrupt();
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
  
  NewAltitude = (int32_t) CorrectedBMPAltitude();
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
  
  NewHeading = (int32_t) angle * 1024 / 360;  // angle * 1024 / 360 following OGN tracking protocol 
  DeltaH = 10*(NewHeading - LastHeading);
  LastHeading = NewHeading;
  TurnRate = DeltaH/DeltaT;
}


uint32_t OGNGPS::GetOGNLatitude(void)
{
  uint32_t Latitude;
  
  Latitude = (latitudeDegrees * 600000);
  Latitude>>=3; // Lat * 100000 * 6 / 8 following OGN tracking protocol 
  return Latitude;
}

uint32_t OGNGPS::GetOGNLongitude(void)
{
  uint32_t Longitude;
  Longitude = (longitudeDegrees * 600000);
  Longitude>>=4; // Lon * 100000 * 6 / 16 following OGN tracking protocol 
  return Longitude;
}
float OGNGPS::CorrectedBMPAltitude(void)
{
  if (!use_bmp) {
    return (altitude < 0 ? 0 : altitude);
  }
  
  int current_correction = 10 * (altitude - BaroAltitude) ;
  tenseconds.add(current_correction);
  samples++;
  if (samples == 20){
    tenminutes.add(tenseconds.getAverage());
    samples = 0;
  }
  float CorrectionFactor = tenminutes.getAverage() /10;
  
  Serial.print(use_bmp);
  Serial.print(" GPSAlt: ");
  Serial.print(altitude,2);
  Serial.print(" BaroAlt: ");
  Serial.print(BaroAltitude,2);
  Serial.print(" Corrected alt:");
  Serial.print(BaroAltitude + CorrectionFactor);
  Serial.print(" Correction factor:");
  Serial.println(CorrectionFactor);
  return (BaroAltitude + CorrectionFactor < 0 ? 0 : BaroAltitude + CorrectionFactor);
}
uint32_t OGNGPS::GetOGNAltitude(void)
{
  uint32_t Altitude = CorrectedBMPAltitude();
  
  if (Altitude < 0x1000)
    return Altitude;
  else if (Altitude < 0x3000)
    return (0x1000 + ((Altitude - 0x1000)>>1));
  else if (Altitude < 0x7000)
    return (0x2000 + ((Altitude - 0x3000)>>2));
  else if (Altitude < 0xF000)
    return (0x3000 + ((Altitude - 0x7000)>>3));
  else return 0x3FFF;  
}

uint32_t OGNGPS::GetOGNSpeed(void)
{
  uint32_t Speed;
  Speed = (speed<0?0:speed) * 5;    // OGN speed in 0.2 knots
  Speed /= 10;          // I have truly no idea where this is comming from...
  
  if (Speed < 0x100)
    return Speed;
  else if (Speed < 0x300)
    return (0x100 + ((Speed - 0x100)>>1));
  else if (Speed < 0x700)
    return (0x300 + ((Speed - 0x300)>>2));
  else if (Speed < 0xF00)
    return (0x700 + ((Speed - 0x700)>>3));
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
  return angle *1024 / 360;
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
    Rate = 0x040 + (Rate - 0x40)>>1;
  else if (Rate < 0x1C0)
    Rate = 0x0C0 + (Rate - 0x0C0)>>2;
  else if (Rate < 0x3C0)
     Rate = 0x1C0 + (Rate - 0x1C0)>>3;
  else
     Rate = 0x0FF;
     
  return (uint16_t)(UpDown | Rate);
}
/*static*/
double OGNGPS::distanceBetween(double lat1, double long1, double lat2, double long2)
{
  // returns distance in meters between two positions, both specified
  // as signed decimal-degrees latitude and longitude. Uses great-circle
  // distance computation for hypothetical sphere of radius 6372795 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers
  double delta = radians(long1-long2);
  double sdlong = sin(delta);
  double cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double slat1 = sin(lat1);
  double clat1 = cos(lat1);
  double slat2 = sin(lat2);
  double clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  return delta * 6372795;
}


double OGNGPS::courseTo(double lat1, double long1, double lat2, double long2)
{
  // returns course in degrees (North=0, West=270) from position 1 to position 2,
  // both specified as signed decimal-degrees latitude and longitude.
  // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
  // Courtesy of Maarten Lamers
  double dlon = radians(long2-long1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double a1 = sin(dlon) * cos(lat2);
  double a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  return degrees(a2);
}
