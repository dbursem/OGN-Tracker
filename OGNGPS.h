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
 
#ifndef OGNGPS_h
#define OGNGPS_h

#include <stdint.h>

#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>

class OGNGPS : public Adafruit_GPS
{
  public:
    OGNGPS(SoftwareSerial *ser);
    void OGNparse();
    uint8_t ProcessInput(void);
    uint32_t GetOGNLatitude();
    uint32_t GetOGNLongitude();
    uint32_t GetOGNAltitude();
    uint32_t GetOGNSpeed();
    uint32_t GetOGNDOP();
    uint8_t  GetOGNFixQuality();
    uint8_t  GetOGNFixMode();
    uint16_t GetOGNHeading();
    int16_t GetOGNClimbRate();
    int16_t GetOGNTurnRate();
    void CalculateClimbRate(int32_t DeltaT);
    void CalculateTurnRate(int32_t DeltaT);
    static double distanceBetween(double lat1, double long1, double lat2, double long2);
    static double courseTo(double lat1, double long1, double lat2, double long2);
    float BaroAltitude;
    boolean use_bmp = false;
    void startInterrupt();
  protected:
				
  private:
    float CorrectedBMPAltitude();
    int16_t LastHeading;
    int32_t LastAltitude;
    uint32_t TurnRate;
    uint32_t ClimbRate;
    int CorrectionFactor;
    int num_iterations;
    
};


#endif 

