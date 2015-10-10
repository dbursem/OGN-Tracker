


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

#include <Wire.h>
#include <Adafruit_BMP085.h>  
#include <SPI.h>
#include <RFM69registers.h>
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>
#include <EEPROM.h>
#include <stdint.h>
#include <RunningMedian.h>  //https://github.com/RobTillaart/Arduino/tree/master/libraries/RunningMedian

#include "OGNGPS.h"
#include "OGNRadio.h"
#include "Configuration.h"
#include "OGNPacket.h"
#include "ReceiveQueue.h"


//#define LOCALTEST
#define GPSECHO false //makes Adafruit_GPS echo all NMEA data to Serial. 


void FormRFPacket(OGNPacket *Packet);

OGNRadio *Radio;
Configuration *TrackerConfiguration;
ReceiveQueue *ReceivedData;

uint8_t ReceiveActive = false;

uint32_t ReportTime = 0;
#define REPORTDELAY 1000
SoftwareSerial mySerial(5,4);
OGNGPS GPS(&mySerial);
Adafruit_BMP085 bmp;
void setup() 
{
  TrackerConfiguration = new Configuration();
  TrackerConfiguration->LoadConfiguration();
  
  ReceivedData = new ReceiveQueue();
  
  Serial.begin(115200);
  
  GPS.begin(9600);
  if (!bmp.begin()) {
    GPS.use_bmp=false;
  }
  else {
    GPS.use_bmp=true;
  }
  Radio = new OGNRadio(); 
  Radio->Initialise(TrackerConfiguration->GetTxPower());
  ConfigurationReport();
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}
void loop() 
{
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
    if (GPS.use_bmp)
      GPS.BaroAltitude = bmp.readAltitude();
  }
  OGNPacket *ReportPacket;
  uint32_t TimeNow;
  
  TimeNow = millis();
  
  if( (TimeNow - ReportTime) > REPORTDELAY)
  {
      ReportTime = TimeNow;
    /*
      Serial.print("BMP in use: ");
      Serial.println(use_bmp);
      Serial.print("Alt BMP: ");
      Serial.print(bmp.readAltitude());
      Serial.print(" Alt GPS: ");
      Serial.print(alt_gps);
      Serial.print(" Alt OGNGPS: ");
      Serial.println(GPS.altitude);
      Serial.print("Lon: ");
      Serial.print(GPS.longitude,4);
      Serial.print(" Lat: ");
      Serial.print(GPS.latitude,4);
      Serial.print(" Speed: ");
      Serial.print(GPS.GetOGNSpeed());
      Serial.print(" - ");
      Serial.println(GPS.speed);
      Serial.print("Sats: ");
      Serial.print(GPS.satellites);
      Serial.print(" Fix: ");
      Serial.print(GPS.GetOGNFixQuality());
      Serial.print(" Mode: ");
      Serial.println(GPS.GetOGNFixMode());
      */
#ifdef LOCALTEST
    if(1)
#else
    if(GPS.GetOGNFixQuality())
#endif
    {
      
      if(ReceiveActive)
      {
        Radio->EndReceive();
        ReceiveActive = false;
      }
      
      GPS.CalculateTurnRate(TimeNow);
      GPS.CalculateClimbRate(TimeNow);
 
      ReportPacket = new OGNPacket;
      FormRFPacket(ReportPacket);
        
      Radio->SendPacket(ReportPacket->ManchesterPacket,OGNPACKETSIZE*2,F8684,TrackerConfiguration->GetTxPower());
            
      Radio->StartReceive(F8684, ReportPacket->ManchesterPacket); ReceiveActive = true;
      
      delete ReportPacket;
    }
  }

  if(Serial.available())
  {
    ProcessSerial();
  }
  
  if(ReceiveActive)
  {
    if(Radio->CheckReceive())
    {
      ReportPacket = new OGNPacket;
      Radio->GetReceivePacket(ReportPacket->ManchesterPacket);
      DecodeRFPacket(ReportPacket);
      delete ReportPacket;
    }
  }
}

void FormRFPacket(OGNPacket *Packet)
{
  Packet->MakeSync();
 
  Packet->MakeHeader(false,false,0,false,TrackerConfiguration->GetAddressType(),TrackerConfiguration->GetAddress());
   
  Packet->MakeLatitude(GPS.GetOGNFixQuality(), GPS.seconds, GPS.GetOGNLatitude());
    
  Packet->MakeLongitude(GPS.GetOGNFixMode(), GPS.use_bmp, GPS.GetOGNDOP(), GPS.GetOGNLongitude());
  
  Packet->MakeAltitude(GPS.GetOGNTurnRate(), GPS.GetOGNSpeed(), GPS.GetOGNAltitude());
  
  Packet->MakeHeading(TrackerConfiguration->GetAircraftType(), TrackerConfiguration->GetPrivate(), GPS.GetOGNClimbRate(), GPS.GetOGNHeading());

  Packet->Whiten();
  
  Packet->AddFEC();
  
  Packet->ManchesterEncodePacket();
}

void DecodeRFPacket(OGNPacket *Packet)
{
  Packet->ManchesterDecodePacket();
  if(Packet->CheckFEC()!=0)
  {
#ifdef LOCALTEST
    Serial.print("CRC Error "); Serial.println(Packet->CheckFEC(),DEC);
    Packet->PrintRawPacket();
#endif
    return;
  }
  Packet->DeWhiten();
  ReceivedData->AddPacket((uint32_t *)&Packet->RawPacket[4]);
}

#define PI_OVER_180 0.0174532925

void ProcessReceivedPackets(OGNGPS *GPS)
{
  float TargetLatitude, TargetLongitude;
  float TargetDistance, TargetBearing;
  
  uint32_t ID;
  int32_t Altitude;
  int32_t NorthDist, EastDist;
  uint8_t AcType;
  uint16_t Heading;

  TargetLatitude = ReceivedData->GetLatitude();
  TargetLongitude = ReceivedData->GetLongitude();
  TargetDistance = GPS->distanceBetween(GPS->latitudeDegrees, GPS->longitudeDegrees,TargetLatitude, TargetLongitude);
  TargetBearing = PI_OVER_180 * GPS->courseTo(GPS->latitudeDegrees, GPS->longitudeDegrees,TargetLatitude, TargetLongitude);
  NorthDist = TargetDistance * cos(TargetBearing);
  EastDist = TargetDistance * sin(TargetBearing);
  ID = ReceivedData->GetID();
  AcType = ReceivedData->GetType();
  Altitude = ReceivedData->GetAltitude();
  Altitude = Altitude - GPS->altitude;
  Heading = ReceivedData->GetHeading();
  SendTargetString(NorthDist,EastDist,ID,AcType,Altitude,Heading);
}  
  
void SendTargetString(int32_t North, int32_t East, uint32_t ID, uint8_t AcType, int32_t Altitude, uint16_t Heading)
{
  String NMEAString;
  uint32_t i;
  uint32_t Check = 0;
  
  NMEAString += F("$PFLAA,0,");
  NMEAString += North;
  NMEAString += F(",");
  NMEAString += East;
  NMEAString += F(",");
  NMEAString += Altitude;
  NMEAString += F(",1,");
  NMEAString += String(ID,HEX);
  NMEAString += F(",");
  NMEAString += String(Heading,HEX);
  NMEAString += F("0,0,0,");
  NMEAString += String(AcType,DEC);
  
  for(i=1;i<NMEAString.length();i++)
  {
    Check ^= NMEAString.charAt(i);
  }
  
  NMEAString += F("*");
  if(Check<0x10) NMEAString += F("0");
  NMEAString += String(Check,HEX);
  
  NMEAString.toUpperCase();
  
  Serial.println(NMEAString);
}



void ConfigurationReport(void)
{
    Serial.print(F("Device Address \t")); Serial.println(TrackerConfiguration->GetAddress(),HEX);
    Serial.print(F("Address Type \t"));
    switch(TrackerConfiguration->GetAddressType())
    {
      case ADDRESS_TYPE_RANDOM  : Serial.println(F("Random")); break;
      case ADDRESS_TYPE_ICAO   : Serial.println(F("ICAO")); break; 
      case ADDRESS_TYPE_FLARM  : Serial.println(F("Flarm")); break;
      case ADDRESS_TYPE_OGN   : Serial.println(F("OGN")); break; 
      default : Serial.println();
    }
    
    Serial.print(F("Aircraft Type is\t"));
    switch(TrackerConfiguration->GetAircraftType())
    {
      case AIRCRAFT_TYPE_UNKNOWN  : Serial.println(F("Unknown")); break;
      case AIRCRAFT_TYPE_GLIDER   : Serial.println(F("Glider")); break; 
      case AIRCRAFT_TYPE_TOW_PLANE : Serial.println(F("Tow Plane")); break;
      case AIRCRAFT_TYPE_HELICOPTER : Serial.println(F("Helicopter")); break;
      case AIRCRAFT_TYPE_PARACHUTE : Serial.println(F("Parachute")); break;
      case AIRCRAFT_TYPE_DROP_PLANE : Serial.println(F("Drop Plane")); break;
      case AIRCRAFT_TYPE_HANG_GLIDER : Serial.println(F("Hang Glider")); break;
      case AIRCRAFT_TYPE_PARA_GLIDER : Serial.println(F("Para Glider")); break;
      case AIRCRAFT_TYPE_POWERED_AIRCRAFT : Serial.println(F("Powered Aircraft")); break; 
      case AIRCRAFT_TYPE_JET_AIRCRAFT : Serial.println(F("Jet Aircraft")); break;
      case AIRCRAFT_TYPE_UFO : Serial.println(F("UFO")); break;
      case AIRCRAFT_TYPE_BALLOON : Serial.println(F("Balloon")); break;
      case AIRCRAFT_TYPE_AIRSHIP : Serial.println(F("Airship")); break;
      case AIRCRAFT_TYPE_UAV : Serial.println(F("UAV")); break;
      case AIRCRAFT_TYPE_STATIC_OBJECT : Serial.println(F("Static")); break;
      default : Serial.println();
    }
    
    Serial.print(F("Serial Baud Rate \t")); Serial.println(TrackerConfiguration->GetSerialBaud());
    Serial.print(F("GPS Type is \t")); Serial.println(F("NMEA"));
    Serial.print(F("GPS Baud Rate \t")); Serial.println(TrackerConfiguration->GetGPSBaud(),DEC);
    Serial.print(F("GSP RX pin:"));  Serial.println(TrackerConfiguration->GetDataInPin(),DEC);
    Serial.print(F("GPS TX pin: "));  Serial.println(TrackerConfiguration->GetDataOutPin(),DEC);
    Serial.print(F("Privacy is "));  if(TrackerConfiguration->GetPrivate()) Serial.println(F("On")); else Serial.println(F("Off"));
    Serial.print(F("OutputPower ")); Serial.print(TrackerConfiguration->GetTxPower()); Serial.println(F(" dBm"));
    Serial.print(F("NMEA Out is "));  
    if(TrackerConfiguration->GetNMEAOut())
    {
      Serial.print(F("On. Delay is "));Serial.println(TrackerConfiguration->GetNMEADelay());
    } 
    else 
    {
      Serial.println(F("Off"));
    }
    
    Serial.println();
}

void StatusReport(void)
{ 
  Serial.print(F("Tracking "));Serial.print(GPS.satellites); Serial.println(F(" satellites"));

}

#define MAXLENGTH 40
void ProcessSerial(void)
{
  static String Buffer = "";
  int8_t Byte;
  uint32_t Address;
  uint8_t Type;
  uint8_t Power;
  
  while (Serial.available())
  {
    if(Buffer.length() < MAXLENGTH)
    {
      Byte = toupper(Serial.read());
      
      if ((Byte == '\r' ) || (Byte == '\n' ))
      {
        Serial.println(Buffer);
        if(Buffer.startsWith("STATUS"))
        {
          StatusReport();
        }
        else if(Buffer.startsWith("CONFIG"))
        {
          ConfigurationReport();
        }         
        else if(Buffer.startsWith("ADDRESS "))
        {
          Buffer.remove(0,8);
          Address = strtol(Buffer.c_str(),NULL,16);
          if(Address)
            TrackerConfiguration->SetAddress(Address);
          ConfigurationReport();
        }         
        else if(Buffer.startsWith("SAVE"))
        {
          Serial.println(F("Saving Config"));
          TrackerConfiguration->WriteConfiguration();
          TrackerConfiguration->LoadConfiguration();
          Serial.print(F("\r\nSaved Configuration\r\n"));
          ConfigurationReport();          
        }         
        else if(Buffer.startsWith("ADDRESSTYPE "))
        {
          Buffer.remove(0,12);
          Type = (uint8_t)Buffer.toInt();
          TrackerConfiguration->SetAddressType(Type);
          ConfigurationReport();
        }         
        else if(Buffer.startsWith("AIRCRAFTTYPE "))
        {
          Buffer.remove(0,13);
          Type = (uint8_t)Buffer.toInt();
          TrackerConfiguration->SetAircraftType(Type);
          ConfigurationReport();
        }
        else if(Buffer.startsWith("POWER "))
        {
          Buffer.remove(0,6);
          Power = (uint8_t)Buffer.toInt();
          TrackerConfiguration->SetTxPower(Power);
          ConfigurationReport();
        }         

        Buffer = "";
      }
      else
      {
         Buffer = Buffer + (char)Byte;
      }
    }
    else
    {
      Buffer = "";
    }  
  }
} 



