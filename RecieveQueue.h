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

#ifndef RECIEVEQUEUE_h
#define RECIEVEQUEUE_h

#include <stdint.h>

#define QUEUESIZE 5
#define PACKETSIZE 5

typedef uint32_t PACKET[PACKETSIZE] ;

/////////////////////////////////////////////////////////////////////
class RecieveQueue 
{
  public:
    RecieveQueue(void);
    void AddPacket(uint32_t *Data);
    uint8_t Available(void);
    void RemovePacket(void);
  protected:
				
  private:
    uint8_t ReadPtr,WritePtr;
    PACKET Packet[QUEUESIZE];        
};


#endif 

