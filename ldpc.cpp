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

#include "ldpc.h"

#include <avr/pgmspace.h>
// every row represents the generator for a parity bit
static const uint32_t LDPC_ParityGen[48][5]
PROGMEM
=   // Parity bits generator: 48 vectors to generate 48 parity bits
{
    // Each vector applied to the user data yields a corresponding parity bit
    { 0x40A90281, 0x9159D249, 0xCE9D516B, 0x2FDEED0B, 0xD9267CD4  },
    { 0xCCBC0FC3, 0xCC4FA4BC, 0x811EC3D0, 0xB07EC1B3, 0xA3B8E8D8  },
    { 0x66418D56, 0x3B85ADFF, 0xD2A6532E, 0x48CF52E4, 0x6A16586D  },
    { 0x44C71240, 0x2C94631F, 0x15F15A4A, 0x7459D901, 0x037863CC  },
    { 0x7386D718, 0x7F6C9623, 0x738E2E0C, 0xD2351593, 0xEF358669  },
    { 0x7BF87232, 0x9E4CCD68, 0xBB82590E, 0x9C9292EA, 0x4CE2AEB9  },
    { 0xAA8436BC, 0x94A61C4D, 0x1DA89B11, 0x72EAF204, 0x34D3A041  },
    { 0xBCE77760, 0x229935B2, 0xAAF85CE3, 0xFFE7B602, 0xF26BCC64  },
    { 0xD0C371D0, 0xA553D12F, 0xA0685BF2, 0x5C553C81, 0x0218EB48  },
    { 0x8D29034D, 0xEB20A394, 0x1A8C82A3, 0x41B4DA0C, 0x8632F81E  },
    { 0x15A50876, 0x9BC10F59, 0xF979D1E0, 0xCFF6BD88, 0x88FE5895  },
    { 0x037A9ED5, 0xFA5DB837, 0x61395ACA, 0xE65B5839, 0x9A2D9D02  },
    { 0xEE70D18F, 0x8AE909C6, 0x8AF5BECA, 0x66968559, 0x1BD9B5E7  },
    { 0xC56397AC, 0xD8FF6A30, 0x8E165AF3, 0xC01686B9, 0xEC26BEDC  },
    { 0xB2D63859, 0xFACA8CFD, 0x7EE85EB7, 0x19BFDA46, 0xC8C1CA52  },
    { 0xD7EB4D94, 0x426104DA, 0x124FBD54, 0xBF610A1D, 0x0E615094  },
    { 0x68EFE180, 0x15A1549C, 0x18D20289, 0xBD28AD44, 0x8DADDAEC  },
    { 0xD7EB4D18, 0x426548DA, 0x12CDFD50, 0xBF61081D, 0x0E615094  },
    { 0xC92E426E, 0x648641B5, 0xC16A07B9, 0xA52D48AC, 0x842364AB  },
    { 0xB71DAB61, 0x2B15995C, 0x6BE7E0C1, 0x97ECE351, 0xDF622A04  },
    { 0x55CD7406, 0x5E0F3507, 0x23F6C372, 0x7ECAFE84, 0x7E68A8DF  },
    { 0x97DB831C, 0xD46D648F, 0x14FA22B3, 0x4F875648, 0x94C23936  },
    { 0x60D940EC, 0xFCC18797, 0xD0DE7383, 0xF38F22E5, 0x2E7A733E  },
    { 0xD8C22D55, 0x8D45EB4E, 0xAC695FF3, 0xDED59211, 0x8851288A  },
    { 0xCE9D11A1, 0xD8E8F438, 0xAF3102EE, 0xCB2FE547, 0xC11845BD  },
    { 0x61D940EC, 0xACC18F17, 0xD0DE7311, 0xE7CF22E5, 0x2E7A6B7E  },
    { 0x66AD8025, 0x493D883C, 0x538E9261, 0x5F0E116B, 0xB17492FA  },
    { 0x747C4C9E, 0x3780804E, 0x29A7B2F1, 0x2838DF6D, 0xA68C11EB  },
    { 0x7E33E90F, 0x2FB3D8E9, 0x2A9DE538, 0x3AC1ABDC, 0x59C14EAF  },
    { 0x16B6095E, 0x4883D57E, 0xF765FF4B, 0x431C6EF3, 0xF2C45F6C  },
    { 0x3F04D4F4, 0xEEA73108, 0x567ECF38, 0x15200560, 0x56AB6942  },
    { 0x1E5ECFFE, 0x29426F53, 0x17057060, 0xA774ED7F, 0x4FE7EACB  },
    { 0xF9F02A12, 0xFADEBEE2, 0xBE67EB8B, 0x5506F594, 0xC5037599  },
    { 0x7BF87632, 0x960CC528, 0xBB825D0E, 0x9C929A7A, 0x4CE22FBB  },
    { 0x6E2BE90F, 0x2FB3DAED, 0x2A9DCD38, 0x1A81A3DC, 0x59C14EAF  },
    { 0x16B6A97A, 0x4883D57E, 0xF765FB49, 0x4BBC7EF3, 0xF2C41F7C  },
    { 0x260C82A4, 0xD8645AF5, 0x9913D30A, 0x7158DC67, 0x68526E0A  },
    { 0x5DAD3406, 0x5E1F6607, 0xF7F2D35A, 0x5ACAFEA4, 0x3E48A8D7  },
    { 0xAF04D4E4, 0x67232129, 0x567ECE20, 0x1D280560, 0x56A96942  },
    { 0xBA8536B8, 0x94AE1C4D, 0x5EA81B11, 0x62EAF604, 0xB4D3A141  },
    { 0xDF522858, 0x25CE2C46, 0x6C1E93BA, 0xDB9FBF4E, 0x9F8AACF9  },
    { 0xC92E4E6B, 0x6CD659D5, 0xCD6A03B8, 0x872D423C, 0x0623AF69  },
    { 0xD8C20E55, 0x8945EB4E, 0x0C615FF3, 0xFED5D211, 0x8853A88A  },
    { 0x11EC2FBB, 0xE98188FA, 0x6D02584A, 0x7BF87EBD, 0x0C324421  },
    { 0xE1AB0619, 0x62864C22, 0xBC029B88, 0xDC501DA2, 0x3DB63518  },
    { 0x85657508, 0xE76CE85B, 0x35A012AB, 0xD7719D8D, 0xC1CE9294  },
    { 0x7E33EB0F, 0x2FB3D9F9, 0x2E9DE438, 0x3AC1ABDC, 0x71814AAF  },
    { 0x55CD3406, 0x5E1F7407, 0x63F2D35A, 0x5ACAFEA4, 0x7E48A8DF  }
} ;

void LDPC::LDPC_Encode(const uint32_t *Data, uint32_t *Parity, const uint32_t ParityGen[48][5])
{ 
     uint8_t ParIdx=0;
    Parity[ParIdx]=0;
    uint32_t Mask=1;
    for(uint8_t Row=0; Row<48; Row++)
    {
        uint8_t Count=0;
        const uint32_t *Gen=ParityGen[Row];
        for(uint8_t Idx=0; Idx<5; Idx++)
        {
            Count+=u32Count1s(Data[Idx] & pgm_read_dword(Gen+Idx));
        }
        if(Count&1) Parity[ParIdx]|=Mask;
        Mask<<=1;
        if(Mask==0)
        {
            ParIdx++;
            Parity[ParIdx]=0;
            Mask=1;
        }
    }
}


void LDPC::LDPC_EncodeBlock(const uint32_t *Data, uint32_t *Parity)
{
    LDPC_Encode(Data, Parity, LDPC_ParityGen);
}


LDPC::LDPC(void)
{
}


uint8_t LDPC::u8Count1s(uint8_t Byte)
{
  uint8_t Count = 0;
  uint8_t i;
  
  for(i=0;i<8;i++)
  {
    if( (Byte & 0x01) == 0x01)
     Count ++;
    Byte = Byte>>1;
  }
  return Count;
}
    
   
uint8_t LDPC::u32Count1s(uint32_t uWord)
{
  uint8_t count = 0;
  uint8_t i;
  for(i=0;i<4;i++)
  {
    count += u8Count1s( uWord & 0x000000FF);
    uWord = uWord >> 8;
  }
  return count;
}
