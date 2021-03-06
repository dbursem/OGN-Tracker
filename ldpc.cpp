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
static const uint32_t LDPC_ParityCheck[48][7]
PROGMEM
= 
{ // parity check vectors: 48 vectors for 48 parity checks
    // Eaech vector applied to the data packet should yield even number of bits
 { 0x00000805, 0x00000020, 0x04000000, 0x20000000, 0x00000040, 0x00044020, 0x00000000 },
 { 0x00000001, 0x00800800, 0x00000000, 0x00000000, 0x00000000, 0x10010000, 0x00008C98 },
 { 0x00004001, 0x01000080, 0x80000400, 0x00000000, 0x08000200, 0x00200000, 0x00000005 },
 { 0x00000101, 0x20000200, 0x00000022, 0x00000000, 0x00000000, 0xCC008000, 0x00005002 },
 { 0x00000401, 0x00000000, 0x00004900, 0x00000020, 0x00000000, 0x20C00349, 0x00000020 },
 { 0x03140001, 0x00000002, 0x00000000, 0x40000001, 0x41534100, 0x00102C00, 0x00002000 },
 { 0x04008800, 0x82000642, 0x00000000, 0x00000020, 0x88040020, 0x03000010, 0x00000400 },
 { 0x00000802, 0x20000000, 0x02000014, 0x01200000, 0x04000403, 0x00800004, 0x0000A004 },
 { 0x02020820, 0x00000000, 0x80020820, 0x10190040, 0x30000000, 0x00000002, 0x00000900 },
 { 0x40804950, 0x00090000, 0x00000000, 0x00021204, 0x40001000, 0x10001100, 0x00000000 },
 { 0x08000A00, 0x00020008, 0x00040000, 0x02400010, 0x01002000, 0x40280280, 0x00000010 },
 { 0x00000000, 0x00008010, 0x118000A0, 0x00040080, 0x01000084, 0x00040100, 0x00000444 },
 { 0x20040108, 0x18000000, 0x08608800, 0x0000000A, 0x08000010, 0x00040080, 0x00008000 },
 { 0x00004080, 0x00422201, 0x00010000, 0x0000A400, 0x00400800, 0x00840000, 0x00000800 },
 { 0x00000000, 0x60200000, 0x80100240, 0x08000021, 0x02800000, 0x100C0000, 0x00000000 },
 { 0x00001000, 0x01010002, 0x00082001, 0x04000000, 0x00000001, 0x00040002, 0x00004030 },
 { 0x00002300, 0x04000000, 0xA0080000, 0x20004000, 0x00028000, 0x00800000, 0x00000400 },
 { 0x00004000, 0x00104100, 0x40041028, 0x24000020, 0x00200000, 0x00100000, 0x00008000 },
 { 0x08011000, 0x20040000, 0x00000000, 0xA0800000, 0x08090000, 0x00000100, 0x00000A00 },
 { 0x10180000, 0x00000204, 0x00002800, 0x20400800, 0x00000000, 0x10000000, 0x00000004 },
 { 0x00000000, 0xC0000000, 0x10200000, 0x20028000, 0x20000000, 0x80000008, 0x00002011 },
 { 0x82004000, 0x20000000, 0x04202000, 0x00000000, 0x00000000, 0x00020200, 0x00000400 },
 { 0x08600000, 0x00001200, 0x94000000, 0x00000000, 0x40000008, 0x00000000, 0x00008020 },
 { 0x04040000, 0x04010000, 0x04100000, 0x00000100, 0x00200000, 0x40000008, 0x00000804 },
 { 0x00000200, 0x00000110, 0x04000100, 0x00000000, 0x28400400, 0x10000000, 0x00004000 },
 { 0x00080000, 0x00000080, 0x04001000, 0x01882007, 0x00008024, 0x04000001, 0x00000010 },
 { 0x20200000, 0x00000020, 0x00010040, 0x81000800, 0x10001000, 0x00300008, 0x00004400 },
 { 0x90000010, 0x89841021, 0x00000118, 0x08080000, 0x00020000, 0x40000000, 0x00000040 },
 { 0x04C20000, 0x10404034, 0x00000000, 0x00004000, 0x00810001, 0x04000200, 0x00000009 },
 { 0x40102000, 0x020020A0, 0x40100000, 0x00100080, 0x00080400, 0x80030080, 0x00000020 },
 { 0x00010000, 0x04020920, 0x00000200, 0x00060000, 0x00000218, 0x01002007, 0x00001000 },
 { 0x00020008, 0x00A08040, 0x00080000, 0x40001400, 0x04200040, 0x80200001, 0x00000200 },
 { 0x40000402, 0x01100000, 0x20808000, 0x00008000, 0x10100060, 0x00080000, 0x00001008 },
 { 0x200010A0, 0x00000000, 0x01040100, 0x00000104, 0x02040042, 0x08012000, 0x00000001 },
 { 0x01000000, 0x50000880, 0x00000092, 0x14400000, 0x00001840, 0x02400000, 0x00000000 },
 { 0x00000010, 0x02000000, 0x00014000, 0x00200018, 0x00000240, 0x04000800, 0x00000180 },
 { 0x00008000, 0x00880008, 0x08000044, 0x00100000, 0x00000004, 0x00400820, 0x00001001 },
 { 0x01000000, 0x00002000, 0x02004001, 0x00000042, 0x00000000, 0x09201020, 0x00000048 },
 { 0x00800000, 0x01000400, 0x00400002, 0xC0002000, 0x00002080, 0x00010064, 0x00000100 },
 { 0x00000400, 0x08400840, 0x00000400, 0x00000890, 0x00008102, 0x00000020, 0x00000002 },
 { 0x00200040, 0x00000081, 0x00000000, 0x02050000, 0x04940000, 0x20008020, 0x00000080 },
 { 0x00000404, 0x00800000, 0x00001000, 0x00014000, 0x00082200, 0x0A000400, 0x00000000 },
 { 0x0000A024, 0x00000000, 0x00000402, 0x08A01000, 0x00004010, 0x20000000, 0x00000008 },
 { 0x00480046, 0x00008000, 0x00000208, 0x00000048, 0x00000000, 0x00410010, 0x00000002 },
 { 0x0000008C, 0x00044C00, 0x00824004, 0x00000200, 0x00000000, 0x00028000, 0x00000000 },
 { 0x10010004, 0x00080000, 0x43008000, 0x10000400, 0x80000100, 0x00000040, 0x00000080 },
 { 0x80000000, 0x0020000C, 0x20420480, 0x00000100, 0x00000008, 0x00005410, 0x00000080 },
 { 0x00000000, 0x00101000, 0x08000001, 0x02000200, 0x82004A80, 0x00004000, 0x00000202 }
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


int8_t LDPC::LDPC_Check(const uint8_t *Data) // 20 data bytes followed by 6 parity bytes
{ 
  uint8_t Errors=0;
  for(uint8_t Row=0; Row<48; Row++)
  { 
    uint8_t Count=0;
    const uint8_t *Check = (uint8_t *)LDPC_ParityCheck[Row];
    
    for(uint8_t Idx=0; Idx<26; Idx++)
    { 
      Count+=u32Count1s(Data[Idx] & pgm_read_byte(Check+Idx)); 
    }
    if(Count&1) 
      Errors++; 
  }
  return Errors; 
}


int8_t LDPC::LDPC_CheckBlock(const uint32_t *Packet)
{
    return LDPC_Check( (uint8_t *)Packet ); 
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
