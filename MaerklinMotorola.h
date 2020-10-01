/*
  MaerklinMotorola.h - Library for decoding the signals from the Märklin-Motorola-protocol. 
  Created by Michael Henzler (Laserlicht), Februar 27, 2018.
  Released under BSD 2-Clause "Simplified" License.
*/

#ifndef MaerklinMotorola_h
#define MaerklinMotorola_h

#include "Arduino.h"
#include <avr/pgmspace.h>

#if defined (__AVR_ATtiny85__)
	#define MM_QUEUE_LENGTH	4
#else
	#define MM_QUEUE_LENGTH	10
#endif

/* Bit manipulations for byte arrays */
#define IsBitSet(var,pos)   (!!(var[(pos/8)] & (1 << (pos%8))))  
#define SetBit(var,pos)     (var[(pos/8)] |= (1 << (pos%8)))
#define ClearBit(var,pos)   (var[(pos/8)] &= ~(1 << (pos%8)))

//Mapping table for MM2 addresses from 0...255 (which equals to 1...256 on CS/MS!) (MSB first)
//Found on http://www.drkoenig.de/digital/verbess.htm and translated to index-lookup
//How to read: Use byte value from MM(2) protocol as index. i.e. Byte value 0 means address 80, byte value 1 refers to address 229 and so on
//TODO: This lookup uses a "lot" a program space but I didn't found the logic how to translate byte value to MM2 address value...
const PROGMEM unsigned char MM2_ADDRESS_MAP[256]  =
{ 
	80, 229, 54, 27, 193, 195, 194, 196, 18, 247, 72, 45, 9, 238, 63, 36, 145, 153, 149, 157, 147, 155, 151, 159, 146, 154, 150, 158, 148, 156, 152, 160, 6, 235, 60, 33, 217, 219, 218, 220, 24, 253, 78, 51, 15, 244, 69, 42, 3, 232, 57, 30, 205, 207, 206, 208, 21, 250, 75, 48, 12, 241, 66, 39, 81, 113, 97, 129, 89, 121, 105, 137, 85, 117, 101, 133, 93, 125, 109, 141, 191, 115, 99, 131, 91, 192, 107, 139, 87, 119, 103, 135, 95, 127, 111, 143, 82, 114, 98, 130, 90, 122, 106, 138, 86, 118, 102, 134, 94, 126, 110, 142, 84, 116, 100, 132, 92, 124, 108, 140, 88, 120, 104, 136, 96, 128, 112, 144, 2, 231, 56, 29, 201, 203, 202, 204, 20, 249, 74, 47, 11, 240, 65, 38, 177, 185, 181, 189, 179, 187, 183, 83, 178, 186, 182, 190, 180, 188, 184, 123, 8, 237, 62, 35, 225, 227, 226, 228, 26, 255, 0, 53, 17, 246, 71, 44, 5, 234, 59, 32, 213, 215, 214, 216, 23, 252, 77, 50, 14, 243, 68, 41, 1, 230, 55, 28, 197, 199, 198, 200, 19, 248, 73, 46, 10, 239, 64, 37, 161, 169, 165, 173, 163, 171, 167, 175, 162, 170, 166, 174, 164, 172, 168, 176, 7, 236, 61, 34, 221, 223, 222, 224, 25, 254, 79, 52, 16, 245, 70, 43, 4, 233, 58, 31, 209, 211, 210, 212, 22, 251, 76, 49, 13, 242, 67, 40
};

enum DataGramState
{
	DataGramState_Reading,
	DataGramState_ReadyToParse,
	DataGramState_Parsed,
	DataGramState_Validated,
	DataGramState_Finished,
	DataGramState_Error,
};

enum MM2DirectionState
{
	MM2DirectionState_Unavailable,
	MM2DirectionState_Forward,
	MM2DirectionState_Backward,
};

struct MaerklinMotorolaData {
  //Holds the 18 MM(2) bits
  byte Bits[3];
  unsigned int tm_package_delta;
  MM2DirectionState MM2Direction;
  
  unsigned char Address;
  
  unsigned char Speed;
  unsigned char Step;
  unsigned char MM2FunctionIndex;

  unsigned char SubAddress;

  DataGramState State;
  
  bool Function:1;
  bool Stop:1;
  bool ChangeDir:1;
  bool MagnetState:1; //bei aus werden normalerweise alle ausgeschaltet
  bool IsMagnet:1;
  bool IsMM2:1;
  bool IsMM2FunctionOn:1;
};

//the Timings[] array stores the timing values in a condensed format to fit into a single byte array instead of an unsigned int (16bit)
#define MMTimingsRatio 2

class MaerklinMotorola {
public:
  MaerklinMotorola(unsigned char p);
  void PinChange();
  MaerklinMotorolaData* GetData();

private:
  unsigned char Timings[35];
  unsigned char pin;
  unsigned int last_tm = 0;
  unsigned int sync_tm = 0;
  bool sync:1;
  unsigned char timings_pos = 0;
  unsigned char DataQueueWritePosition = 0;
  MaerklinMotorolaData DataQueue[MM_QUEUE_LENGTH];

  void Parse();
};

#endif
