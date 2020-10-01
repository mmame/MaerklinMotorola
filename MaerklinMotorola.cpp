/*
  MaerklinMotorola.cpp - Library for decoding the signals from the Märklin-Motorola-protocol. 
  Created by Michael Henzler (Laserlicht), Februar 27, 2018.
  Released under BSD 2-Clause "Simplified" License.
*/

#include <MaerklinMotorola.h>

MaerklinMotorola::MaerklinMotorola(unsigned char p) {
  pin = p;
  DataQueueWritePosition = 0;
  sync = false;
}

MaerklinMotorolaData* MaerklinMotorola::GetData() {
	Parse();
	for(unsigned char QueuePos=0; QueuePos<MM_QUEUE_LENGTH;QueuePos++) {
		if(DataGramState_Validated == DataQueue[QueuePos].State) {
			DataQueue[QueuePos].State = DataGramState_Finished;
			return DataQueue + QueuePos;
		}
	}
	return 0;
}

void MaerklinMotorola::Parse() {
	for(unsigned char QueuePos=0; QueuePos<MM_QUEUE_LENGTH;QueuePos++) {
		if(DataGramState_ReadyToParse == DataQueue[QueuePos].State) {
			
		  bool valid = true;
		  bool parsed = false;

		  DataQueue[QueuePos].IsMM2 = false;
		  DataQueue[QueuePos].Function = false;
		  DataQueue[QueuePos].Stop = false;
		  DataQueue[QueuePos].ChangeDir = false;
		  DataQueue[QueuePos].IsMM2FunctionOn = false;
		  DataQueue[QueuePos].Address = 0;
		  DataQueue[QueuePos].SubAddress = 0;
		  DataQueue[QueuePos].Speed = 0;
		  DataQueue[QueuePos].MagnetState = 0;
		  DataQueue[QueuePos].MM2FunctionIndex = 0;
		  DataQueue[QueuePos].MM2Direction = MM2DirectionState_Unavailable;
		  
		  //For MM1, the only possible values of the least 4 "trits" are 11 or 00 - in MM2 they are quarternary
		  for(unsigned char i=5;i<9;i++) { //Trits aus Bits dekodieren
			if(IsBitSet(DataQueue[QueuePos].Bits, i*2) != IsBitSet(DataQueue[QueuePos].Bits, i*2+1))
			{
				//10 or 01
				//MM1 trailing "trits" only use "11" and "00" so we have MM2 here
				DataQueue[QueuePos].IsMM2 = true;
			}
		  }

		  //Decoder
		  if(DataQueue[QueuePos].tm_package_delta > 1300 && DataQueue[QueuePos].tm_package_delta < 4200 && valid) { //Protokollspezifische Telegramlänge: Weichen oder Lokprotokoll
			
			DataQueue[QueuePos].Address = 0;
			
			//Lookup MM(2) address from given address byte (first 8 bits of MM(2) message)
			unsigned char address = IsBitSet(DataQueue[QueuePos].Bits, 7) + IsBitSet(DataQueue[QueuePos].Bits, 6) * 2 + IsBitSet(DataQueue[QueuePos].Bits, 5) * 4 + IsBitSet(DataQueue[QueuePos].Bits, 4) * 8
								+ IsBitSet(DataQueue[QueuePos].Bits, 3) * 16 + IsBitSet(DataQueue[QueuePos].Bits, 2) * 32 + IsBitSet(DataQueue[QueuePos].Bits, 1) * 64 + IsBitSet(DataQueue[QueuePos].Bits, 0) * 128;
			DataQueue[QueuePos].Address = pgm_read_byte_near(MM2_ADDRESS_MAP + address);

			if(!DataQueue[QueuePos].IsMagnet) { //Loktelegramm
			  DataQueue[QueuePos].Function = IsBitSet(DataQueue[QueuePos].Bits, 8);

			  unsigned char s = IsBitSet(DataQueue[QueuePos].Bits, 10) + IsBitSet(DataQueue[QueuePos].Bits, 12) * 2 + IsBitSet(DataQueue[QueuePos].Bits, 14) * 4 + IsBitSet(DataQueue[QueuePos].Bits, 16) * 8;
			  DataQueue[QueuePos].Stop = (s==0) ? true : false;
			  DataQueue[QueuePos].ChangeDir = (s==1) ? true : false;
			  DataQueue[QueuePos].Speed = (s==0) ?  0 : s-1;
			  if(DataQueue[QueuePos].IsMM2)
			  {
				//convert MM2 bits to one number
				unsigned char sMM2 = IsBitSet(DataQueue[QueuePos].Bits, 17) + IsBitSet(DataQueue[QueuePos].Bits, 15) * 2 + IsBitSet(DataQueue[QueuePos].Bits, 13) * 4 + IsBitSet(DataQueue[QueuePos].Bits, 11) * 8;
				
				switch(sMM2)
				{
					case 2:
					case 3:
					DataQueue[QueuePos].MM2FunctionIndex = 2;
					DataQueue[QueuePos].IsMM2FunctionOn = sMM2 & 1;
					break;

					case 4:
					case 5:
					DataQueue[QueuePos].MM2Direction = MM2DirectionState_Forward;
					break;

					case 6:
					case 7:
					DataQueue[QueuePos].MM2FunctionIndex = 3;
					DataQueue[QueuePos].IsMM2FunctionOn = sMM2 & 1;
					break;

					case 10:
					case 11:
					DataQueue[QueuePos].MM2Direction = MM2DirectionState_Backward;
					break;

					case 12:
					case 13:
					DataQueue[QueuePos].MM2FunctionIndex = 1;
					DataQueue[QueuePos].IsMM2FunctionOn = sMM2 & 1;
					break;

					case 14:
					case 15:
					DataQueue[QueuePos].MM2FunctionIndex = 4;
					DataQueue[QueuePos].IsMM2FunctionOn = sMM2 & 1;
					break;
					
					default:
					break;
				}
			  }

			  parsed=true;
			} else { //Magnettelegramm
			  if(!IsBitSet(DataQueue[QueuePos].Bits, 8)) {
				unsigned char s = IsBitSet(DataQueue[QueuePos].Bits, 10) + IsBitSet(DataQueue[QueuePos].Bits, 12) * 2 + IsBitSet(DataQueue[QueuePos].Bits, 14) * 4;
				DataQueue[QueuePos].SubAddress = s;
				DataQueue[QueuePos].MagnetState = IsBitSet(DataQueue[QueuePos].Bits, 16);
				parsed=true;
			  }
			}  
		  }	
		  if(parsed) {
			  //Get previous DataGram from Queue
			  unsigned char previousDataGramPos = QueuePos > 0 ? QueuePos - 1 : MM_QUEUE_LENGTH - 1;
			  DataQueue[QueuePos].State = DataGramState_Parsed;
			  if(DataGramState_Parsed == DataQueue[previousDataGramPos].State) {
				  //Check if previous DataGram was identical
				  if(0 == memcmp(DataQueue[QueuePos].Bits, DataQueue[previousDataGramPos].Bits, 3)) {
					  DataQueue[QueuePos].State = DataGramState_Validated;
				  }
			  }
		  }
		  else {
			  //Invalid frame
			  DataQueue[QueuePos].State = DataGramState_Error;
		  }
		}
	}
}

void MaerklinMotorola::PinChange() {
  //bool state = digitalRead(pin);
  unsigned int tm = micros();
  unsigned int tm_delta = tm - last_tm;
  unsigned int period;
  bool valid;

  if(sync) { //erst nach syncronisation bits sammeln
	//store timings "compressed" (drop LSB)
    Timings[timings_pos] = tm_delta / MMTimingsRatio; //ablage des zeitunterschieds zwischen den letzten flanken
    timings_pos++;

	if(tm_delta>500) {
		//timeout - resync
		timings_pos = 0;
		sync = true;
		sync_tm = tm;
	}
    if(timings_pos==35) {
      DataQueue[DataQueueWritePosition].tm_package_delta = tm - sync_tm; //paket-laenge berechen
	  
	  period = (Timings[0] * MMTimingsRatio) + (Timings[1] * MMTimingsRatio); //bit-laenge berechnen	  
	  DataQueue[DataQueueWritePosition].IsMagnet = ((period < 150) ? true : false);  //Unterscheidung Protokoll (Fest-Zeit)
	  
	  valid = true;
	  
	  //zero all bits
	  DataQueue[DataQueueWritePosition].Bits[0] = 0;
	  DataQueue[DataQueueWritePosition].Bits[1] = 0;
	  DataQueue[DataQueueWritePosition].Bits[2] = 0;
	  
	  /*
	  Serial.print(F(" Timings: "));
	  for(unsigned char i=0;i<35;i++) 
	  {
		Serial.print(Timings[i]);
		Serial.print(' ');
	  }
	  Serial.println();
	  */
	  
	  for(unsigned char i=0;i<35;i+=2) { //Bits dekodieren
	  if((Timings[i] * MMTimingsRatio) > (period>>1))
	  {
		  //Länger als die Hälfte: 1
		  SetBit(DataQueue[DataQueueWritePosition].Bits, i/2);
	  }

	  if(i<33) {
		  unsigned int period_tmp = Timings[i] * MMTimingsRatio + Timings[i+1] * MMTimingsRatio;
		  if(period_tmp > 125 && period_tmp < 175) 
		  {
			  valid = false; //MFX herausfiltern
		  }
		}
	  }	  

	  if(valid)
	  {
		  DataQueue[DataQueueWritePosition].State = DataGramState_ReadyToParse;
		  DataQueueWritePosition ++;
		  //Queue end - go to queue start
		  if(MM_QUEUE_LENGTH <= DataQueueWritePosition)
		  {
			 DataQueueWritePosition = 0;
		  }
	  }
	  
	  DataQueue[DataQueueWritePosition].State = DataGramState_Reading;
      sync = false;
      timings_pos = 0;
    }
  } else {
    if(tm_delta>500) { //Protokollspezifische Pausen-Zeit
      sync = true;
      sync_tm = tm;
    }
  }

  last_tm = tm;
}
