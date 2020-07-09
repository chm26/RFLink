//#######################################################################################################
//##                    This Plugin is only for use with the RFLink software package                   ##
//##                                      Plugin-052: ATECH WS301E Outdoor Sensor                      ##
//#######################################################################################################
/*********************************************************************************************\
 * This plugin takes care of decoding the protocol used for outdoor sensors of the ATECH WS301E weather station
 * 
 * Author  (present)  : chm26
 * Support (present)  : https://github.com/chm26/RFLink 
 * License            : This code is free for use in any open source project when this header is included.
 *                      Usage of any parts of this code in a commercial application is prohibited!
 *********************************************************************************************
 * The sensor sends 28 +1 bits 4 times every ~57 s
 * The bits are coding by a kind of PPM modulation: each bits are separated by a long gap of ~2080us, 
 * a 0 is represented by one pulse of ~1550 us,
 * a 1 is represented by 3 pulses of ~1550 us separated by a 2 short gaps of ~270 us, 
 * the last bit of a packet (bit 29) is represented by 2 pulses of ~1550 us separated by a short gap of ~270 us
 * Each packet is separated by a gap of ~ 50 or 61 ms.
 * Each packet starts with a preamble pattern (8 pulses with a short gap, for 14400us) with a sync gap of  ~7600us (before message).
 * 
 *  Message Format: (28 bits):
 *
 * Format for Temperature 
 *   AAAA BBBB CCCC EEEE EEEE EEEE FFFF
 *
 *   A = always 0000
 *   B = id/channel: channel II = 1000 or channel I = 1100 
 *   C = T sign: 0000 for >0 , 0010 for < 0
 *   E = Temperature in °C (BCD coded)
 *   F = checksum (XOR of previous bytes)
 * 
 * 20;XX;DEBUG;Pulses=92;Pulses(uSec)=1408,2048,1568,2048,1568,2048,1568,2048,1568,224,1568,256,1568,2080,1568,256,1536,256,1536,2080,1536,2080,1568,2080,1568,2080,1568,2080,1568,2080,1568,2080,1568,2080,1568,2080,1568,224,1568,256,1536,2080,1568,2080,1568,2080,1568,256,1568,256,1536,2080,1568,2080,1568,2080,1568,2080,1568,224,1568,256,1536,2080,1536,256,1536,256,1536,2080,1568,2080,1568,256,1536,256,1536,2080,1536,256,1536,256,1536,2080,1536,2080,1536,1824,1568,256,1536;
\*********************************************************************************************/
#define _PLUGIN_ID 052
#define PLUGIN_DESC_052 "ATECH WS301E"
#define ATECH301_PULSECOUNT_MIN 60 // 60 min to 172 max
#define ATECH301_PULSECOUNT_MAX 172 

#define ATECH301_PULSE_MIN 1400 / RAWSIGNAL_SAMPLE_RATE
#define ATECH301_PULSE_MAX 1600 / RAWSIGNAL_SAMPLE_RATE
#define ATECH301_GAP0_MIN 1700 / RAWSIGNAL_SAMPLE_RATE
#define ATECH301_GAP0_MAX 2200 / RAWSIGNAL_SAMPLE_RATE
#define ATECH301_PAT1_MIN 3400 / RAWSIGNAL_SAMPLE_RATE
#define ATECH301_PAT1_MAX 3700 / RAWSIGNAL_SAMPLE_RATE

#ifdef PLUGIN_052
#include "../4_Display.h"

#if 0
void display_RawSignal(int mode)
{
   // ----------------------------------
   Serial.print(F("20;XX;DEBUG_52;Pulses=")); // debug data
   Serial.print(RawSignal.Number);         // print number of pulses
   Serial.print(F(";Pulses(uSec)="));      // print pulse durations
   // ----------------------------------
   char dbuffer[3];

   for (int i = 0 ; i <= RawSignal.Number ; i++)
   {
      if (mode == 1)
      {
         sprintf(dbuffer, " %02x", RawSignal.Pulses[i]);
         Serial.print(dbuffer);
      }
      else
      {
         Serial.print(RawSignal.Pulses[i] * RAWSIGNAL_SAMPLE_RATE);
         if (i < RawSignal.Number)
            Serial.write(',');
      }
   }
   Serial.print(F(";\r\n"));
}
#endif

//#define ERROR_END_P052(x)   {err = x; goto ERROR;}
#define ERROR_END_P052(x)   {goto ERROR;}

boolean Plugin_052(byte function, char *string)
{
   if ((RawSignal.Number < ATECH301_PULSECOUNT_MIN) || (RawSignal.Number > (ATECH301_PULSECOUNT_MAX)))
      return false;

   byte buf[11];     // 11 = decoding worst wrong case ( (ATECH301_PULSECOUNT_MAX-4) /2 )
   int i = 0;
   int temperature = 0;
   byte cksum;
   unsigned long tmpval;
   byte c = 0;
   byte nbit = 0;
   //byte err = 0;

   //==================================================================================
   // Decode all bits in buff
   //==================================================================================
   buf[i] = 0;
   for (byte x = 1 ; x <= (RawSignal.Number-4) ; x++)      //Signal starts at Pulses[1], -4 don't decode "stop" bit
   { 
      if ((RawSignal.Pulses[x] < ATECH301_PULSE_MIN) || (RawSignal.Pulses[x] > ATECH301_PULSE_MAX))
         ERROR_END_P052(__LINE__);

      c <<= 1; // Always shift
      x++;
      if ((RawSignal.Pulses[x] > ATECH301_GAP0_MIN) && (RawSignal.Pulses[x] < ATECH301_GAP0_MAX)) 
      {
         // case long gap -> bit 0
      }
      else
      {  //check case bit 1: short gap + pulse + short gap + pulse ~ 3.7 ms
         unsigned int total = RawSignal.Pulses[x] + RawSignal.Pulses[x+1] + RawSignal.Pulses[x+2] + RawSignal.Pulses[x+3];
         if ((total > ATECH301_PAT1_MAX) || (total < ATECH301_PAT1_MIN)) 
           ERROR_END_P052(__LINE__);
         c |= 0x1;
         x += 4;
      }
      nbit ++;
      if((nbit % 8) == 0)
      {
         buf[i++] = c;
         c = 0; 
      }
   }
   buf[i] = c;

   //==================================================================================
   // Perform a quick sanity check
   //==================================================================================
   if (nbit != 28 )
      ERROR_END_P052(__LINE__);

   cksum = buf[0] ^ buf[1] ^ buf [2];
   cksum = (cksum & 0x0F) ^ (cksum >> 4);
   if(cksum != buf[3])
      ERROR_END_P052(__LINE__);

   //==================================================================================
   // Now process the various sensor types
   //==================================================================================
   temperature = (buf[1] & 0x0F)*100 + (buf[2]>>4)*10 + (buf[2] & 0x0F);
   if(buf[1] & 0x20)     
      temperature = temperature + 0x8000;      // set temp. <0 according to RFlink format
      
   //==================================================================================
   // Prevent repeating signals from showing up
   //==================================================================================
   tmpval = (buf[0]<<16) + temperature;

   if ((SignalHash != SignalHashPrevious) || ((RepeatingTimer + 500) < millis()) || (SignalCRC != tmpval))
      SignalCRC = tmpval; // not seen this RF packet recently
   else
      return true; // already seen the RF packet recently

   //==================================================================================
   // Output
   //==================================================================================
   display_Header();
   display_Name(PSTR("Atech 301E"));
   display_IDn(buf[0], 2);
   display_TEMP(temperature);
   display_RawFrame(buf, nbit);
   display_Footer();

   //==================================================================================
   RawSignal.Repeats = true; // suppress repeats of the same RF packet
   RawSignal.Number = 0;
   return true;

ERROR:
 /*   Serial.print(" Error "); Serial.println(err);
   display_RawSignal(0);*/

    return false;
}
#endif // PLUGIN_052
