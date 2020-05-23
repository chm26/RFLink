//#######################################################################################################
//##                    This Plugin is only for use with the RFLink software package                   ##
//##                                      Plugin-051: DIGOO DG-EX003 Outdoor Sensor                    ##
//#######################################################################################################
/*********************************************************************************************\
 * This plugin takes care of decoding the protocol used for outdoor sensors of the Diggo DG-EX003 weather station
 * 
 * Author  (present)  : chm26
 * Support (present)  : https://github.com/chm26/RFLink 
 * License            : This code is free for use in any open source project when this header is included.
 *                      Usage of any parts of this code in a commercial application is prohibited!
 * Note: the frame structure is from rtl_433 infactory.c (https://github.com/merbanan/rtl_433)
 *********************************************************************************************
 * The sensor sends 40 bits 6 times every 68s
 * The packets are PPM or PDM modulated (distance coding) with a pulse of ~570 us
 * followed by a short gap of ~2000 us for a 0 bit or a long ~4000 us gap for a 1 bit, 
 * the sync gap is ~16000 us.
 * Note : before each packet, there is a preamble pattern : 4 (pulse + gap) of 1ms (thus total of 8ms) + 1 pulse of 560us with a gap 7.9ms
 *  Message Format: (40 bits):
 *
 * Format for Temperature Humidity
 *   AAAA AAAA BBBB CCCC EEEE EEEE EEEE FFFF FFFF GGGG
 *
 *   A = Id (new code at each power on)
 *   B = Checksum ? 
 *   C = T tendency ??? (0: stable, 1: T increase, 2: T decrease) ?
 *   E = Temperature + 90° in F (binary coded)
 *   F = Humidity (BCD format)
 *   G = channel?
 * 
 * Note : rtl_433 see this sensor as a infactory device.
 * 
 * 20;XX;DEBUG;Pulses=82;Pulses(uSec)=512,1888,544,4064,544,1856,544,4128,512,4000,544,4064,544,4064,544,4128,512,1792,544,1856,544,1856,544,1952,512,1792,544,1856,544,1856,544,1952,512,1792,544,4064,544,4064,544,1952,512,1792,544,1856,544,4064,544,4128,512,4000,544,4064,544,4064,544,1952,512,1792,544,4064,544,1856,544,4128,512,4096,544,1760,544,1856,544,1952,512,1792,544,1856,544,1856,544,4064,576,4992;
 \*********************************************************************************************/
#define DIGOOEX3_PLUGIN_ID 051
#define PLUGIN_DESC_051 "DIGOO EX3"
#define DIGOOEX3_PULSECOUNT 82  // 40 bit + sync/timeout 

#define DIGOOEX3_MIDHI 700 / RAWSIGNAL_SAMPLE_RATE
#define DIGOOEX3_PULSEMIN 1500 / RAWSIGNAL_SAMPLE_RATE
#define DIGOOEX3_PULSEMINMAX 2500 / RAWSIGNAL_SAMPLE_RATE
#define DIGOOEX3_PULSEMAXMIN 3000 / RAWSIGNAL_SAMPLE_RATE

#ifdef PLUGIN_051
#include "../4_Display.h"


#if ((SIGNAL_MIN_PREAMBLE_US + SIGNAL_END_TIMEOUT_US) > 7000)  // in fact delay between preamble and frame is 7.9ms. If MIN_PREAMBLE is too long, the beginning of the frame is missed.
#error SIGNAL_MIN_PREAMBLE_US is to long to detect correctly the start of a frame
#endif

boolean Plugin_051(byte function, char *string)
{
   if (RawSignal.Number != DIGOOEX3_PULSECOUNT)
      return false;

   byte buf[5];
   byte nbuf = 0;
   int32_t temperature = 0;
   byte humidity = 0;
   byte c = 0;
   byte nbit = 0;

   //==================================================================================
   // Get all 40 bits
   //==================================================================================
   byte start = 0;

   buf[nbuf] = 0;
   for (byte x = start+1 ; x <= (DIGOOEX3_PULSECOUNT-2 + start) ; x++)      //Signal starts at Pulses[1], 
   { 
      if (RawSignal.Pulses[x] > DIGOOEX3_MIDHI)
         return false;

      c <<= 1; // Always shift
      x++;
      if (RawSignal.Pulses[x] > DIGOOEX3_PULSEMAXMIN)
         c |= 0x1;
      else
      {
         if (RawSignal.Pulses[x] < DIGOOEX3_PULSEMIN)
            return false;
         if (RawSignal.Pulses[x] > DIGOOEX3_PULSEMINMAX)
            return false;
         // c |= 0x0;
      }

      if(++nbit == 8)
      {
         buf[nbuf++] = c;
         c= 0; 
         nbit = 0;
      }
   }
   buf[nbuf] = c;

   humidity = (buf[3]<<4) + (buf[4]>>4);
   temperature = (((buf[2]<<4) + (buf[3]>>4) - 900 -320)*5)/9;  //convert in °C * 10
   if(temperature < 0)     
      temperature = -temperature + 0x8000;      // set temp. <0 according to RFlink format
      
   //==================================================================================
   // Perform a quick sanity check
   //==================================================================================
   //if (temperature > 0x258)
   //   return false; // temperature out of range ( > 60.0 degrees)

   if ((humidity == 0) || (humidity > 0x99))
      return false; // Humidity out of range
   //==================================================================================
   // Prevent repeating signals from showing up
   //==================================================================================
   unsigned long tmpval = ((temperature << 8) | humidity);

   if ((SignalHash != SignalHashPrevious) || ((RepeatingTimer + 500) < millis()) || (SignalCRC != tmpval))
      SignalCRC = tmpval; // not seen this RF packet recently
   else
      return true; // already seen the RF packet recently

   //==================================================================================
   // Output
   //==================================================================================
   display_Header();
   display_Name(PSTR("Digoo EX3"));
   display_IDn(buf[0], 2);
   display_TEMP(temperature);
   if (humidity <= 0x99) // Only report valid humidty values
      display_HUM(humidity, HUM_BCD);
   display_Footer();
   //==================================================================================
   RawSignal.Repeats = true; // suppress repeats of the same RF packet
   RawSignal.Number = 0;
   return true;
}
#endif // PLUGIN_051
