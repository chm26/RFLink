//#######################################################################################################
//##                    This Plugin is only for use with the RFLink software package                   ##
//##                                      Plugin-050: DIGOO R8S                                        ##
//#######################################################################################################
/*********************************************************************************************\
 * This plugin takes care of decoding the protocol used for outdoor sensors of the Diggo Digoo DG-R8S weather stations 
 * 
 * Author  (present)  : chm26
 * Support (present)  : https://github.com/chm26/RFLink 
 * License            : This code is free for use in any open source project when this header is included.
 *                      Usage of any parts of this code in a commercial application is prohibited!
 * Note: this plugin is initially based on Plugin_032 from https://github.com/couin3/RFLink
 *********************************************************************************************
 * The sensor sends 37 bits 7 times every 50s.
 * The packets are PPM (or PDM ?) modulated (distance coding) with a pulse of ~570 us
 * followed by a short gap of ~2100 us for a 0 bit or a long ~4100 us gap for a 1 bit, 
 * the sync gap is ~9000 us. Before the first packet there is a pulse with a sync gap (9 ms).
 * 
 *  Message Format: (37 bits):
 *
 * Format for Temperature Humidity
 *   AAAA BBBBBBBB CDEE FFFF FFFF FFFF GGGGGGGG H
 *
 *   A = Type (5 or 9)
 *   B = Rolling Code
 *   C = Battery low
 *   D = 0=scheduled transmission, 1=requested transmission (test button)
 *   E = Channel number (00=ch1 01=ch2 10=ch3)
 *   F = Temperature (two's complement)
 *   G = Humidity binary format
 *   H = always 0 (short bit)
 *
 * Note 1: this format is very similar to Plugin_032 (AlectoV4) except H bit.
 * Note 2: In case of Test button, H bit of the 1st packet has a strange gap of ~7300 us (which may be interpreted has a sync) and thus the packet is 36 bits and looks like plugin32 (Alecto V4)
 * 
 * 20;XX;DEBUG;Pulses=76;Pulses(uSec)=576,4032,576,1984,576,1984,576,4032,576,2016,576,4032,576,4064,576,4064,576,4032,576,4064,576,4064,576,1984,576,4096,576,2016,576,2016,576,2016,576,2016,576,2016,576,2016,576,2016,576,4032,576,4064,576,4064,576,2016,576,4064,576,4064,576,2016,576,2016,576,2016,576,2016,576,4064,576,4128,576,1984,576,4032,576,2016,576,2016,576,2112,576,288;
 \*********************************************************************************************/
#define DIGOOR8S_PLUGIN_ID 050
#define PLUGIN_DESC_050 "DIGOO R8S"
#define DIGOOR8S_PULSECOUNT 76

#define DIGOOR8S_MIDHI 600 / RAWSIGNAL_SAMPLE_RATE
#define DIGOOR8S_PULSEMIN 1500 / RAWSIGNAL_SAMPLE_RATE
#define DIGOOR8S_PULSEMINMAX 2500 / RAWSIGNAL_SAMPLE_RATE
#define DIGOOR8S_PULSEMAXMIN 3000 / RAWSIGNAL_SAMPLE_RATE

#ifdef PLUGIN_050
#include "../4_Display.h"

boolean Plugin_050(byte function, char *string)
{
   //Serial.print(RawSignal.Number);
   if (RawSignal.Number != DIGOOR8S_PULSECOUNT)
      return false;

   unsigned long bitstream = 0L;
   int temperature = 0;
   byte humidity = 0;
   byte channel = 0;   
   byte bat = 0;
   //byte test = 0;
   byte rc = 0;
   byte rc2 = 0;
   //==================================================================================
   // Get the 36 bits (ignore H)
   //==================================================================================
   byte start = 0;

   for (byte x = 2 + start; x <= 56 + start; x += 2)
   { // Get first 28 bits
      if (RawSignal.Pulses[x + 1] > DIGOOR8S_MIDHI)
         return false;

      bitstream <<= 1; // Always shift
      if (RawSignal.Pulses[x] > DIGOOR8S_PULSEMAXMIN)
         bitstream |= 0x1;
      else
      {
         if (RawSignal.Pulses[x] < DIGOOR8S_PULSEMIN)
            return false;
         if (RawSignal.Pulses[x] > DIGOOR8S_PULSEMINMAX)
            return false;
         // bitstream |= 0x0;
      }
   }

   for (byte x = 58 + start; x <= 72 + start; x = x + 2)
   { // Get remaining 8 bits
      if (RawSignal.Pulses[x + 1] > DIGOOR8S_MIDHI)
         return false;

      humidity <<= 1; // Always shift
      if (RawSignal.Pulses[x] > DIGOOR8S_PULSEMAXMIN)
         humidity |= 0x1;
      else
      {
         if (RawSignal.Pulses[x] < DIGOOR8S_PULSEMIN)
            return false;
         if (RawSignal.Pulses[x] > DIGOOR8S_PULSEMINMAX)
            return false;
         // humidity |= 0x0;
      }
   }
   //==================================================================================
   // Perform a quick sanity check
   //==================================================================================
   if (bitstream == 0)
      return false;

   if (humidity == 0)
      return false; // Sanity check

   //==================================================================================
   // Now process the various sensor types
   //==================================================================================
   rc = (bitstream >> 24) & 0x0F;
   rc2 = (bitstream >> 16) & 0xFF;
   /*if (((rc2)&0x08) != 0x08)
      return false; // needs to be 1 */
   temperature = (bitstream & 0xFFF);
   channel = ((bitstream & 0x3000) >> 12) + 1;
   bat = ((bitstream & 0x8000) == 0);  // bit = battery low
   //test = ((bitstream & 0x4000) !=0);
   //fix 12 bit signed number conversion
   if ((temperature & 0x800) == 0x800)
   {
      temperature = 4096 - temperature; // fix for minus temperatures
      if (temperature > 0x258)
         return false;                    // temperature out of range ( > 60.0 degrees)
      temperature = temperature | 0x8000; // turn highest bit on for minus values
   }
   else
   {
      if (temperature > 0x258)
         return false; // temperature out of range ( > 60.0 degrees)
   }
   if (humidity > 99)
      return false; // Humidity out of range

   //==================================================================================
   // Prevent repeating signals from showing up
   //==================================================================================
   unsigned long tmpval = (((bitstream << 8) & 0xFFF00) | humidity); // All but 8 1st ID bits ...

   if ((SignalHash != SignalHashPrevious) || ((RepeatingTimer + 500) < millis()) || (SignalCRC != tmpval))
      SignalCRC = tmpval; // not seen this RF packet recently
   else
      return true; // already seen the RF packet recently
   //==================================================================================
   // Output
   //==================================================================================
   display_Header();
   display_Name(PSTR("Digoo R8S"));
   char c_ID[5];
   sprintf(c_ID, "%01x%02x%01x", rc, rc2, channel);
   display_IDc(c_ID);
   display_TEMP(temperature);
   if (humidity < 99) // Only report valid humidty values
      display_HUM(humidity, HUM_HEX);
   display_BAT(bat);
   display_Footer();
   //==================================================================================
   RawSignal.Repeats = true; // suppress repeats of the same RF packet
   RawSignal.Number = 0;
   return true;
}
#endif // PLUGIN_050
