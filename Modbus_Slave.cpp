/*
	ModBusSlave.cpp is a library for the Arduino System.  It complies with ModBus protocol, customized for exchanging 
	information between Industrial controllers and the Arduino board.
	
	Copyright (c) 2012 Tim W. Shilling (www.ShillingSystems.com)
	Arduino Modbus Slave is free software: you can redistribute it and/or modify it under the terms of the 
	GNU General Public License as published by the Free Software Foundation, either version 3 of the License, 
	or (at your option) any later version.

	Arduino Modbus Slave is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
	without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
	See the GNU General Public License for more details.

	To get a copy of the GNU General Public License see <http://www.gnu.org/licenses/>.
*/

#include "Arduino.h"
#include "Modbus_Slave.h"
#include <CRC16.h>

CRC CheckSum;	// From Checksum Library, CRC15.h, CRC16.cpp

//################## ModBusSlave ###################
// Takes:   Slave Address, Pointer to Registers and Number of Available Registers
// Returns: Nothing
// Effect:  Initializes Library

ModBusSlave::ModBusSlave(unsigned char _Slave, unsigned short * _Coils, unsigned short _Coil_Count)
{
	Slave = _Slave;
	Coils = _Coils;
	Error = 0;
	Coil_Count = _Coil_Count;
}

//################## Process Data ###################
// Takes:   Data stream buffer from serial port, number of characters to read
// Returns: Nothing
// Effect:  Reads in and parses data

void ModBusSlave::Process_Data(unsigned char *Buffer, unsigned char Count)
{
    Count--;									// Convert Byte count to byte index;
    unsigned char Slave_Address = Buffer[0];	// Grab Slave Address out of buffer
	if(Slave_Address > 247)						// If Address greater than 247 => Invalid
	{
		Error=1;
		return;
	}
    if(Slave_Address != 0) 						// Slave = 0 for Broadcast Msg
    {
      if(Slave != Slave_Address)  				// If Msg is not for this Slave
      {
        return;
      }
    }
    unsigned char Function = Buffer[1];									// Grab Function Code
    unsigned short CRC = ((Buffer[Count]<<8)|Buffer[Count-1]);			// Transmitted CRC
    unsigned short Recalculated_CRC = CheckSum.CRC16(Buffer,Count-1);	// Computer CRC
    if(Recalculated_CRC != CRC)											// Compare CRC, if not equal, then...
    {
		Error = 2;
      return;
    }
    if(Function == 1)													//Read Coils
    {
      Read_Coils(Buffer);
    }
    if(Function == 2)													//Read Descrete Input
    {
      Read_Coils(Buffer);
    }
    if(Function == 3)													//Read Reg
    {
      Read_Reg(Buffer);
    }
    if(Function == 4)													//Read Input Reg
    {
      Read_Reg(Buffer);
    }
    if(Function == 5)													//Write Single Coil
    {
      Write_Single_Coil(Buffer);
    }
    if(Function == 6)													//Write Single Reg
    {
      Write_Single_Reg(Buffer);
    }
	if(Function == 7)													//Read Exception Status
	{
		Read_Exception(Buffer);
	}
    if(Function == 15)													//Write Coils
    {
      Write_Coils(Buffer);
    }
    if(Function == 16)													//Write Reg
    {
      Write_Reg(Buffer);
    }
	Error = 0;															//We made it to the end, Set Error to 0
}

//################## Read Exception ###################
// Takes:   In Data Buffer
// Returns: Nothing
// Effect:  Sets Reply Data and sends Response
void ModBusSlave::Read_Exception(unsigned char *Data_In)
{
	Data_In[2] = Error;
	Send_Response(Data_In,3);
	Error = 0;
}
//################## Send Response ###################
// Takes:   In Data Buffer, Length
// Returns: Nothing
// Effect:  Sends Response over serial port
void ModBusSlave::Send_Response(unsigned char *Data_In,unsigned short Length)
{
	if(Data_In[0] == 0) 										// If Broadcast Msg, then no reply	
		return;
  if(Length > 0)												// If there is Data to be sent, then...
  {
    unsigned short MyCRC = CheckSum.CRC16(Data_In,Length);		// Calculate new CRC
    Data_In[Length++] = MyCRC & 0x00ff;							// Load lower byte into Buffer
    Data_In[Length++] = MyCRC >> 8;								// Load upper byte into Buffer
    for(int C = 0; C < Length;C++)								// Write Data
    {
      Serial.write(Data_In[C]);
    }
  }
}
//################## Write_Single_Reg ###################
// Takes:   In Data Buffer
// Returns: Nothing
// Effect:  Sets Reply Data and Register values, sends Response for Write Single Reg.
void ModBusSlave::Write_Single_Reg(unsigned char *Data_In)  // Function code 1
{
  unsigned short Addr_Hi = Data_In[2];
  unsigned short Addr_Lo = Data_In[3];
  unsigned short Value_Hi = Data_In[4];
  unsigned short Value_Lo = Data_In[5];

  unsigned short Address = (Addr_Lo + (Addr_Hi<<8));
  	if(Address >= Coil_Count)								// Invalid Address;
	{
		Error = 3;
		return;
	}
  Coils[Address] = (Value_Hi<<8) + Value_Lo;
  Send_Response(Data_In,6);
}
//################## Write_Single_Coil ###################
// Takes:   In Data Buffer
// Returns: Nothing
// Effect:  Writes single bit in Registers.  Sets Reply Data and sends Response
void ModBusSlave::Write_Single_Coil(unsigned char *Data_In)  // Function code 1
{
  unsigned short Addr_Hi = Data_In[2];
  unsigned short Addr_Lo = Data_In[3];
  unsigned short Value_Hi = Data_In[4];
  unsigned short Value_Lo = Data_In[5];
  unsigned short Address = Addr_Lo + (Addr_Hi<<8);  
  unsigned short Write_Address = Address/16;
  unsigned char Write_Bit = Address&0x000F;
    	if(Address >= Coil_Count*16)			// Invalid Address;
	{
		Error = 3;
		return;
	}
    if(Value_Hi>0 | Value_Lo > 0)  				// Real Protocol requires 0xFF00 = On and 0x0000 = Off, Custom, using anything other than 0 => ON
    {
      Coils[Write_Address] |= (1<<Write_Bit);
    }
    else
    {
      Coils[Write_Address] &= ~(1<<Write_Bit);
    }
  Send_Response(Data_In,6);
}
//################## Write_Reg ###################
// Takes:   In Data Buffer
// Returns: Nothing
// Effect:  Writes Register in Registers.  Sets Reply Data and sends Response
void ModBusSlave::Write_Reg(unsigned char *Data_In)  // Function code 1
{
  unsigned short Addr_Hi = Data_In[2];
  unsigned short Addr_Lo = Data_In[3];
  unsigned short Cnt_Hi = Data_In[4];
  unsigned short Cnt_Lo = Data_In[5];
  unsigned char Byte_Count = Data_In[6];
  unsigned short Address = (Addr_Lo + (Addr_Hi<<8));
    	if(Address >= Coil_Count)	// Invalid Address;
	{
		Error = 3;
		return;
	}
  unsigned short Read_Byte = 7;  // First entry in input Data_In
  for(int C = 0; C < Byte_Count;C+=2)
  {
	  Coils[Address] = (Data_In[Read_Byte]<<8)+Data_In[Read_Byte+1];
      Address+=1;
      Read_Byte+=2;
  }
  Send_Response(Data_In,6);
}
//################## Write_Coils ###################
// Takes:   In Data Buffer
// Returns: Nothing
// Effect:  Writes multiple bits in Registers.  Sets Reply Data and sends Response
void ModBusSlave::Write_Coils(unsigned char *Data_In)  // Function code 1
{
  unsigned short Addr_Hi = Data_In[2];
  unsigned short Addr_Lo = Data_In[3];
  unsigned short Cnt_Hi = Data_In[4];
  unsigned short Cnt_Lo = Data_In[5];
  unsigned char Byte_Count = Data_In[6];
  
  unsigned short Address = Addr_Lo + (Addr_Hi<<8);
  unsigned short Write_Bit_Count = Cnt_Lo + (Cnt_Hi<<8);
  	if(Address >= Coil_Count*16)	// Invalid Address;
	{
		Error = 3;
		return;
	}
  
  unsigned short Write_Address = Address/16;
  unsigned char Write_Bit = Address&0x000F;
  
  unsigned short Read_Byte = 7; // First entry in input Data_In
  unsigned char Read_Bit = 0;
  for(int C = 0; C < Write_Bit_Count;C++)
  {
    if((Data_In[Read_Byte]&(1<<Read_Bit))>0)		// If set Bit is a 1, then, set corrisponding bit in register
    {
      Coils[Write_Address] |= (1<<Write_Bit);
    }
    else											// If set bit is a 0, clear the bit in the register
    {
      Coils[Write_Address] &= ~(1<<Write_Bit);
    }
    Read_Bit++;										// Increment the Read Bit
    Write_Bit++;									// Increment the write bit	
    if(Read_Bit>=8)									// Reads are from bytes of data, so increment every 8th bit
    {
      Read_Bit = 0;
      Read_Byte++;
    }
    if(Write_Bit >= 16)								// Write are to 16bit registers, so increment every 16th bit.	
    {
      Write_Bit = 0;
      Write_Address++;
    }
  }
  Send_Response(Data_In,6);
}
//################## Read Reg ###################
// Takes:   In Data Buffer
// Returns: Nothing
// Effect:  Reads bytes at a time and composes 16 bit replies.  Sets Reply Data and sends Response
void ModBusSlave::Read_Reg(unsigned char *Data_In)  // Function code 1
{
  unsigned short Addr_Hi = Data_In[2];
  unsigned short Addr_Lo = Data_In[3];
  unsigned short Cnt_Hi = Data_In[4];
  unsigned short Cnt_Lo = Data_In[5];
  //Read date
  unsigned short Byte_Count = Cnt_Lo + (Cnt_Hi<<8);
  Data_In[2] = Byte_Count * 2;
  
  unsigned short Address = (Addr_Lo + (Addr_Hi<<8));
    	if(Address >= Coil_Count)	// Invalid Address;
	{
		Error = 3;
		return;
	}
  unsigned short Item = 3;
  for(int Count = 0; Count < Byte_Count; Count++)
  {
    Data_In[Item+1] = lowByte(Coils[Address]);
    Data_In[Item] = highByte(Coils[Address]);
	Address++;
    Item+=2;
  }
  Send_Response(Data_In,Item);
}

//################## Read Coils ###################
// Takes:   In Data Buffer
// Returns: Nothing
// Effect:  Reads a bit at a time, composing 8 bit replies.  Sets Reply Data and sends Response
void ModBusSlave::Read_Coils(unsigned char *Data_In)  // Function code 1
{
  unsigned short Addr_Hi = Data_In[2];
  unsigned short Addr_Lo = Data_In[3];
  unsigned short Cnt_Hi = Data_In[4];
  unsigned short Cnt_Lo = Data_In[5];
  //Read date
  unsigned short Bit_Count = Cnt_Lo + (Cnt_Hi<<8);
  unsigned char Byte_Count = 0;
  unsigned char Sub_Bit_Count = 0;
  unsigned char Working_Byte = 0;
  unsigned short Address = Addr_Lo + (Addr_Hi<<8);
    	if(Address >= Coil_Count*16)	// Invalid Address;
	{
		Error = 3;
		return;
	}
  unsigned short Item = 3;
  for(int Bit = 0; Bit < Bit_Count; Bit++)
  {
    Working_Byte = Working_Byte | ((Coils[Address/16]>>(Address&0x000f))<<Sub_Bit_Count);
    Address++;
    Sub_Bit_Count++;
    if(Sub_Bit_Count >=8)
    {
      Data_In[Item] = Working_Byte;
      Working_Byte = 0;
      Sub_Bit_Count=0;
      Byte_Count++;
      Item++;
    }
  }
  //If not a full byte of info
  if(Sub_Bit_Count != 0)
  {
      Data_In[Item] = Working_Byte;
      Working_Byte = 0;
      Sub_Bit_Count=0;
      Byte_Count++;
      Item++;
  }
  Data_In[2] = Byte_Count; 
  Send_Response(Data_In,Item);
};
