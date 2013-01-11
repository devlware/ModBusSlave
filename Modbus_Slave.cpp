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

    Modified 27 December 2012 by Diegowa
*/

#include "Arduino.h"
#include "Modbus_Slave.h"
#include <CRC16.h>

CRC CheckSum;   // From Checksum Library, CRC15.h, CRC16.cpp

//################## ModBusSlave ###################
// Takes:   Slave Address, Pointer to Registers and Number of Available Registers
// Returns: Nothing
// Effect:  Initializes Library
ModBusSlave::ModBusSlave(const uint8_t slave, uint16_t *coils, const uint16_t coil_count)
{
    _slave = slave;
    _coils = coils;
    _error = Error_No_Error;
    _coil_count = coil_count;
}

void ModBusSlave::setSlaveId(const uint8_t s)
{
    if (s > 0 && s < 16) {
        _slave = s;
    }   
}
/* 
 * configure(slave, baud, parity, txenpin)
 *
 * sets the communication parameters for of the serial line.
 *
 * slave: identification number of the slave in the Modbus network (1 to 127)
 * baud: baudrate in bps (typical values 9600, 19200... 115200)
 * parity: a single character sets the parity mode (character frame format): 
 *         'n' no parity (8N1); 'e' even parity (8E1), 'o' for odd parity (8O1).
 * txenpin: arduino pin number that controls transmision/reception
 *        of an external half-duplex device (e.g. a RS485 interface chip).
 *        0 or 1 disables this function (for a two-device network)
 *        >2 for point-to-multipoint topology (e.g. several arduinos)
 */
void ModBusSlave::Configure(const long baud_rate, const int8_t parity, const uint8_t txenpin)
{
    this->_txenpin = txenpin;

    switch (parity) {
        case 'e': // 8E1
            UCSR0C |= ((1 << UPM01) | (1 << UCSZ01) | (1 << UCSZ00));
            //      UCSR0C &= ~((1<<UPM00) | (1<<UCSZ02) | (1<<USBS0));
            break;
        case 'o': // 8O1
            UCSR0C |= ((1 << UPM01) | (1 << UPM00) | (1 << UCSZ01) | (1<<UCSZ00));
            //      UCSR0C &= ~((1<<UCSZ02) | (1<<USBS0));
            break;
        case 'n': // 8N1
            UCSR0C |= ((1 << UCSZ01) | (1 << UCSZ00));
            //      UCSR0C &= ~((1<<UPM01) | (1<<UPM00) | (1<<UCSZ02) | (1<<USBS0));
            break;
        default:
            break;
    }

    if (txenpin > 1) { // pin 0 & pin 1 are reserved for RX/TX
        pinMode(txenpin, OUTPUT);
        digitalWrite(txenpin, LOW);
    }

    Serial.end();
    delay(500);
    Serial.begin(baud_rate);  // Open Serial port at Defined Baudrate
}

//################## Process Data ###################
// Takes:   Data stream buffer from serial port, number of characters to read
// Returns: Nothing
// Effect:  Reads in and parses data

void ModBusSlave::Process_Data(uint8_t *Buffer, uint8_t Count)
{
    Count--;                                    // Convert Byte count to byte index;
    uint8_t Slave_Address = Buffer[0];          // Grab Slave Address out of buffer

    if(Slave_Address > 247)                     // If Address greater than 247 => Invalid
    {
        _error = Error_Invalid_Slave_Address;
        return;
    }
    
    if(Slave_Address != 0)                      // Slave = 0 for Broadcast Msg
    {
      if(_slave != Slave_Address)                // If Msg is not for this Slave
      {
        return;
      }
    }
    uint8_t Function = Buffer[1];                                 // Grab Function Code
    uint16_t CRC = ((Buffer[Count]<<8)|Buffer[Count-1]);          // Transmitted CRC
    uint16_t Recalculated_CRC = CheckSum.CRC16(Buffer,Count-1);   // Computer CRC

    if(Recalculated_CRC != CRC)                                         // Compare CRC, if not equal, then...
    {
        _error = Error_Invalid_CRC;
        return;
    }
    if(Function == 1)                                                   //Read Coils
    {
        Read_Coils(Buffer);
    }
    if(Function == 2)                                                   //Read Descrete Input
    {
        Read_Coils(Buffer);
    }
    if(Function == 3)                                                   //Read Reg
    {
        Read_Reg(Buffer);
    }
    if(Function == 4)                                                   //Read Input Reg
    {
        Read_Reg(Buffer);
    }
    if(Function == 5)                                                   //Write Single Coil
    {
      Write_Single_Coil(Buffer);
    }
    if(Function == 6)                                                   //Write Single Reg
    {
      Write_Single_Reg(Buffer);
    }
    if(Function == 7)                                                   //Read Exception Status
    {
        Read_Exception(Buffer);
    }
    if(Function == 15)                                                  //Write Coils
    {
      Write_Coils(Buffer);
    }
    if(Function == 16)                                                  //Write Reg
    {
      Write_Reg(Buffer);
    }

    _error = Error_No_Error;                                            //We made it to the end, Set _error to 0
}

//################## Read Exception ###################
// Takes:   In Data Buffer
// Returns: Nothing
// Effect:  Sets Reply Data and sends Response
void ModBusSlave::Read_Exception(uint8_t *Data_In)
{
    Data_In[2] = _error;
    Send_Response(Data_In,3);
    _error = Error_No_Error;
}
//################## Send Response ###################
// Takes:   In Data Buffer, Length
// Returns: Nothing
// Effect:  Sends Response over serial port
void ModBusSlave::Send_Response(uint8_t *Data_In, uint16_t Length)
{
    if (Data_In[0] == 0)                                        // If Broadcast Msg, then no reply  
        return;

    if (this->_txenpin > 1) {                                   // set MAX485 to speak mode 
        UCSR0A = UCSR0A | (1 << TXC0);
        digitalWrite(this->_txenpin, HIGH);
        delay(1);
    }

    if (Length > 0) {                                           // If there is Data to be sent, then...
        uint16_t MyCRC = CheckSum.CRC16(Data_In,Length);      // Calculate new CRC
        Data_In[Length++] = MyCRC & 0x00ff;                         // Load lower byte into Buffer
        Data_In[Length++] = MyCRC >> 8;                             // Load upper byte into Buffer

        for(int C = 0; C < Length; C++) {                           // Write Data
            Serial.write(Data_In[C]);
        }
    }

    if (this->_txenpin > 1) {                                   // set MAX485 to listen mode 
        while (!(UCSR0A & (1 << TXC0)));
        digitalWrite(this->_txenpin, LOW);
    }
}

//################## Write_Single_Reg ###################
// Takes:   In Data Buffer
// Returns: Nothing
// Effect:  Sets Reply Data and Register values, sends Response for Write Single Reg.
void ModBusSlave::Write_Single_Reg(uint8_t *Data_In)  // Function code 1
{
    uint16_t Addr_Hi = Data_In[2];
    uint16_t Addr_Lo = Data_In[3];
    uint16_t Value_Hi = Data_In[4];
    uint16_t Value_Lo = Data_In[5];

    uint16_t Address = (Addr_Lo + (Addr_Hi<<8));
    if(Address >= _coil_count)                               // Invalid Address;
    {
        _error = Error_Invalid_Register_Address;
        return;
    }
    _coils[Address] = (Value_Hi<<8) + Value_Lo;
    Send_Response(Data_In,6);
}

//################## Write_Single_Coil ###################
// Takes:   In Data Buffer
// Returns: Nothing
// Effect:  Writes single bit in Registers.  Sets Reply Data and sends Response
void ModBusSlave::Write_Single_Coil(uint8_t *Data_In)  // Function code 1
{
    uint16_t Addr_Hi = Data_In[2];
    uint16_t Addr_Lo = Data_In[3];
    uint16_t Value_Hi = Data_In[4];
    uint16_t Value_Lo = Data_In[5];
    uint16_t Address = Addr_Lo + (Addr_Hi<<8);  
    uint16_t Write_Address = Address/16;
    uint8_t Write_Bit = Address&0x000F;

    if(Address >= _coil_count*16)            // Invalid Address;
    {
        _error = Error_Invalid_Register_Address;
        return;
    }
    if(Value_Hi>0 | Value_Lo > 0)               // Real Protocol requires 0xFF00 = On and 0x0000 = Off, Custom, using anything other than 0 => ON
    {
        _coils[Write_Address] |= (1<<Write_Bit);
    }
    else
    {
        _coils[Write_Address] &= ~(1<<Write_Bit);
    }

    Send_Response(Data_In,6);
}

//################## Write_Reg ###################
// Takes:   In Data Buffer
// Returns: Nothing
// Effect:  Writes Register in Registers.  Sets Reply Data and sends Response
void ModBusSlave::Write_Reg(uint8_t *Data_In)  // Function code 1
{
    uint16_t Addr_Hi = Data_In[2];
    uint16_t Addr_Lo = Data_In[3];
    uint16_t Cnt_Hi = Data_In[4];
    uint16_t Cnt_Lo = Data_In[5];
    uint8_t Byte_Count = Data_In[6];
    uint16_t Address = (Addr_Lo + (Addr_Hi<<8));

    if (Address >= _coil_count)   // Invalid Address;
    {
        _error = Error_Invalid_Register_Address;
        return;
    }

    uint16_t Read_Byte = 7;  // First entry in input Data_In
    for (int C = 0; C < Byte_Count; C+= 2)
    {
        _coils[Address] = (Data_In[Read_Byte] << 8) + Data_In[Read_Byte + 1];
        Address += 1;
        Read_Byte += 2;
    }

    Send_Response(Data_In,6);
}

//################## Write_Coils ###################
// Takes:   In Data Buffer
// Returns: Nothing
// Effect:  Writes multiple bits in Registers.  Sets Reply Data and sends Response
void ModBusSlave::Write_Coils(uint8_t *Data_In)  // Function code 1
{
    uint16_t Addr_Hi = Data_In[2];
    uint16_t Addr_Lo = Data_In[3];
    uint16_t Cnt_Hi = Data_In[4];
    uint16_t Cnt_Lo = Data_In[5];
    uint8_t Byte_Count = Data_In[6];

    uint16_t Address = Addr_Lo + (Addr_Hi<<8);
    uint16_t Write_Bit_Count = Cnt_Lo + (Cnt_Hi<<8);

    if(Address >= _coil_count*16)    // Invalid Address;
    {
       _error = Error_Invalid_Register_Address;
        return;
    }

    uint16_t Write_Address = Address/16;
    uint8_t Write_Bit = Address&0x000F;

    uint16_t Read_Byte = 7; // First entry in input Data_In
    uint8_t Read_Bit = 0;
    for(int C = 0; C < Write_Bit_Count;C++)
    {
        if((Data_In[Read_Byte]&(1<<Read_Bit))>0)        // If set Bit is a 1, then, set corrisponding bit in register
        {
            _coils[Write_Address] |= (1<<Write_Bit);
        }
        else                                            // If set bit is a 0, clear the bit in the register
        {
            _coils[Write_Address] &= ~(1<<Write_Bit);
        }
        Read_Bit++;                                     // Increment the Read Bit
        Write_Bit++;                                    // Increment the write bit  

        if(Read_Bit>=8)                                 // Reads are from bytes of data, so increment every 8th bit
        {
            Read_Bit = 0;
            Read_Byte++;
        }

        if(Write_Bit >= 16)                             // Write are to 16bit registers, so increment every 16th bit.   
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
void ModBusSlave::Read_Reg(uint8_t *Data_In)  // Function code 1
{
    uint16_t Addr_Hi = Data_In[2];
    uint16_t Addr_Lo = Data_In[3];
    uint16_t Cnt_Hi = Data_In[4];
    uint16_t Cnt_Lo = Data_In[5];
    //Read date
    uint16_t Byte_Count = Cnt_Lo + (Cnt_Hi<<8);
    Data_In[2] = Byte_Count * 2;

    uint16_t Address = (Addr_Lo + (Addr_Hi<<8));

    if(Address >= _coil_count)   // Invalid Address;
    {
       _error = Error_Invalid_Register_Address;
        return;
    }

    uint16_t Item = 3;
    for(int Count = 0; Count < Byte_Count; Count++)
    {
        Data_In[Item+1] = lowByte(_coils[Address]);
        Data_In[Item] = highByte(_coils[Address]);
        Address++;
        Item += 2;
    }

    Send_Response(Data_In,Item);
}

//################## Read _coils ###################
// Takes:   In Data Buffer
// Returns: Nothing
// Effect:  Reads a bit at a time, composing 8 bit replies.  Sets Reply Data and sends Response
void ModBusSlave::Read_Coils(uint8_t *Data_In)  // Function code 1
{
    uint16_t Addr_Hi = Data_In[2];
    uint16_t Addr_Lo = Data_In[3];
    uint16_t Cnt_Hi = Data_In[4];
    uint16_t Cnt_Lo = Data_In[5];
    //Read date
    uint16_t Bit_Count = Cnt_Lo + (Cnt_Hi<<8);
    uint8_t Byte_Count = 0;
    uint8_t Sub_Bit_Count = 0;
    uint8_t Working_Byte = 0;
    uint16_t Address = Addr_Lo + (Addr_Hi<<8);

    if(Address >= _coil_count*16)    // Invalid Address;
    {
       _error = Error_Invalid_Register_Address;
        return;
    }

    uint16_t Item = 3;
    for (int Bit = 0; Bit < Bit_Count; Bit++)
    {
        Working_Byte = Working_Byte | ((_coils[Address/16]>>(Address&0x000f))<<Sub_Bit_Count);
        Address++;
        Sub_Bit_Count++;

        if (Sub_Bit_Count >=8) {
            Data_In[Item] = Working_Byte;
            Working_Byte = 0;
            Sub_Bit_Count=0;
            Byte_Count++;
            Item++;
        }
    }
    //If not a full byte of info
    if (Sub_Bit_Count != 0) {
        Data_In[Item] = Working_Byte;
        Working_Byte = 0;
        Sub_Bit_Count=0;
        Byte_Count++;
        Item++;
    }
    Data_In[2] = Byte_Count;

    Send_Response(Data_In,Item);
};
