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

#ifndef MODBUS_SLAVE_H
#define MODBUS_SLAVE_H

#include "Arduino.h"

enum Error_Types_e {
    Error_No_Error = 0,
    Error_Invalid_Slave_Address,
    Error_Invalid_CRC,
    Error_Invalid_Register_Address,
    Error_Kill_Timeout = 255
};

class ModBusSlave
{
    public:
        uint8_t _slave;
        uint8_t _error;
        uint16_t *_coils;
        uint16_t _coil_count;

        ModBusSlave(const uint8_t slave, uint16_t *coils, const uint16_t coil_count);
        void Process_Data(uint8_t *Buffer, uint8_t count);
        void Configure(const long baud_rate, const int8_t parity, const uint8_t txenpin);
        void setSlaveId(const uint8_t s);
    private:
        uint8_t _txenpin;

        void Send_Response(uint8_t *Data_In, uint16_t Length);
        void Read_Exception(uint8_t *Data_In);  // Function code 7
        void Write_Single_Reg(uint8_t *Data_In);  // Function code 1
        void Write_Single_Coil(uint8_t *Data_In);  // Function code 1
        void Write_Reg(uint8_t *Data_In);  // Function code 1
        void Write_Coils(uint8_t *Data_In);  // Function code 1
        void Read_Reg(uint8_t *Data_In);  // Function code 1
        void Read_Coils(uint8_t *Data_In);  // Function code 1
};

#endif
