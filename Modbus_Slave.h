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
#ifndef Modbus_Slave_h
#define Modbus_Slave
#include "Arduino.h"
class ModBusSlave
{
	public:
		ModBusSlave(unsigned char _Slave, unsigned short * _Coils, unsigned short _Coil_Count);
		void Process_Data(unsigned char *Buffer, unsigned char Count);
		unsigned char Slave;
		unsigned short * Coils;
		unsigned char Error;
		unsigned short Coil_Count;
	private:
	void Send_Response(unsigned char *Data_In,unsigned short Length);
	void Read_Exception(unsigned char *Data_In);  // Function code 7
	void Write_Single_Reg(unsigned char *Data_In);  // Function code 1
	void Write_Single_Coil(unsigned char *Data_In);  // Function code 1
	void Write_Reg(unsigned char *Data_In);  // Function code 1
	void Write_Coils(unsigned char *Data_In);  // Function code 1
	void Read_Reg(unsigned char *Data_In);  // Function code 1
	void Read_Coils(unsigned char *Data_In);  // Function code 1
};
#endif
		
