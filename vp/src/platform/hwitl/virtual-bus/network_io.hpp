/*
 * magic.hpp
 *
 *  Created on: Oct 19, 2022
 *      Author: dwd
 */

#pragma once
#include <unistd.h>
//#include <iostream>

template<typename T>
bool writeStruct(int handle, T& pl) {
	//std::cout << "write of " << sizeof(T) << " bytes" << std::endl;
	unsigned total_bytes = 0;
	unsigned single = 0;
	char* raw = reinterpret_cast<char*>(&pl);
	while((single = write(handle, &raw[total_bytes], sizeof(T) - total_bytes)) > 0) {
		total_bytes += single;
		//std::cout << "Success writing " << total_bytes << " bytes" << std::endl;
		if(total_bytes == sizeof(T)) {
			return true;
		}
	}
	//std::cerr << "Incomplete write of " << total_bytes << " bytes with last return of " << single << std::endl;
	return false;
}

template<typename T>
bool readStruct(int handle, T& pl) {
	//std::cout << "Read of " << sizeof(T) << " bytes" << std::endl;
	unsigned total_bytes = 0;
	unsigned single = 0;
	char* raw = reinterpret_cast<char*>(&pl);
	while((single = read(handle, &raw[total_bytes], sizeof(T) - total_bytes)) > 0) {
		total_bytes += single;
		//std::cout << "read of " << single << " bytes (" << total_bytes << ") in total" << std::endl;
		if(total_bytes == sizeof(T)) {
			//std::cout << "Success reading " << total_bytes << " bytes" << std::endl;
			return true;
		}
	}
	//std::cerr << "Incomplete read of " << total_bytes << " bytes with last return of " << single << std::endl;
	return false;
}

bool setBaudrate(int handle, unsigned baudrate);

bool setTTYRawmode(int handle);
