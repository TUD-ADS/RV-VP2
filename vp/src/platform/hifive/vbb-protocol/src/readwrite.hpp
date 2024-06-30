/*
 * magic.hpp
 *
 *  Created on: Dec 9, 2022
 *      Author: dwd
 */

#pragma once

#include "gpio-common.hpp"
#include <unistd.h>
#include <vector>

template<typename T>
bool writeStruct(int handle, T* s){
	return write(handle, s, sizeof(T)) == sizeof(T);
}
template<typename T>
bool readStruct(int handle, T* s){
	return read(handle, s, sizeof(T)) == sizeof(T);
}
// specialization for vector types
template<>
bool writeStruct(int handle, gpio::UART_Bytes* bytes);
template<>
bool readStruct(int handle, gpio::UART_Bytes* bytes);
