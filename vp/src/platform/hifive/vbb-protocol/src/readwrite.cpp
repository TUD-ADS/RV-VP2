/*
 * magic.hpp
 *
 *  Created on: Dec 9, 2022
 *      Author: dwd
 */
#include "readwrite.hpp"

// Warn: This may overflow on very big UART_Bytes (that hopefully are never used through the connection)

template<>
bool writeStruct(int handle, gpio::UART_Bytes* bytes) {
	return write(handle, bytes->data(), bytes->size()) == static_cast<ssize_t>(bytes->size());
}
template<>
bool readStruct(int handle, gpio::UART_Bytes* bytes) {
	return read(handle, bytes->data(), bytes->size()) == static_cast<ssize_t>(bytes->size());
}
