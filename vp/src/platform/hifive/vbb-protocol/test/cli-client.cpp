/*
 * cli-client.cpp
 *
 *  Created on: 7 Nov 2018
 *      Author: dwd
 */

#include <unistd.h>
#include <iostream>
#include <functional>

#include "gpio-client.hpp"

using namespace std;
using namespace gpio;


int justPrint(GpioClient& gpio) {
	while (true) { //just update the view
		if (!gpio.update()) {
			cerr << "Error updating" << endl;
			return -1;
		}
		GpioCommon::printState(gpio.state);
		usleep(125000);
	}
	return 0;
}

int setPins(GpioClient& gpio) {
	// example actions
	for (uint8_t i = 0; i < 64; i++) {
		if (!gpio.setBit(i, Tristate::HIGH)) {
			cerr << "Error setting Bit " << i << endl;
			return -1;
		}
		if (!gpio.update()) {
			cerr << "Error updating" << endl;
			return -1;
		}
		GpioCommon::printState(gpio.state);
		usleep(750);
	}

	for (uint8_t i = 0; i < 64; i++) {
		if (!gpio.setBit(i, Tristate::LOW)) {
			cerr << "Error resetting Bit " << i << endl;
			return -1;
		}
		if (!gpio.update()) {
			cerr << "Error updating" << endl;
			return -1;
		}
		GpioCommon::printState(gpio.state);
		usleep(750);
	}
	return 0;
}

int registerForSPI(GpioClient& gpio) {

	if (!gpio.update()) {
		cerr << "Error updating" << endl;
		return -1;
	}

	PinNumber spi_pin;
	//looking for all available SPI pins
	for(spi_pin = 0; spi_pin < max_num_pins; spi_pin++){
		if(gpio.state.pins[spi_pin] == Pinstate::IOF_SPI) {
			if(gpio.registerSPIOnChange(spi_pin,
					[spi_pin](SPI_Command c){
						cout << "Pin " << +spi_pin << " got SPI command " << (int)c << endl; return c%4;
					}
				)){
				cout << "Registered SPI on Pin " << +spi_pin << endl;
			} else {
				cerr << "Could not register SPI onchange" << endl;
				return -1;
			}
		}
	}

	while(gpio.update()){
		usleep(1000000); // one second
		GpioCommon::printState(gpio.state);
	}
	return 0;
}

int uartPingPong(GpioClient& gpio) {
	if (!gpio.update()) {
		cerr << "Error updating" << endl;
		return -1;
	}

	PinNumber uart_rx;
	//looking for first available UART TX pin
	for(uart_rx = 0; uart_rx < max_num_pins; uart_rx++){
		if(gpio.state.pins[uart_rx] == Pinstate::IOF_UART_TX) {
			if(gpio.registerUARTOnChange(uart_rx,
					[uart_rx](gpio::UART_Bytes bytes){
						//cout << "Pin " << +uart_rx << " UART: ";
						for(const auto& byte : bytes)
							cout << byte << " (" << +byte << ")";
						cout << endl;
					}
				)){
				cout << "Registered UART receiver on Pin " << +uart_rx << endl;
				break;
			} else {
				cerr << "Could not register UART listener!" << endl;
				return -1;
			}
		}
	}
	if(uart_rx >= max_num_pins){
		cerr << "Could not find a UART_TX pin" << endl;
		return -2;
	}

	// finding first available UART RX port
	PinNumber uart_tx;
	for(uart_tx = 0; uart_tx  < max_num_pins; uart_tx++) {
		if(gpio.state.pins[uart_tx] == Pinstate::IOF_UART_RX) {
			cout << "Found SoC RX on pin " << +uart_tx << endl;
			break;
		}
	}
	if(uart_tx >= max_num_pins){
		cerr << "Could not find a UART_RX pin" << endl;
		return -2;
	}

	unsigned testnumber = 0;
	std::string basetext = "Hello from the other side ";
	char x[] = "X";
	while(gpio.update()){
		usleep(1000000); // one second
		GpioCommon::printState(gpio.state);
		// send uart burst
		auto testtext = basetext + std::to_string(testnumber++);
		if (testnumber > 9)
			testnumber = 0;

		gpio.sendUart(uart_tx, UART_Bytes{x, x + 1});	// test single
		gpio.sendUart(uart_tx, UART_Bytes{testtext.begin(), testtext.end()}); // test burst
	}

	return 0;
}

int main(int argc, char* argv[]) {
	if (argc < 3) {
		cout << "usage: " << argv[0] << " host port [testnr] (e.g. localhost 1339)" << endl;
		exit(-1);
	}

	GpioClient gpio;

	while(!gpio.setupConnection(argv[1], argv[2])) {
		cout << "connecting..." << endl;
		usleep(1000000);
	}
	cout << "connected." << endl;

	int test = 0;
	if (argc > 3)
		test = atoi(argv[3]);

	cout << "Running test nr " << test << endl;

	switch(test){
	case 0:
		return justPrint(gpio);
	case 1:
		return setPins(gpio);
	case 2:
		return registerForSPI(gpio);
	case 3:
		return uartPingPong(gpio);
	default:
		cerr << "Invalid test number given." << endl;
		return -1;
	}
}
