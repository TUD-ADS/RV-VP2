/*
 * gpio-server.cpp
 *
 *  Created on: 5 Nov 2018
 *      Author: dwd
 */

#include <unistd.h>
#include <csignal>
#include <functional>
#include <iostream>
#include <thread>

#include "gpio-server.hpp"

using namespace std;
using namespace gpio;

bool stop = false;

void signalHandler(int signum) {
	cout << "Interrupt signal (" << signum << ") received.\n";

	if (stop)
		exit(signum);
	stop = true;
	raise(SIGUSR1);  // this breaks wait in thread
}

void onChangeCallback(GpioServer* gpio, PinNumber pin, Tristate val) {
	gpio->state.pins[pin] = toPinstate(val);
	printf("Bit %d changed to %d\n", pin, gpio->state.pins[pin] == Pinstate::HIGH ? 1 : 0);
}

int main(int argc, char* argv[]) {
	if (argc < 2) {
		cout << "usage: " << argv[0] << " port (e.g. 1339)" << endl;
		exit(-1);
	}

	GpioServer gpio;

	if (!gpio.setupConnection(argv[1])) {
		cerr << "cant set up server" << endl;
		exit(-1);
	}

	signal(SIGINT, signalHandler);

	gpio.registerOnChange(bind(onChangeCallback, &gpio, placeholders::_1, placeholders::_2));
	thread server(bind(&GpioServer::startAccepting, &gpio));

	const PinNumber spi_cs = 0;
	gpio.state.pins[spi_cs] = Pinstate::IOF_SPI;
	gpio.state.pins[1] = Pinstate::IOF_PWM;



	const PinNumber uart_tx = 2;
	const PinNumber uart_rx = 3;
	gpio.state.pins[uart_tx] = Pinstate::IOF_UART_TX;
	gpio.state.pins[uart_rx] = Pinstate::IOF_UART_RX;
	gpio.registerUARTRX(uart_rx, [uart_rx,uart_tx,&gpio](gpio::UART_Bytes bytes){
		cout << "UartRX pin " << +uart_rx << ": ";
		for(const auto& byte : bytes)
			cout << byte;
		cout << endl;

		std::string answ{"And gondor will answer"};
		answ += std::to_string(bytes.back());
		gpio.pushUART(uart_tx, UART_Bytes{answ.begin(), answ.end()});
	});


	SPI_Command sumbyte = 0;
	while (!stop) {
		// some example actions
		usleep(500000);

		cout << " SPI Command " << +sumbyte << " returned " << +gpio.pushSPI(spi_cs, sumbyte) << endl;
		sumbyte++;

		// cycle through states in pin 11
		auto pin = reinterpret_cast<uint8_t*>(&gpio.state.pins[11]);
		(*pin)++;
		if(*pin > 6)
			*pin = 0;

		cout << " SPI Command " << +sumbyte << " returned ";
		cout << +gpio.pushSPI(spi_cs, sumbyte++) << endl;
	}
	gpio.quit();
	server.join();
}
