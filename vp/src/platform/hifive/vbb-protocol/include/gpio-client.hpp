/*
 * gpio-client.hpp
 *
 *  Created on: 7 Nov 2018
 *      Author: dwd
 */

#pragma once

#include "gpio-common.hpp"

#include <functional>
#include <unordered_map>
#include <thread>
#include <mutex>
#include <iostream>

class GpioClient : public GpioCommon {
public:
	typedef std::function<gpio::SPI_Response(gpio::SPI_Command byte)> OnChange_SPI;
	typedef std::function<void(gpio::Tristate val)> OnChange_PIN;
	typedef std::function<void(gpio::UART_Bytes bytes)> OnChange_UART;

private:
	typedef int Socket;
	Socket control_channel;
	Socket data_channel;

	const char* currentHost;

	std::thread iof_dispatcher;

	struct IOFChannelDescription {
		gpio::Request::IOFunction iof;
		gpio::PinNumber pin;
		// This is somewhat memory wasteful as only one of the onchanges are used,
		// but we are not expecting a huge count of open channels.
		struct {
		OnChange_SPI spi;
		OnChange_PIN pin;
		OnChange_UART uart;
		} onchange;
	};
	std::unordered_map<gpio::IOF_Channel_ID, IOFChannelDescription> activeIOFs;
	std::mutex activeIOFs_m;

	static void closeAndInvalidate(Socket& fd);

	static Socket connectToHost(const char* host, const char* port);

	// Wrapper for server request
	gpio::Req_IOF_Response requestIOFchannel(gpio::PinNumber pin, gpio::Request::IOFunction iof_type);
	void notifyEndIOFchannel(gpio::PinNumber pin);

	// starts the data channel thread if necessary, and inserts given callback
	bool addIOFchannel(IOFChannelDescription desc);

	// Main IOF-Dispatcher
	void handleDataChannel();

public:
	GpioClient();
	~GpioClient();
	bool setupConnection(const char* host, const char* port);
	void destroyConnection();
	bool update();
	bool setBit(gpio::PinNumber pos, gpio::Tristate val);
	bool sendUart(gpio::PinNumber pos, std::vector<gpio::UART_Byte> bytes);

	// Intended to be used by the devices in the environment model
	bool registerSPIOnChange(gpio::PinNumber pin, OnChange_SPI fun, bool noResponse = false);
	bool registerUARTOnChange(gpio::PinNumber pin, OnChange_UART fun);
	bool registerPINOnChange(gpio::PinNumber pin, OnChange_PIN fun = +[](gpio::Tristate){});
	// TODO registerI2C...

	bool isIOFactive(gpio::PinNumber pin);
	void closeIOFunction(gpio::PinNumber pin);
};
