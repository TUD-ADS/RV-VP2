/*
 * gpio-server.hpp
 *
 *  Created on: 7 Nov 2018
 *      Author: dwd
 */

#pragma once

#include "gpio-common.hpp"

#include <functional>
#include <atomic>
#include <unordered_map>


class GpioServer : public GpioCommon {
public:
	typedef std::function<void(gpio::PinNumber pin, gpio::Tristate val)> OnChangeCallback;
	typedef std::function<void(gpio::UART_Bytes bytes)> OnUART_RX_Callback;
	//typedef std::function<gpio::UART_Bytes(void)> UART_TX_Function;

private:
	typedef int Socket;
	Socket listener_socket_fd;
	Socket control_channel_fd;
	Socket data_channel_fd;
	uint16_t data_channel_port;

	const char *base_port;
	std::atomic<bool> stop;
	OnChangeCallback onchange_fun;
	void handleConnection(Socket conn);

	// TODO: Performance testing. Better as static array?
	struct IOF_Channelinfo {
		gpio::IOF_Channel_ID id;
		gpio::Request::IOFunction requested_iof;		// Requested IO-Function to avoid protocol mismatch
	};
	std::unordered_map<gpio::PinNumber, IOF_Channelinfo> active_IOF_channels;
	std::unordered_map<gpio::PinNumber, OnUART_RX_Callback> active_UART_channels;

	gpio::IOF_Channel_ID findNewID();

	static void closeAndInvalidate(Socket& fd);
	void resetAllDatachannels();

	static int openSocket(const char* port);

	static int awaitConnection(int socket);

public:
	GpioServer();
	~GpioServer();
	bool setupConnection(const char* port);
	void quit();
	void registerOnChange(OnChangeCallback fun);
	void startAccepting();
	bool isConnected();

	void pushPin(gpio::PinNumber pin, gpio::Tristate state);
	// pin number should be the active CS
	gpio::SPI_Response pushSPI(gpio::PinNumber pin, gpio::SPI_Command byte);
	void registerUARTRX(gpio::PinNumber pin, OnUART_RX_Callback callback);
	void pushUART(gpio::PinNumber pin, gpio::UART_Bytes bytes);
};
