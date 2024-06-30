/*
 * gpio.hpp
 *
 *  Created on: 7 Nov 2018
 *      Author: dwd
 */

#pragma once

#include <inttypes.h>
#include <stddef.h>
#include <limits>       // std::numeric_limits
#include <vector>

void hexPrint(unsigned char* buf, size_t size);
void bitPrint(unsigned char* buf, size_t size);

namespace gpio {

	// TODO: Remove Redundancies and make Pinstate a struct
	// State of Server pins
	enum class Pinstate : uint8_t {
		UNSET = 0,
		LOW,
		HIGH,
		// TODO: Maybe _weak for pullups/downs

		// TODO: Explicitly state which function, e.g. UART_RX
		START_OF_IOFs = 4,
		IOF_SPI = START_OF_IOFs,
		IOF_I2C,	// not yet used
		IOF_PWM,	// planned to be used
		IOF_UART_RX,// Seen from SoC
		IOF_UART_TX,// Seen from SoC
	};

	enum class Tristate : uint8_t {
		UNSET = 0,
		LOW,
		HIGH,
		//TODO: Maybe _weak for pullups/downs
	};

	static constexpr Pinstate toPinstate (const Tristate from) {
		// this is safe because first 2 bits are identical;
		return static_cast<Pinstate>(from);
	}
	static constexpr Tristate toTristate (const Pinstate from) {
		// this is only safe if a !isIof(from) was done
		return static_cast<Tristate>(from);
	}

	bool isIOF(const Pinstate s);

	constexpr unsigned default_port = 1400;

	typedef uint8_t PinNumber;
	constexpr PinNumber max_num_pins = 64;
	static_assert(std::numeric_limits<PinNumber>::max() >= max_num_pins,
			"max_num_pins do not fit into PinNumber type");

	typedef uint8_t IOF_Channel_ID;
	typedef uint8_t SPI_Command;
	typedef uint8_t SPI_Response;
	typedef uint8_t UART_Byte;
	typedef std::vector<gpio::UART_Byte> UART_Bytes;

	struct State {
		//TODO somehow packed?
		union {
			gpio::Pinstate pins[max_num_pins];
			uint64_t port[(sizeof(gpio::Pinstate) * sizeof(pins) + 1) / sizeof(uint64_t)];
		};
		static_assert(sizeof(gpio::Pinstate) * sizeof(pins) != sizeof(port) * sizeof(uint64_t),
				"Warning: Convenience-function 'port' is causing State to be bigger than necessary");
	};

	struct Request {
		enum class IOFunction: uint8_t {
			SPI = 0,
			SPI_NORESPONSE,
			I2C,
			PWM,
			UART_RX,	// as seen from initiator (Client)
			BITSYNC,
		};

		enum class Type : uint8_t {
			GET_BANK = 1,
			SET_BIT,
			UART_TRANSMIT_SINGLE,		// FixMe: Perhaps better with second channel?
			UART_TRANSMIT_BURST,
			REQ_IOF,
			END_IOF,
			REQ_START_SIM,
			REQ_END_SIM
		} op;
		union {
			struct {
				PinNumber pin;
				gpio::Tristate val;
			} setBit;

			struct {
				// Todo: Decide how to determine SPI's Chip Select
				// Perhaps pin shall be one of the hardware CS pins
				PinNumber pin;
				IOFunction iof; // request a specific IO-function (in advance).
			} reqIOF;

			struct {	// client to server
				PinNumber pin;
				uint8_t byte;
			} uartSingle;

			struct {	// client to server
				PinNumber pin;
				uint8_t num_bytes;
			} uartBurst;
		};
	};

	struct Req_IOF_Response {
		static constexpr uint16_t invalid_port_below = 1024;
		uint16_t port = 0;	// below 'invalid_port_below' is error condition
		IOF_Channel_ID id = 0;
	};

	struct IOF_Update {		// Server to Client
		IOF_Channel_ID id;
		union {
			SPI_Command spi;
			UART_Byte uart;
			Tristate pin;
		} payload;
	};
};

struct GpioCommon {

	static void printRequest(const gpio::Request& req);
	static void printPinstate(const gpio::Pinstate& state);
	static void printState(const gpio::State& state);

	gpio::State state;

	GpioCommon();
};
