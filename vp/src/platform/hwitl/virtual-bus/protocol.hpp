/*
 * protocol.hpp
 *
 *  Created on: Oct 19, 2022
 *      Author: dwd
 */

#pragma once

#include <inttypes.h>

namespace hwitl {
typedef uint32_t Address;
typedef uint32_t Payload;

struct __attribute__((packed)) Request {
	enum class Command : uint8_t {
		reset = 0,
		read = 1,
		write,
		getIRQ,
		setTime,
		exit
	};
private:
	Command m_command;
	static_assert(sizeof(Command) == 1, "command now also needs endianess conversion");
	Address m_address;	// this should always be network order
public:

	Request() = default;
	Request(const Command command, const Address address);
	static Request fromNetwork(const Command raw_command, const Address raw_address);
	Command getCommand() const;
	Address getAddressToHost() const;	// helper function for responder
};
static_assert(sizeof(Request) == sizeof(Request::Command) + sizeof(Address));

struct RequestRead {
	Request request;

	RequestRead() = default;
	RequestRead(const Address addr);
	static RequestRead fromNetwork(const Address network_addr);
};
static_assert(sizeof(RequestRead) == sizeof(Request));

struct __attribute__((packed)) RequestWrite {	// packed need inherited from request
	Request m_request;
	Payload m_payload;							// Always network order

	RequestWrite() = default;
	RequestWrite(const Address addr, const Payload payload);
	static RequestWrite fromNetwork(const Address network_addr, const Payload network_payload);
	Payload getPayload() const;
};
static_assert(sizeof(RequestWrite) == sizeof(Request) + sizeof(Payload));

struct __attribute__((packed)) ResponseStatus {
	/*
	 * packed struct starts from LSB to MSB
	 * Ack: bits 0 to 6
	 * irq_waiting: bit 7
	 */
	enum class Ack : uint8_t {
		never = 0,
		ok = 1,
		not_mapped,
		command_not_supported
	} ack : 7;
	bool irq_waiting : 1;

	ResponseStatus() =  default;
	ResponseStatus(const Ack status, const bool is_irq_waiting);
};
static_assert(sizeof(ResponseStatus) == 1);

struct __attribute__((packed)) ResponseRead {	// packed need inherited from ResponseStatus
private:
	ResponseStatus m_status;
	Payload m_payload;
public:
	ResponseRead() = default;
	ResponseRead(const ResponseStatus status, const Payload payload);
	ResponseStatus getStatus() const;
	Payload getPayload() const;
};
static_assert(sizeof(ResponseRead) == sizeof(ResponseStatus) + sizeof(Payload));

struct ResponseWrite {
	ResponseStatus status;
};
static_assert(sizeof(ResponseWrite) == sizeof(ResponseStatus));

}
