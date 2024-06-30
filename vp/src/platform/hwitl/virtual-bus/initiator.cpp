#include "initiator.hpp"
#include <iostream>
#include <string.h> // strerror

using namespace std;
using namespace hwitl;

Initiator::Initiator(int& handle) : m_handle(handle), is_irq_waiting(false) {
	//if(!clearInputBuffer(m_handle))
	//	throw std::runtime_error("[Initiator read] Could not clear input buffer");

	auto command = Request::Command::reset;
	if(!writeStruct(m_handle, command)) {
		throw std::runtime_error("[Initiator read] Error transmitting initial reset request");
	}
	// no response expected
};

std::optional<ResponseRead> Initiator::read(Address address) {
	RequestRead req (address);
	if(!writeStruct(m_handle, req)) {
		cerr << "[Initiator read] Error transmitting read request" << endl;
		return std::nullopt;
	}

	ResponseRead ret;
	if(!readStruct(m_handle, ret)) {
		cerr << "[Initiator read] Error reading read response:";
		cerr << strerror(errno) << endl;
		return std::nullopt;
	}

	if(ret.getStatus().ack == ResponseStatus::Ack::ok)
		is_irq_waiting = ret.getStatus().irq_waiting;
	return ret;
}
ResponseStatus::Ack Initiator::write(Address address, Payload pl) {
	RequestWrite req(address, pl);
	if(!writeStruct(m_handle, req)) {
		cerr << "[Initiator write] Error transmitting write request" << endl;
		return ResponseStatus::Ack::never;
	}
	ResponseWrite res;
	if(!readStruct(m_handle, res)) {
		cerr << "[Initiator write] Error reading response" << endl;
		return ResponseStatus::Ack::never;
	}
	if(res.status.ack == ResponseStatus::Ack::ok)
		is_irq_waiting = res.status.irq_waiting;
	return res.status.ack;
}

ResponseStatus::Ack Initiator::update() {
	auto req = Request::Command::getIRQ;
	if(!writeStruct(m_handle, req))
		throw std::runtime_error("[Initiator read] Error transmitting update request");
	ResponseStatus res;
	if(!readStruct(m_handle, res))
		throw std::runtime_error("[Initiator read] Error reading update response");
	if(res.ack == ResponseStatus::Ack::ok)
		is_irq_waiting = res.irq_waiting;
	return res.ack;
}

bool Initiator::getInterrupt() {
	if(!is_irq_waiting) {
		update();	// is getIRQ necessary?
	}
	return is_irq_waiting;
}
