#pragma once
#include "protocol.hpp"
#include <fstream>

#include <functional>
#include <list>
#include "network_io.hpp"

class Responder {
public:
	struct AddressRange{
		hwitl::Address from;
		hwitl::Address to;
	};
static bool isAddressRangeValid(AddressRange& r){
    return r.from < r.to;
}
	typedef std::function<hwitl::Payload(hwitl::Address)> ReadCallback;
	typedef std::function<void(hwitl::Address,hwitl::Payload)> WriteCallback;
	struct CallbackEntry{
		AddressRange range;
		ReadCallback read;
		WriteCallback write;
	};
private:
	int m_handle;
	volatile bool irq_active;
	std::list<CallbackEntry> registeredRanges;
	CallbackEntry getRegisteredCallback(hwitl::Address);
public:
	Responder(int handle) : m_handle(handle), irq_active(false){};

	void addCallback(CallbackEntry);
	void listener();
	void setIRQ(bool active = true);
};
