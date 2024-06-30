#pragma once
#include "protocol.hpp"
#include "network_io.hpp"
#include <fstream>
#include <optional>

class Initiator {
	int m_handle;
	bool is_irq_waiting;
public:
	Initiator(int& handle);

	std::optional<hwitl::ResponseRead> read(hwitl::Address pl);
	hwitl::ResponseStatus::Ack write(hwitl::Address address, hwitl::Payload pl);
	hwitl::ResponseStatus::Ack update();
	bool getInterrupt();
};
