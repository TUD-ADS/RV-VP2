#include "initiator.hpp"
#include <iostream>
#include <fcntl.h>	// File control definitions
#include <string.h>	// strerror
#include <iomanip>	// setw()

using namespace std;
using namespace hwitl;

template<typename T>
struct HexPrint {
	HexPrint(T a) : a(a){};
	T a;
};

template<typename T>
std::ostream& operator<<( std::ostream& o, const HexPrint<T>& a ){
	const auto flags = o.flags();
	o << hex;
	o << "0x";
	o << setfill('.');
	o << setw( sizeof(T) * 2);
	o << uppercase;
	o << a.a;
	o.setf(flags);
	return o;
}

void readWriteAtZero(Initiator& remote) {
	constexpr Address address = 0x00000000;
	constexpr Payload payload = 0xFF;

	cout << "[Initiator-cli] write " << HexPrint{address} << " value " << payload << endl;
	auto stat = remote.write(address, payload);
	if(stat != ResponseStatus::Ack::ok) {
		cerr << "[Initiator-cli] Nak on write: " << static_cast<unsigned>(stat) << endl;
		return;
	}
	cout << "[Initiator-cli] read " << HexPrint{address} << " ";
	const auto ret = remote.read(address);
	if(!ret) {
		cerr << "[Initiator-cli] Could not read from remote" << endl;
		return;
	}
	if(ret->getStatus().ack != ResponseStatus::Ack::ok) {
		cerr << "[Initiator-cli] Nak on read: "  << static_cast<unsigned>(ret->getStatus().ack) << endl;
		return;
	}
	cout << "[Initiator-cli] value " << HexPrint{ret->getPayload()} << endl;
	if(remote.getInterrupt())
		cout << "[Initiator-cli] Interrupt was triggered" << endl;
}

void sweepAddressRoom(Initiator& remote) {
	constexpr Address startAddr = 0x50000000;
	constexpr Address   endAddr = 0x50002FFF;
	constexpr Payload payload   = 0x8BADF00D;
	constexpr Address printStep = 0x00001000;
	for(Address address = startAddr; address <= endAddr; address += sizeof(Payload)) {
		if(address % printStep == 0) {
			cout << "[Initiator-cli] Scanning " << HexPrint{address} << " - " << HexPrint{(address + printStep)} << " ..." << endl;
		}
		auto ret = remote.read(address);
		if(!ret) {
			cerr << "[Initiator-cli] Could not read from remote" << endl;
			return;
		}
		const auto status = ret->getStatus().ack;
		if(status == ResponseStatus::Ack::not_mapped) {
			// nothing
		} else if(status == ResponseStatus::Ack::ok) {
			cout << "[Initiator-cli] \t" << "Found Register at " << HexPrint{address} << " : " << HexPrint{ret->getPayload()} << endl;
			if(remote.write(address, payload) == ResponseStatus::Ack::ok) {
				cout << "[Initiator-cli] \t\t" << "Register Writable: " << HexPrint{payload} << endl;
			}
		} else {
			cout << "[Initiator-cli] Other error at " << HexPrint{address} << ": [" << static_cast<int>(status) << "]" << endl;
		}
	}
}

void printUsage(const char* b) {
	cout << "Usage: " << b << " path_to_file [--test num] [--baudrate num] [--help]" << endl;
}

int main(int argc, char* argv[]) {
	int handle = -1;

	// mandatory argument
	if (argc > 1){
		if(argv[1][0] == '-') {
			// indicates some argument
			printUsage(argv[0]);
			return 0;
		}
		handle = open(argv[1], O_RDWR| O_NOCTTY);
	} else {
		printUsage(argv[0]);
		return -1;
	}

	if (handle < 0) {
		cerr << "[Initiator-cli] : " << argv[1] << " : " << strerror(errno) << endl;
		return -1;
	}

	// optional arguments
	if (argc > 2) {
		for(int i = 2; i < argc; i++) {
			if(strcmp(argv[i], "--baudrate") == 0) {
				if(argc <= i+1) {
					cerr << "Baudrate needs an argument!" << endl;
					printUsage(argv[0]);
					return -1;
				} else {
					if(!setBaudrate(handle, atoi(argv[i+1]))) {
						return -2;
					}
					i++; // Argument consumed
				}
			} else if(strcmp(argv[i], "--help") == 0) {
				printUsage(argv[0]);
				return 0;
			} else {
				cerr << "Unknown argument " << argv[i] << endl;
			}
			// TODO: Different tests
		}
	}

	setTTYRawmode(handle);

	Initiator initiator(handle);

	//while(handle) {
		// different modes
		sweepAddressRoom(initiator);
	//}

	close(handle);
	cerr << "[Initiator-cli] end" << endl;
	return 0;
}
