#include "responder.hpp"
#include <iostream>

#include <fcntl.h>	// File control definitions
#include <errno.h>	// errno
#include <string.h>	// strerror
#include <iomanip>	// setw and others
#include <thread>

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

Payload genericReadCallback(Address address) {
	static uint32_t b = 0;
	cout << "[responder-cli] [Callback] read " << HexPrint{address} << ": " << HexPrint{b} << endl;
	return b++;
}

void genericWriteCallback(Address address, Payload payload) {
	cout << "[responder-cli] [Callback] write " << HexPrint{address}<< " value " << HexPrint{payload} << endl;
}

void generateInterrupts(Responder& responder){
	bool on = true;
	while(true) {
		if(on)
			cout << "[responder-cli] triggered interrupt" << endl;
		responder.setIRQ(on);
		sleep(1);
		on = !on;
	}
}

int main(int argc, char* argv[]) {
	int handle = -1;
	if (argc > 1){
		handle = open(argv[1], O_RDWR| O_NOCTTY);
	}
	if (handle < 0) {
		cerr << "Handle could not be opened: " << strerror(errno) << endl;
		return -1;
	}

	Responder responder(handle);

	responder.addCallback(Responder::CallbackEntry{
			.range = Responder::AddressRange{.from = 0x0000, .to = 0xFFFFFFFF},
			.read = bind(genericReadCallback, placeholders::_1),
			.write = bind(genericWriteCallback, placeholders::_1, placeholders::_2)}
	);

	thread listener = thread(&Responder::listener, &responder);
	thread interruptor = thread([&responder](){generateInterrupts(responder);});

	if(listener.joinable())
		listener.join();

	// don't care for interruptor thread

	cerr << "end" << endl;
	return 0;
}
