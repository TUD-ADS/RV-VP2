/*
 * gpio-server.cpp
 *
 *  Created on: 5 Nov 2018
 *      Author: dwd
 */

#include "gpio-server.hpp"
#include "gpio-client.hpp" // for force quit of listening switch
#include <arpa/inet.h>
#include <errno.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>	// for TCP_NODELAY
#include <readwrite.hpp>
#include <set>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <iostream>
#include <thread>

#define ENABLE_DEBUG (0)
#include "debug.h"

using namespace std;
using namespace gpio;

// get sockaddr, IPv4 or IPv6:
/*
static void *get_in_addr(struct sockaddr *sa) {
	if (sa->sa_family == AF_INET) {
		return &(((struct sockaddr_in *)sa)->sin_addr);
	}

	return &(((struct sockaddr_in6 *)sa)->sin6_addr);
}
*/

GpioServer::GpioServer() :
		listener_socket_fd(-1), control_channel_fd(-1), data_channel_fd(-1),
		data_channel_port(0),
		base_port(""), stop(false), onchange_fun(nullptr){}

GpioServer::~GpioServer() {
	if (listener_socket_fd >= 0) {
		DEBUG("closing gpio-server socket: %d\n", listener_socket_fd);
		closeAndInvalidate(listener_socket_fd);
	}

	closeAndInvalidate(control_channel_fd);
	closeAndInvalidate(data_channel_fd);
}

IOF_Channel_ID GpioServer::findNewID() {
	if(active_IOF_channels.size() >= std::numeric_limits<IOF_Channel_ID>::max()) {
		cerr << "[GPIO-Server] Too many channels in use (" << active_IOF_channels.size() << "), can not generate new one" << endl;
		return std::numeric_limits<IOF_Channel_ID>::max();
	}

	// copy IDs into set (-> sorted) for faster search
	set<IOF_Channel_ID> used_ids;
	for(const auto& [pin, channelinfo] : active_IOF_channels) {
		used_ids.insert(channelinfo.id);
	}

	IOF_Channel_ID unused_id = 0;
	while(used_ids.find(unused_id) != used_ids.end()) {
		unused_id++;
	}

	return unused_id;
}

void GpioServer::closeAndInvalidate(Socket& fd) {
	close(fd);
	fd = -1;
}

void GpioServer::resetAllDatachannels(){
	closeAndInvalidate(data_channel_fd);
	active_IOF_channels.clear();
	active_UART_channels.clear();
}

int GpioServer::openSocket(const char *port) {
	int new_fd = -1;

	struct addrinfo hints, *servinfo, *p;
	const int reuse_addr = 1;
	const int disable_nagle_buffering = 1;
	int rv;

	memset(&hints, 0, sizeof hints);
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_flags = AI_PASSIVE;	// use my IP

	if ((rv = getaddrinfo(NULL, port, &hints, &servinfo)) != 0) {
		fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
		return -1;
	}

	// loop through all the results and bind to the first we can
	bool found = false;
	for (p = servinfo; p != NULL; p = p->ai_next) {
		if ((new_fd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1) {
			cerr << "[gpio-server] opening of socket unsuccessful " << strerror(errno) << endl;
			continue;
		}

		if (setsockopt(new_fd, SOL_SOCKET, SO_REUSEADDR, &reuse_addr, sizeof(int)) == -1) {
			close(new_fd);
			cerr << "[gpio-server] setsockopt unsuccessful " << strerror(errno) << endl;
			continue;
		}

		if (setsockopt(new_fd, IPPROTO_TCP, TCP_NODELAY, &disable_nagle_buffering, sizeof(int)) == -1){
			cerr << "[gpio-server] WARN: setup TCP_NODELAY unsuccessful " << strerror(errno) << endl;
		}

		if (::bind(new_fd, p->ai_addr, p->ai_addrlen) == -1) {
			close(new_fd);
			cerr << "[gpio-server] could not bind to addr " << p->ai_addr->sa_data << endl;
			continue;
		}

		found = true;
		break;
	}

	if(!found) {
		return -1;
	}

	freeaddrinfo(servinfo);  // all done with this structure

	if (p == NULL) {
		fprintf(stderr, "gpio-server: failed to bind\n");
		return -3;
	}

	if (listen(new_fd, 1) < 0) {
		cerr << "[gpio-server] Could not start listening on new socket " << new_fd << " ";
		close(new_fd);
		return -4;
	}

	return new_fd;
}

bool GpioServer::setupConnection(const char *port) {
	if (!(this->base_port = strdup(port))) {
		perror("[gpio-server] strdup");
		return false;
	}

	if((listener_socket_fd = openSocket(base_port)) < 0) {
		cerr << "[gpio-server] Could not setup control channel" << endl;
		return false;
	}

	return true;
}

void GpioServer::quit() {
	stop = true;

	// this should force read command to return
	closeAndInvalidate(listener_socket_fd);
	closeAndInvalidate(control_channel_fd);
	closeAndInvalidate(data_channel_fd);


	/* The startAccepting() loop only checks the stop member
	* variable after accept() returned. However, accept() is a
	* blocking system call and may not return unless a new
	* connection is established. For this reason, we set the stop
	* variable and afterwards connect() to the server socket to make
	* sure the receive loop terminates. */

	GpioClient client;
	if (base_port) client.setupConnection(NULL, base_port);
}

void GpioServer::registerOnChange(OnChangeCallback fun) {
	this->onchange_fun = fun;
}

int GpioServer::awaitConnection(int socket) {
	struct sockaddr_storage their_addr;  // connector's address information
	socklen_t sin_size = sizeof their_addr;
	// char s[INET6_ADDRSTRLEN];

	int new_connection = accept(socket, (struct sockaddr *)&their_addr, &sin_size);
	if (new_connection < 0) {
		cerr << "[gpio-server] accept return " << strerror(errno) << endl;
		return -1;
	}

	//inet_ntop(their_addr.ss_family, get_in_addr((struct sockaddr *)&their_addr), s, sizeof s);
	//DEBUG("gpio-server: got connection from %s\n", s);

	return new_connection;
}


void GpioServer::startAccepting() {
	// printf("gpio-server: accepting connections (%d)\n", fd);

	while (!stop)  // this would block a bit
	{
		if((control_channel_fd = awaitConnection(listener_socket_fd)) < 0) {
			cerr << "[gpio-server] could not accept new connection" << endl;
			continue;
		}
		handleConnection(control_channel_fd);
		closeAndInvalidate(control_channel_fd);
	}
}

bool GpioServer::isConnected() {
	return control_channel_fd >= 0;
}

void GpioServer::handleConnection(Socket conn) {
	Request req;
	memset(&req, 0, sizeof(Request));
	int bytes;
	while (readStruct(conn, &req)) {
		// hexPrint(reinterpret_cast<char*>(&req), bytes);
		switch (req.op) {
			case Request::Type::GET_BANK:
				if (!writeStruct(conn, &state)) {
					cerr << "could not write answer" << endl;
					return;
				}
				break;
			case Request::Type::SET_BIT: {
				// printRequest(&req);
				if (req.setBit.pin >= max_num_pins) {
					cerr << "[gpio-server] invalid request setbit pin number " << req.setBit.pin << endl;
					break;
				}

				if (isIOF(state.pins[req.setBit.pin])){
					cerr << "[gpio-server] Ignoring setPin on IOF" << endl;
					break;
				}
				// sanity checks ok

				if (onchange_fun != nullptr) {
					onchange_fun(req.setBit.pin, req.setBit.val);
				} else {
					state.pins[req.setBit.pin] = toPinstate(req.setBit.val);
				}
				break;
			}
			case Request::Type::UART_TRANSMIT_SINGLE: {
				if (req.uartSingle.pin >= max_num_pins) {
					cerr << "[gpio-server] invalid request sendUART pin number " << req.uartSingle.pin << endl;
					break;
				}

				if (state.pins[req.uartSingle.pin] != Pinstate::IOF_UART_RX){
					cerr << "[gpio-server] Info: Ignoring sendUart " << +req.uartSingle.byte << " on not UART to pin " << +req.uartBurst.pin << endl;
					break;
				}

				const auto& cb = active_UART_channels.find(req.uartSingle.pin);
				if(cb == active_UART_channels.end()) {
					cerr << "[gpio-server] Info: Ignoring uart single to not-listening pin " << +req.uartSingle.pin << endl;
					break;
				}
				cb->second(std::vector{req.uartSingle.byte});
				break;
			}
			case Request::Type::UART_TRANSMIT_BURST: {
				if (req.uartBurst.pin >= max_num_pins) {
					cerr << "[gpio-server] invalid request sendUART pin number " << req.uartBurst.pin << endl;
					break;
				}

				if (state.pins[req.uartBurst.pin] != Pinstate::IOF_UART_RX){
					cerr << "[gpio-server] Info: Ignoring sendUart on not UART pin " << +req.uartBurst.pin << endl;
					break;
				}

				// TODO: Very slow memory-wise if not on stack (with reserve and resize(0))
				gpio::UART_Bytes uart_bytes(req.uartBurst.num_bytes);
				if(!readStruct(conn, &uart_bytes)) {
					cerr << "[gpio-server] Could not get read UART burst bytes" << endl;
					closeAndInvalidate(control_channel_fd);
					break;
				}

				const auto& cb = active_UART_channels.find(req.uartBurst.pin);
				if(cb == active_UART_channels.end()) {
					cerr << "[gpio-server] Info: Ignoring uart burst to not-listening pin " << +req.uartBurst.pin << endl;
					break;
				}
				cb->second(uart_bytes);
				break;
			}
			case Request::Type::REQ_IOF:
			{
				Req_IOF_Response response{0};
				response.id = findNewID();			// FIXME: ignoring the fact that IDs may run out
				response.port = data_channel_port;	// will be overwritten if channel not existing

				if(data_channel_fd < 0) {
					// need to offer and connect new connection
					Socket data_channel_listener = openSocket("0");	// zero requests random port
					if (data_channel_listener < 0) {
						cerr << "[gpio-server] Could not setup IOF data channel socket" << endl;
						// no break, so that response is 0
					}

					{ // setup new listening socket for IOF data channel
						struct sockaddr_in sin;
						socklen_t len = sizeof(sin);
						if (getsockname(data_channel_listener, (struct sockaddr *)&sin, &len) == -1) {
							cerr << "[gpio-server] Could not get port with sockname" << endl;
							closeAndInvalidate(data_channel_listener);
						}
						response.port = ntohs(sin.sin_port);
						data_channel_port = response.port;
					}

					// send port response to client
					if (!writeStruct(conn, &response)) {
						cerr << "[gpio-server] could not write IOF-Req answer" << endl;
						closeAndInvalidate(data_channel_listener);
						closeAndInvalidate(control_channel_fd);
						return;
					}

					// TODO: set socket nonblock or with timeout (~1s)
					data_channel_fd = awaitConnection(data_channel_listener);
					closeAndInvalidate(data_channel_listener);	// accepting just the first connection
				}
				else {
					if (!writeStruct(conn, &response)) {
						cerr << "[gpio-server] could not write IOF-Req answer" << endl;
						closeAndInvalidate(control_channel_fd);
						return;
					}
				}

				//cout << "[gpio-server] Started IOF channel type " << static_cast<unsigned>(req.reqIOF.iof) <<
				//		" on pin " << +req.reqIOF.pin << " with ID " << +response.id << endl;
				IOF_Channelinfo info = {.id = response.id, .requested_iof = req.reqIOF.iof };
				active_IOF_channels.emplace(req.reqIOF.pin, info);

				break;
			}
			case Request::Type::END_IOF:
				{
				auto channel = active_IOF_channels.find(req.reqIOF.pin);
				if(channel == active_IOF_channels.end()) {
					cerr << "[gpio-server] IOF quit on non active pin " << +req.reqIOF.pin << endl;
					return;
				}
				//cout << "[gpio-server] IOF quit on pin " << (int)req.reqIOF.pin << endl;
				active_IOF_channels.erase(channel);
				}
				break;
			default:
				cerr << "[gpio-server] invalid request operation" << endl;
				return;
		}
	}

	DEBUG("gpio-client disconnected (%d)\n", bytes);
}

void GpioServer::pushPin(gpio::PinNumber pin, gpio::Tristate state) {
	// TODO: Maybe add redundant array with active PIN subscriptions to speed up this check
	auto channel = active_IOF_channels.find(pin);
	if(channel == active_IOF_channels.end()) {
		return;
	}

	if(channel->second.requested_iof != Request::IOFunction::BITSYNC) {
		// requested different IOF
		return;
	}

	IOF_Update update;
	update.id = channel->second.id;
	update.payload.pin = state;

	if(!writeStruct(data_channel_fd, &update)) {
		cerr << "[gpio-server] Could not write PIN update to pin " << +pin << endl;
		resetAllDatachannels();
	}
}

SPI_Response GpioServer::pushSPI(gpio::PinNumber pin, gpio::SPI_Command byte) {
	auto channel = active_IOF_channels.find(pin);
	if(channel == active_IOF_channels.end()) {
		// Not even an active IOF channel
		return 0;
	}
	/*
	 * TODO: If registered on SPI data pins (miso, mosi, clk) and not on CS,
	 * It should receive all SPI data, not just the CS activated ones
	 */

	if(channel->second.requested_iof != Request::IOFunction::SPI &&
	   channel->second.requested_iof != Request::IOFunction::SPI_NORESPONSE) {
		// requested different IOF
		return 0;
	}

	// TODO: Check if PIN is actually a CS pin (or request was "global")

	const bool noResponse = channel->second.requested_iof == Request::IOFunction::SPI_NORESPONSE;

	IOF_Update update;
	update.id = channel->second.id;
	update.payload.spi = byte;

	if(!writeStruct(data_channel_fd, &update)) {
		cerr << "[gpio-server] Could not write SPI command to cs " << +pin << endl;
		resetAllDatachannels();
		return 0;
	}

	SPI_Response response = 0;
	if(!noResponse && !readStruct(data_channel_fd, &response)) {
		cerr << "[gpio-server] Could not read SPI response to cs " << +pin << endl;
		resetAllDatachannels();
	}
	return response;
}

void GpioServer::registerUARTRX(gpio::PinNumber pin, OnUART_RX_Callback callback) {
	const auto uartCB = active_UART_channels.find(pin);
	if(uartCB != active_UART_channels.end()) {
		// Already an CB on pin active
		cerr << "[gpio-server] [uart] Warn: Overwriting an active callback on pin " << +pin << endl;
		active_UART_channels.erase(pin);
	}
	// check of configured as RX is done in dispatcher
	active_UART_channels.emplace(pin, callback);
}

void GpioServer::pushUART(gpio::PinNumber pin, UART_Bytes bytes) {
	auto channel = active_IOF_channels.find(pin);
	if(channel == active_IOF_channels.end()) {
		// Not even an active IOF channel
		return;
	}
	if(channel->second.requested_iof != Request::IOFunction::UART_RX){
		// requested different IOF
		return;
	}
	if(state.pins[pin] != Pinstate::IOF_UART_TX){
		// not configured as TX pin
		return;
	}

	// TODO: BURST not yet implemented

	IOF_Update update;
	update.id = channel->second.id;
	for(const auto& byte : bytes) {
		update.payload.uart = byte;
		if(!writeStruct(data_channel_fd, &update)) {
			cerr << "[gpio-server] Could not write uart byte to pin " << +pin << endl;
			resetAllDatachannels();
			return;
		}
	}
}

