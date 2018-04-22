//
// Laboratory of Robotics and Cognitive Science
// Created by:  Leonardo de Oliveira Ramos
// Github:      https://github.com/oramleo
//

extern "C" {
#include "extApi.h"
}

#include <iostream>
#include "connection.h"

Connection::Connection()
{
	server_ip = "127.0.0.1";
	server_port = 19999;

	time_out_in_ms = 2000;
	comm_thread_cycle_in_ms = 5;

	waitUntilConnected = true;
	doNotReconnectOnceDisconnected = true;

	std::cout << "Connection started" << std::endl;
	client_id = -1;

	client_id = simxStart((simxChar *) server_ip.c_str(), (simxInt) server_port, (simxUChar) waitUntilConnected,
						  (simxUChar) doNotReconnectOnceDisconnected, time_out_in_ms, comm_thread_cycle_in_ms);
}

Connection::Connection(const std::string &server_ip, int server_port, simxInt time_out_in_ms,
					   simxInt comm_thread_cycle_in_ms, bool waitUntilConnected, bool doNotReconnectOnceDisconnected,
					   int client_id) : server_ip(server_ip), server_port(server_port), time_out_in_ms(time_out_in_ms),
										comm_thread_cycle_in_ms(comm_thread_cycle_in_ms),
										waitUntilConnected(waitUntilConnected),
										doNotReconnectOnceDisconnected(doNotReconnectOnceDisconnected),
										client_id(client_id)
{
	client_id = -1;
	client_id = simxStart((simxChar *) server_ip.c_str(), (simxInt) server_port, (simxUChar) waitUntilConnected,
						  (simxUChar) doNotReconnectOnceDisconnected, time_out_in_ms, comm_thread_cycle_in_ms);
}

bool Connection::isConnected()
{
	return client_id != -1;
}

int Connection::getClientId() const
{
	return client_id;
}


