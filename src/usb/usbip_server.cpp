#include "usb/usbip_server.hpp"
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

using namespace USB::USBIP;

Server::Server()
: m_server_socket{socket(AF_INET, SOCK_STREAM, 0)}
{

}

void Server::listen()
{
    sockaddr_in serverAddress;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(m_port);
    serverAddress.sin_addr.s_addr = INADDR_ANY;
}