#include "usb/usbip_server.hpp"
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <fnctl.h>
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
    if (bind(m_server_socket, (sockaddr*)&serverAddress, sizeof(serverAddress)) != 0) {
        // ERROR
    }
    if (listen(m_server_socket, 5) != 0) {
        // ERROR
    }
    // where socketfd is the socket you want to make non-blocking
    int flags = fcntl(m_server_socket, F_GETFL, 0);
    int status = fcntl(m_server_socket, F_SETFL, flags | O_NONBLOCK);

    if (status == -1){
        perror("calling fcntl");
        // handle the error.  By the way, I've never seen fcntl fail in this way
    }
}

void Server::poll()
{
    // check for any incoming/closed connections
    // update connection list
    // iterate over connections
    
    // process any data requests
    for (auto &[k, v]: m_client_sockets) {
        if (v.m_has_device) {
            // handle USBIP_CMD_SUBMIT commands
            // or USBIP_UNLINK command
            USBIP_CMD_Header_Basic header;
            int status = recv(k, &header, sizeof(header), 0);
            if (status <= 0) continue;
            if (status < sizeof(header)) {
                // kill connection.
                continue;
            }
            header.command   = ntohl(header.command  );
            header.seqnum    = ntohl(header.seqnum   );
            header.devid     = ntohl(header.devid    );
            header.direction = ntohl(header.direction);
            header.ep        = ntohl(header.ep       );
            switch(header.command) {
                case 0x00000001: // CMD_SUBMIT
                {
                    USBIP_CMD_SUBMIT_Packet packet;
                    status = recv(k, &packet, sizeof(packet), 0);
                    if (status != sizeof(packet)) {
                        // error, kill
                        continue;
                    }
                    packet.transfer_flags = ntohl(packet.transfer_flags);
                    packet.transfer_buffer_length = ntohl(packet.transfer_buffer_length);
                    packet.start_frame = ntohl(packet.start_frame);
                    packet.num_iso_packets = ntohl(packet.num_iso_packets);
                    packet.interval = ntohl(packet.interval);
                    std::vector<uint8_t> transfer_buffer{};
                    transfer_buffer.resize(transfer_buffer_length);
                    status = recv(k, transfer_buffer.data(), transfer_buffer_length, 0);
                    if (status != transfer_buffer_length) {
                        // error, kill
                        continue;
                    }


                }
                case 0x00000003: // CMD_UNLINK
                default: // Error case
            }
        } else {
            // OP_REQ_DEVLIST
            // OP_REQ_IMPORT
            OP_REQ_Header header;
            int status = recv(k, &header, sizeof(header), 0);
            if (status <= 0) continue;
            if (status < sizeof(header)) {
                // kill connection.
                continue;
            }
            header.ver    = ntohs(header.ver   );
            header.cmd    = ntohs(header.cmd   );
            header.status = ntohl(header.status);
            if (ver != 0x0111) {
                // wrong version!
                continue;
            }
            switch( header.cmd ) {
                case 0x8005: // OP_REQ_DEVLIST
                {
                    std::vector<uint8_t> buf;
                    buf.resize(12);
                    *(uint8_t*)(&htons())
                    send(k, buf, 12);
                }
                case 0x8003: // OP_REQ_IMPORT
                default: // Error case
            }
        }
    }
}