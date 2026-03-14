#include "socket_interface.hpp"

namespace IO
{
    void Server_socket_interface::task() {
        while (true) {
            //printf("socket task\n");
            memset(buffer, 0, sizeof(buffer));
            sockaddr_in cli_addr;
            socklen_t cli_addr_len = sizeof(cli_addr);
            auto n = recvfrom(sockfd, buffer, 256, MSG_WAITALL, (sockaddr *)&cli_addr, &cli_addr_len);
            if (n > 0) {
                //
                memcpy(&pkg, buffer, sizeof pkg);
                //printf("receive pkg yaw:%.6f\tpitch:%.6f\n", pkg.yaw, pkg.pitch);
            }
        }
    }

    Server_socket_interface::Server_socket_interface() : port_num(11453) {
        // NOTE: read this https://www.linuxhowtos.org/C_C++/socket.htm
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd < 0) {
            printf("can't open socket\n");
        }

        serv_addr.sin_family = AF_INET;
        serv_addr.sin_addr.s_addr = INADDR_ANY;
        serv_addr.sin_port = htons(port_num);

        if (bind(sockfd, (sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
            printf("can't bind socket fd with port number");
        }
    }

    Server_socket_interface::~Server_socket_interface() {
        close(sockfd);
    }
}  // namespace IO
