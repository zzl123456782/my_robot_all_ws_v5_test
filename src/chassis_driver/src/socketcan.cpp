#include "socketcan.h"
#include <iostream>
#include <iomanip>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <fcntl.h>
#include "dbc_encoder_decoder.h"
dbc_encoder my_dbc;
// 初始化 CAN 套接字
int initializeSocketCAN(const std::string& interface) {
    int socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd < 0) {
        perror("Socket creation failed");
        return -1;
    }

    struct ifreq ifr;
    strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ);
    if (ioctl(socket_fd, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl failed");
        close(socket_fd);
        return -1;
    }

    struct sockaddr_can addr = {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("Bind failed");
        close(socket_fd);
        return -1;
    }

    /***************add buffer************/
    int receive_buffer_size = 256 * 1024;
    setsockopt(socket_fd, SOL_SOCKET, SO_RCVBUF, &receive_buffer_size, sizeof(receive_buffer_size));
    int flags = fcntl(socket_fd, F_GETFL, 0);
    return socket_fd;
}

// 发送 CAN 帧
bool sendCANFrame(int socket_fd, const struct can_frame& frame) {
    if (write(socket_fd, &frame, sizeof(frame)) != sizeof(frame)) {       
        perror("Write failed");
        return false;
    }
    // std::cout<< "CAN Frame Sent successful!"<< std::endl;
    return true;
}


// void receiveCANFrame(int socket_fd, double& left_speed, double& right_speed) {
//     // std::cout<<"receiveCANFrame"<<std::endl;
//     struct can_frame frame;
//     while ((true)){
//         int nbytes = recv(socket_fd, &frame, sizeof(struct can_frame), MSG_DONTWAIT); /* code */
//         if (nbytes < 0){
//             if (errno == EWOULDBLOCK||errno ==EAGAIN){
//                 break;
//             }else{
//                 perror("recv failed");
//             }
//         }else{
//             my_dbc.parseVehicleSpd(frame, left_speed, right_speed);
//         }
//     }
// }

void receiveCANFrame(int socket_fd, double& left_speed, double& right_speed, BMS_Data& bms_data) {
    // std::cout<<"receiveCANFrame"<<std::endl;
    struct can_frame frame;
    while ((true)){
        int nbytes = recv(socket_fd, &frame, sizeof(struct can_frame), MSG_DONTWAIT); /* code */
        if (nbytes < 0){
            if (errno == EWOULDBLOCK||errno ==EAGAIN){
                break;
            }else{
                perror("recv failed");
            }
        }else if(nbytes == sizeof(struct can_frame)){
            
            // Filter our own tx frames if loopback is on
            if(frame.can_id == 0x20){continue;}
            
            // Standard error checks 
            if(frame.can_id & CAN_ERR_FLAG){continue;}
            if(frame.can_id & CAN_RTR_FLAG){continue;}        
            
            
            // Check if it's the motor feedback (0x581)
            if ((frame.can_id & CAN_SFF_MASK) == 0x581) {
                 my_dbc.parseVehicleSpd(frame, left_speed, right_speed);
            } else {
                 // Try parsing as BMS frame (stores in global/member - wait, we need to pass data back!)
                 // The current signature of receiveCANFrame only returns speeds.
                 // We need to either change the signature or expose the decoder's state.
                 // For minimal invasion, we might need to change how this is called.
                 // Ideally `ros1_can_node` should own the data object.
                 my_dbc.parseFrame(frame, bms_data);
            }
            // Temporarily, we can't fully integrate here without headers update.
            // But `my_dbc` is global here? No, it's global in file scope line 13.
            // We should change `receiveCANFrame` to take the BMS data struct or move the instance logic.
        }
    }
}