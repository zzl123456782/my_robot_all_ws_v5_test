#pragma once
#include <linux/can.h>
#include <string>


#include "dbc_encoder_decoder.h"

int initializeSocketCAN(const std::string& interface);
bool sendCANFrame(int socket_fd, const struct can_frame& frame);
// void receiveCANFrame(int socket_fd, float& vehicleSpd);
void receiveCANFrame(int socket_fd, double& left_speed, double& right_speed, BMS_Data& bms_data);
