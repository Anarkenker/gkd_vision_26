#pragma once
#include <string>
void init_send(std::string ip);
void send_control(double yaw_set, double pitch_set, bool fire);