/*
 * CAN.hpp
 *
 *  Created on: Apr 14, 2016
 *      Author: YuYu
 */

#ifndef L5_APPLICATION_CAN_CAN_HPP_
#define L5_APPLICATION_CAN_CAN_HPP_

#include "mcp2515.hpp"

#define CANSPEED_125    7       // CAN speed at 125 kbps
#define CANSPEED_250    3       // CAN speed at 250 kbps
#define CANSPEED_500    1       // CAN speed at 500 kbps

#define ENGINE_COOLANT_TEMP 0x05
#define ENGINE_RPM          0x0C
#define VEHICLE_SPEED       0x0D
#define MAF_SENSOR          0x10
#define O2_VOLTAGE          0x14
#define THROTTLE            0x11

#define PID_REQUEST         0x7DF
#define PID_REPLY           0x7E8


/*
 * More C++ Wrapper
 * Does the sending/receiving of messages and get Car Velocity in Km/Hr
 * */
class CANbus {
    public:
        CANbus(uint8_t speed);

        bool CAN_sendMessage(uint8_t pid);
        bool CAN_getMessage(unsigned char *messageBuffer);
        uint8_t getSpeed(uint8_t pid);
    private:
        mcp2515 controller;
};

#endif /* L5_APPLICATION_CAN_CAN_HPP_ */
