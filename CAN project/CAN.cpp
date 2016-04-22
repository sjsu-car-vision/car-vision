/*
 * CAN.cpp
 *
 *  Created on: Apr 15, 2016
 *      Author: YuYu
 */

#include "CAN.hpp"


CANbus::CANbus(uint8_t speed): controller(speed) {

}

bool CANbus::CAN_sendMessage(uint8_t pid) {

    canMessage message;

    // building message... from sparkfun example...
    message.id = PID_REQUEST;
    message.header.rtr = 0; // Data Frame
    message.header.length = 8; // # of bytes to be transferred
    message.data[0] = 0x02;
    message.data[1] = 0x01;
    message.data[2] = pid;
    message.data[3] = 0x00;
    message.data[4] = 0x00;
    message.data[5] = 0x00;
    message.data[6] = 0x00;
    message.data[7] = 0x00;

    if(controller.send_message(&message)) {
        return true;
    }
    else {
        return false;
    }

    return true;
}

bool CANbus::CAN_getMessage(uint8_t *messageBuffer) {
    canMessage message;

    if(controller.check_message()) { // check the interrupt pin value
        if(controller.get_message(&message)) { // get CAN message via CAN controller SPI interface
                                                // and store it in passed in array
            messageBuffer[0] = message.data[0];
            messageBuffer[1] = message.data[1];
            messageBuffer[2] = message.data[2];
            messageBuffer[3] = message.data[3];
            messageBuffer[4] = message.data[4];
            messageBuffer[5] = message.data[5];
            messageBuffer[6] = message.data[6];
            messageBuffer[7] = message.data[7];
        }
        else {
            return false;
        }
    }
    else {
        return false;
    }
    return true;
}

uint8_t CANbus::getSpeed(uint8_t pid) {
    canMessage message;
    uint8_t carSpeed = 0;
    // building message... from sparkfun example...
    message.id = PID_REQUEST;
    message.header.rtr = 0; // Data Frame
    message.header.length = 8; // # of bytes to be transferred
    message.data[0] = 0x02;
    message.data[1] = 0x01;
    message.data[2] = pid;
    message.data[3] = 0x00;
    message.data[4] = 0x00;
    message.data[5] = 0x00;
    message.data[6] = 0x00;
    message.data[7] = 0x00;


    if (controller.send_message(&message))  // Send the message
    {
        if (controller.check_message()) // check the interrupt pin value
        {
            if (controller.get_message(&message)) // get CAN message via CAN controller SPI interface
            {                                       // and store it in passed in struct
                if(message.data[2] == VEHICLE_SPEED) {
                    carSpeed = message.data[3];
                }
            }
        }
    }
    return carSpeed;
}


