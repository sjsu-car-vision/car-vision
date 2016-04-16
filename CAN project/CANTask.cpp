/*
 * CANTask.cpp
 *
 *  Created on: Apr 16, 2016
 *      Author: YuYu
 */

#include "CANTask.hpp"
#include <stdio.h>



CANTask::CANTask(uint8_t priority): scheduler_task("CAN_task", 512 * 4, priority), CAN_object(CANSPEED_500)
{

}


bool CANTask::run(void *p)
{
    printf("%d\n", CAN_object.getSpeed(VEHICLE_SPEED));

    vTaskDelay(1000);
    return true;
}


