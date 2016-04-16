/*
 * CANTask.hpp
 *
 *  Created on: Apr 16, 2016
 *      Author: YuYu
 */

#ifndef L5_APPLICATION_CAN_CANTASK_HPP_
#define L5_APPLICATION_CAN_CANTASK_HPP_

#include "CAN.hpp"
#include "tasks.hpp"

/*
 * CANTask
 *initialize and prints out car velocity every 1 second
 */
class CANTask: public scheduler_task
{
    public:
        CANTask(uint8_t priority); //constructor
        bool run(void *p); //


    private:
        CANbus CAN_object;

};



#endif /* L5_APPLICATION_CAN_CANTASK_HPP_ */
