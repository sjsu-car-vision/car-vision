/*
 * mcp2515.h
 *
 *  Created on: Apr 10, 2016
 *      Author: YuYu
 */

#ifndef L5_APPLICATION_CAN_MCP2515_HPP_
#define L5_APPLICATION_CAN_MCP2515_HPP_

#include "mcp2515_def.hpp"
#include <stdio.h>
#include "utilities.h"


#include "own_driver/My_spi1.hpp"
#include "own_driver/my_gpio.hpp"

#include "ssp1.h"

#define FILTER 0
#define PID_REPLY           0x7E8
#define MASK           0x7FF
typedef struct
{
        uint16_t id;
        struct
        {
                int8_t rtr :1;
                char length :4;
        } header;
        char data[8];

}canMessage;

enum int_list {
    /* Interrupt Input Pin */
    INT_PORT = 1, INT_PIN = 20,
};

enum rst_list {
    /* Reset output Pin */
    RST_PORT = 1, RST_PIN = 22,
};


/*
 * CAN controller interface
 *
 */
class mcp2515  {

    public:
        mcp2515(char speed); // constructor inits
        bool init(char speed); // actual init of mcp2515

        void wr_reg(char addr, char data);
        char rd_reg(char addr);
        void bit_modify(char addr, char mask, char data);
        char rd_status(char opcode);
        bool check_message();   // see if interrupt occurred
        bool check_free_buffer();   // check all 3 TX buffers if they are full
        bool get_message(canMessage *message);
        bool send_message(canMessage *message);

        void wr_Filt_reg(char addr);
        void wr_Mask_reg(char addr);

        void read_msg();


        //bool set_CSn_inactive(int pin);
        //bool set_CSn_active(int pin);
    private:
        My_spi1* spi;


};

#endif /* L5_APPLICATION_CAN_MCP2515_HPP_ */
