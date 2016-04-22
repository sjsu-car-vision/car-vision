/*
 * mcp2515.h
 *
 *  Created on: Apr 10, 2016
 *      Author: YuYu
 */

#ifndef L5_APPLICATION_CAN_MCP2515_HPP_
#define L5_APPLICATION_CAN_MCP2515_HPP_

#include "mcp2515_def.hpp"

#include "own_driver/My_spi1.hpp"
#include "own_driver/my_gpio.hpp"


typedef struct
{
        uint16_t id;
        struct
        {
                int8_t rtr :1;
                uint8_t length :4;
        } header;
        uint8_t data[8];

}canMessage;

enum int_list {
    /* Interrupt Input Pin */
    INT_PORT = 1, INT_PIN = 20,
};


/*
 * CAN controller interface
 *
 */
class mcp2515  {

    public:
        mcp2515(uint8_t speed); // constructor inits
        bool init(uint8_t speed); // actual init of mcp2515

        void wr_reg(uint8_t addr, uint8_t data);
        uint8_t rd_reg(uint8_t addr);
        void bit_modify(uint8_t addr, uint8_t mask, uint8_t data);
        uint8_t rd_status(uint8_t opcode);
        bool check_message();   // see if interrupt occurred
        bool check_free_buffer();   // check all 3 TX buffers if they are full
        bool get_message(canMessage *message);
        bool send_message(canMessage *message);


    private:
        My_spi1* spi;


};

#endif /* L5_APPLICATION_CAN_MCP2515_HPP_ */
