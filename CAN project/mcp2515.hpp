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

class mcp2515  {

    public:
        mcp2515(uint8_t speed); // constructor inits

        void wr_reg(uint8_t addr, uint8_t data);
        uint8_t rd_reg(uint8_t addr);
        void bit_modify(uint8_t addr, uint8_t mask, uint8_t data);
        uint8_t rd_status(uint8_t type);
        uint8_t mcp2515_check_message(void);
        uint8_t mcp2515_check_free_buffer(void);
        uint8_t mcp2515_get_message(canMessage *message);
        uint8_t mcp2515_send_message(canMessage *message);


    private:
        My_spi1* spi;


};

#endif /* L5_APPLICATION_CAN_MCP2515_HPP_ */
