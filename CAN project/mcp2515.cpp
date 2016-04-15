/*
 * mcp2515.cpp
 *
 *  Created on: Apr 10, 2016
 *      Author: YuYu
 */

#include "mcp2515.hpp"



using namespace std;

static Gpio cs(FLASH_PORT, FLASH_PIN, GPIO_OUTPUT, GPIO_HIGH);
static Gpio intr(INT_PORT, INT_PIN, GPIO_INPUT, GPIO_HIGH); // set Px.x as interrupt input pin

mcp2515::mcp2515(uint8_t speed) : spi(My_spi1::get_instance())
{
    spi->init(500);
    init(speed);


}

bool mcp2515::init(uint8_t speed) {


    cs.set_state(GPIO_LOW);
    spi->exchange_byte(SPI_RESET); // reset to go into configuration mode
    cs.set_state(GPIO_HIGH);

    // continuous write will increment address pointer
    cs.set_state(GPIO_LOW);
    spi->exchange_byte(SPI_WRITE);
    spi->exchange_byte(CNF3);
    spi->exchange_byte(0x2 << PHSEG2);  // PS2 Length bits, (PHSEG2 + 1) x Tq , minimum valid setting is 2Tq
    spi->exchange_byte(((1 << BTLMODE) | (0x2 << PHSEG1)) ); // set to PS2 bit time length, CNF2
    spi->exchange_byte(speed); // set baud rate, not quite sure how timing is done..., but Tq = 2(speed+1)/16MHz, CNF1
    spi->exchange_byte((1 << RX1IE | 1 << RX0IE)); // CANINTE , Receive buffer 0/1 full interrupt enable
    cs.set_state(GPIO_HIGH);

    if (rd_reg(CNF1) != speed)
        return false;

    wr_reg(BFPCTRL,0); // according to schematic, NC, so disable RXnBF (Hi-z)
    wr_reg(TXRTSCTRL,0); // according to schematic, NC, so disable TXnRTS (Hi-z)

    wr_reg(RXB0CTRL, 0x3 << RXM0); // turn off mask/filter, receive any message
    wr_reg(RXB1CTRL, 0x3 << RXM1); // same

    wr_reg(CANCTRL, 0); // set to normal operation

    return true;
}


void mcp2515::wr_reg(uint8_t addr, uint8_t data) {
    cs.set_state(GPIO_LOW);
    spi->exchange_byte(SPI_WRITE);
    spi->exchange_byte(addr);
    spi->exchange_byte(data);
    cs.set_state(GPIO_HIGH);

}

uint8_t mcp2515::rd_reg(uint8_t addr) {

    uint8_t data;

    cs.set_state(GPIO_LOW);
    spi->exchange_byte(SPI_READ);
    spi->exchange_byte(addr);
    data = spi->exchange_byte(0x0);
    cs.set_state(GPIO_HIGH);

    return data;
}

void mcp2515::bit_modify(uint8_t addr, uint8_t mask, uint8_t data)
{
    cs.set_state(GPIO_LOW);
    spi->exchange_byte(SPI_BIT_MODIFY);
    spi->exchange_byte(addr);
    spi->exchange_byte(mask);
    data = spi->exchange_byte(data);
    cs.set_state(GPIO_HIGH);

}

uint8_t mcp2515::rd_status(uint8_t opcode) {
    uint8_t data;

    cs.set_state(GPIO_LOW);
    spi->exchange_byte(SPI_READ_STATUS);
    data = spi->exchange_byte(0x0);
    cs.set_state(GPIO_HIGH);

    return data;
}

bool mcp2515::check_message(){
    return (!intr.get_state()); // INTn pin

}

bool mcp2515::check_free_buffer(){
    uint8_t status;
    status = rd_status(SPI_READ_STATUS); // check TX buffer
    if(status == 0x54)
        return false;

    return true;

}
