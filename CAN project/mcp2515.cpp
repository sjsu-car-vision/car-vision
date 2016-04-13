/*
 * mcp2515.cpp
 *
 *  Created on: Apr 10, 2016
 *      Author: YuYu
 */

#include "mcp2515.hpp"



using namespace std;

static Gpio cs(FLASH_PORT, FLASH_PIN, GPIO_OUTPUT, GPIO_HIGH);

mcp2515::mcp2515(uint8_t speed) : spi(My_spi1::get_instance())
{
    spi->init(500);


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

uint8_t mcp2515::rd_status(uint8_t type) {
    uint8_t data;

    cs.set_state(GPIO_LOW);
    spi->exchange_byte(SPI_READ_STATUS);
    data = spi->exchange_byte(0x0);
    cs.set_state(GPIO_HIGH);

    return data;
}


