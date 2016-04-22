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
    bool success;
    puts("Initializing SPI driver...");
    spi->init(500);
    puts("Initializing CAN bus controller...");
    success = init(speed);
    if (success)
        puts("CAN bus controller init successful");
    else
        puts("CAN bus controller init failed");
}

bool mcp2515::init(uint8_t speed) {


    cs.set_state(GPIO_LOW);
    spi->exchange_byte(SPI_RESET); // reset to go into configuration mode
    cs.set_state(GPIO_HIGH);

    // continuous write will increment address pointer
    cs.set_state(GPIO_LOW);
    spi->exchange_byte(SPI_WRITE);
    spi->exchange_byte(CNF3);

    // =================== FOR ALL SPEED =====================
    // setting bit timing to 16Tq
    // PS2 Length bits, (PHSEG2 + 1) x Tq , minimum valid setting is 2Tq
    // set to PS2 bit time length, and set PS1, (PHSEG1 + 1) x Tq , and PRSEG
    // setting all config to have 1Tq propagation segment length
    // sync bit by default is 1Tq
    // SJW set to 1Tq
    // Baud rate prescaler, Tq = 2(speed+1)/16MHz
    // Sample Point will be at 62.5% of bit timing

    spi->exchange_byte(0x5 << PHSEG2); // setting PS2 to 6Tq
    spi->exchange_byte(((1 << BTLMODE) | (0x7 << PHSEG1) | (0x0 << PRSEG))); // setting PS1 to 8Tq, 1Tq PRSEG
    spi->exchange_byte(((0 << SJW) |(speed << BRP0))); // set Synchronization Jump Width to 1Tq, set baud rate prescaler

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
    status = rd_status(SPI_READ_STATUS); // check TX buffer (TXBnCNTRL.TXREQ),
    if(status == 0x54)                      // 0 1 0 1   0 1 0 0
        return false;                       //   ^   ^     ^
                                            //  TX2  TX1   TX0
    return true;

}

bool mcp2515::get_message(canMessage *message) {
    uint8_t status;
    uint8_t addr;
    uint16_t SID;
    uint8_t dataLength;

    status = rd_status(SPI_RX_STATUS); // check which buffer received message is in

    if (status & (1 << 6)) { // Message in RXB0
        addr = SPI_READ_RX;
    }
    else if (status & (1 << 7)) { // Message in RXB1
        addr = (SPI_READ_RX | 0x4);
    }
    else {
        // no message
        return false;
    }

    // starting address should be either 0x61 or 0x71

    cs.set_state(GPIO_LOW);
    spi->exchange_byte(addr);

    // getting Standard Identifier bits [10:0]        bits 15   --------------------->    0
    SID =  spi->exchange_byte(0) << 3;                   // 0 0 0 0 0 1 1 1 1 1 1 1 1 0 0 0
    SID |= spi->exchange_byte(0) >> 5;                   // 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1
                                                         // 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 1

    message->id = SID;

    // address pointer auto increments to next register after each read
    spi->exchange_byte(0);  // skipping EIDH reg
    spi->exchange_byte(0);  // skipping EIDL reg

    // reading DLC (data length code) register, only need last 4 bits
    dataLength = (spi->exchange_byte(0) & 0xf);
    message->header.length = dataLength;

    // check if it is a standard data frame or remote frame
    message->header.rtr = (status & (1 << 3)) ? 1: 0;

    // store x amount of data bytes according to DLC
    for(int i =0; i < dataLength; i++) {
        message->data[i] = spi->exchange_byte(0);
    }
    // end transaction
    cs.set_state(GPIO_HIGH);

    // clear receive interrupt flag to reset interrupt condition (need to be cleared to get new message)
    if (status & (1 << 6)) // check which receive buffer has a message
    {
        bit_modify(CANINTF, 1 << RX0IF, 0); // clear receive buffer 0 flag bit
    }
    else
    {
        bit_modify(CANINTF, 1 << RX1IF, 0); // clear receive buffer 1 flag bit
    }

    return true;
}

bool mcp2515::send_message(canMessage *message) {
    uint8_t status;
    uint8_t addr;
    uint8_t dataLength;

    status =rd_status(SPI_READ_STATUS);

    if (!(status & (1 << 2))) // TXB0CNTRL.TXREQ, check TX0 buffer if it is empty
    {
        addr =  0x0; // addr = 0x40, address points to TXB0SIDH (0x31)
    }
    else if (!(status & (1 << 4))) // check TX1 buffer if it is empty
    {
        addr =  0x2; // addr = 0x42, address points to TXB1SIDH (0x41)
    }
    else if (!(status & (1 << 6))) // check TX2 buffer if it is empty
    {
        addr =  0x4; // addr = 0x44, address points to TXB2SIDH (0x51)
    }
    else
    {
        // all buffer full/busy
        return false;
    }

    // start transaction , loading data into registers before transmission
    cs.set_state(GPIO_LOW);
    spi->exchange_byte((SPI_WRITE_TX | addr)); // address depends on top

    spi->exchange_byte(message->id >> 3);  // SID bits [10:3]
    spi->exchange_byte((message->id & 0x7) << 5);  // SID bits [2:0] into bits[7:5] of TXBnSIDL

    spi->exchange_byte(0); // skip EID regs
    spi->exchange_byte(0);

    dataLength = (message->header.length & 0xf); // DLC to be sent

    if(message->header.rtr)
        spi->exchange_byte((1 << RTR | dataLength)); // RTR frame has length, but no header
    else {
        spi->exchange_byte(dataLength);

        for(int i=0; i< dataLength; i++) {
            spi->exchange_byte(message->data[i]); // send x length amount of data
        }
    }
    // end loading register
    cs.set_state(GPIO_HIGH);

    // start transmission of data from one of the 3 TX buffers
    cs.set_state(GPIO_LOW);
    addr = (addr == 0) ? 1: addr; // TX2 = bit 2 , TX1 = bit 1 , TX0 = bit 0
    spi->exchange_byte((SPI_RTS | addr));  // request to send, initiate message transmission
    cs.set_state(GPIO_HIGH);

    return true;
}
