/*
 * mcp2515_def.hpp
 *
 *  Created on: Apr 10, 2016
 *      Author: YuYu
 */

#ifndef L5_APPLICATION_CAN_MCP2515_DEF_HPP_
#define L5_APPLICATION_CAN_MCP2515_DEF_HPP_

// SPI Opcodes
#define SPI_RESET       0xC0
#define SPI_READ        0x03
#define SPI_READ_RX     0x90
#define SPI_WRITE       0x02
#define SPI_WRITE_TX    0x40
#define SPI_RTS         0x80
#define SPI_READ_STATUS 0xA0
#define SPI_RX_STATUS   0xB0
#define SPI_BIT_MODIFY  0x05

// CAN CONTROLLER REGISTERS

/*
 * RxFnSIDH - Filter n Standard Identifier High bits registers
 * bit [7:0]  -  SID: Standard Identifier Filter bits [10:3]
 * These bits hold the filter bits to be applied to bits [10:3] of Standard Identifier portion of
 * received message
 */
typedef enum {
    RXF0SIDH = 0x0,
    RXF1SIDH = 0x04,
    RXF2SIDH = 0x08,
    RXF3SIDH = 0x10,
    RXF4SIDH = 0x14,
    RXF5SIDH = 0x18
}RXFnSIDH;

/*
 * RXFnSIDL - Filter n Standard Identifier Low bits registers
 * bit [7:5] - SID: Standard Identifier Filter bits [2:0]
 * bit 4    - Reserved - 0
 * bit 3    - EXIDE: Extended Identifier Enable bit ;  1: Extended frames | 0: Standard frames
 * bit 2    - Reserved - 0
 * bit [1:0]- EID: Extended Identifier bits [17:16] , Hold filter bits to be applied to bits [17:16] of
 *              Extended Identifier portion of received message
 */
typedef enum {
    RXF0SIDL    = 0x01,
    RXF1SIDL    = 0x05,
    RXF2SIDL    = 0x09,
    RXF3SIDL    = 0x11,
    RXF4SIDL    = 0x15,
    RXF5SIDL    = 0x19
}RXFnSIDL;

/*
 * RxFnEID8 - Filter n Extended Identifier High bits registers
 * bit [7:0]  -  EID: Extended Identifier Filter bits [15:8]
 * These bits hold the filter bits to be applied to bits [15:8] of Extended Identifier portion of
 * received message
 */
typedef enum {
    RXF0EID8    = 0x02,
    RXF1EID8    = 0x06,
    RXF2EID8    = 0x0A,
    RXF3EID8    = 0x12,
    RXF4EID8    = 0x16,
    RXF5EID8    = 0x1A
}RXFnEID8;

/*
 * RxFnEID0 - Filter n Extended Identifier Low bits registers
 * bit [7:0]  -  EID: Extended Identifier Filter bits [7:0]
 * These bits hold the filter bits to be applied to bits [7:0] of Extended Identifier portion of
 * received message
 */
typedef enum {
    RXF0EID0    = 0x03,
    RXF1EID0    = 0x07,
    RXF2EID0    = 0x0B,
    RXF3EID0    = 0x13,
    RXF4EID0    = 0x17,
    RXF5EID0    = 0x1B
}RXFnEID0;

/*
 * RxMnSIDH - Mask n Standard Identifier High bits registers
 * bit [7:0] SID: Standard Identifier Mask bits [10:3]
 * These bits hold the mask bits to be applied to bits [10:3] of Standard Identifier portion of
 * a received message
 */
typedef enum {
    RXM0SIDH    = 0x20,
    RXM1SIDH    = 0x24
}RXMnSIDH;

/*
 * RxMnSIDL - Mask n Standard Identifier Low bits register
 * bit [7:5] - SID: Standard Identifier Mask bits [2:0]
 * bit [4:2] - Reserved - 0
 * bit [1:0]- EID: Extended Identifier Mask bits [17:16] ,
 *              Hold mask bits to be applied to bits [17:16] of
 *                  Extended Identifier portion of received message
 */
typedef enum {
    RXM0SIDL    = 0x21,
    RXM1SIDL    = 0x25
}RXMnSIDL;


/*
 * RxMnEID8 - Mask n Extended Identifier High bits registers
 * bit [7:0] - EID: Extended Identifier bits [15:8]
 * Hold the mask bits to be applied to bits [15:8] of Extended Identifier portion of received message
 */
typedef enum {
    RXM0EID8    = 0x22,
    RXM1EID8    = 0x26
}RXMnEID8;

/*
 * RxMnEID0 - Mask n Extended Identifier Low bits registers
 * bit [7:0] - EID: Extended Identifier bits [7:0]
 * Hold the mask bits to be applied to bits [7:0] of Extended Identifier portion of received message
 */
typedef enum {
    RXM0EID0    = 0x23,
    RXM1EID0    = 0x27
}RXMnEID0;

/*
 * TxBnCTRL - Transmit buffer n Control Register
 * bit 7    - Reserved - 0
 * bit 6    - ABTF: Message Aborted Flag bit :  1 - Aborted  , 0 - transmission successful
 * bit 5    - MLOA: Message Lost Arbitration bit: 1 - Lost arbitration while being sent
 * bit 4    - TXERR: Transmission Error Detected bit:  1 - Bus error occurred while message being transmitted
 * bit 3    - TXREQ: Message Transmit Request bit:
 *              1 - Buffer currently pending transmission; set to request message be transmitted
 *                  ; bit automatically cleared when message is sent
 *              0 - Buffer is not currently pending transmission; clear to abort
 * bit 2:   - Reserved - 0
 * bit [1:0]- Transmit Buffer Priority - 2'b11 - Highest Priority -----> 2'b0 - Lowest Priority
 */
typedef enum {
    TXB0CTRL    = 0x30,
    TXB1CTRL    = 0x40,
    TXB2CTRL    = 0x50
}TXBnCTRL;

/*
 * TxRTSCTRL - TxnRTS Pin Control and Status Register
 * bit [7:6] - Reserved - 0
 * bit 5     - B2RTS - reads TX2RTSn pin when in Digital Input mode ; Read as 0 when in 'Request to Send' mode
 * bit 4     - B1RTS - reads TX1RTSn pin when in Digital Input mode ; Read as 0 when in 'Request to Send' mode
 * bit 3     - B0RTS - reads TX0RTSn pin when in Digital Input mode ; Read as 0 when in 'Request to Send' mode
 * bit 2     - B2RTSM - TX2RTSn: 1 - used to request message transmission on TXB2 buffer ; 0 - Digital Input
 * bit 1     - B1RTSM - TX1RTSn: 1 - used to request message transmission on TXB1 buffer ; 0 - Digital Input
 * bit 0     - B0RTSM - TX0RTSn: 1 - used to request message transmission on TXB0 buffer ; 0 - Digital Input
 */
#define TXRTSCTRL   0x0D

/*
 * TxBnSIDH - Transmit Buffer n Standard Identifier High bits registers
 * bit [7:0] - SID: Standard Identifier bits [10:3]
 */
typedef enum {
    TXB0SIDH    = 0x31,
    TXB1SIDH    = 0x41,
    TXB2SIDH    = 0x51
}TxBnSIDH;


/*
 * TxBnSIDL - Transmit buffer n Standard Identifier Low bits registers
 * bit [7:5] - SID - Standard Identifier bits [2:0]
 * bit 4     - Reserved - 0
 * bit 3     - EXIDE: Extended Identifier Enable bit ; 1 - Extended Identifier , 0 - Standard Identifier
 * bit 2     - Reserved - 0
 * bit [1:0] - EID: Extended Identifier bits [17:16]
 */
typedef enum {
    TXB0SIDL    = 0x32,
    TXB1SIDL    = 0x42,
    TXB2SIDL    = 0x52
}TxBnSIDL;

/*
 * TxBnEID8 - Transmit buffer n Extended Identifier High bits registers
 * bit [7:0] - EID: Extended Identifier bits [15:8]
 */
typedef enum {
    TXB0EID8    = 0x33,
    TXB1EID8    = 0x43,
    TXB2EID8    = 0x53
}TXBnEID8;

/*
 * TxBnEID0 - Transmit buffer n Extended Identifier Low bits register
 * bit [7:0] - Extended Identifier bits [7:0]
 */
typedef enum {
    TXB0EID0    = 0x34,
    TXB1EID0    = 0x44,
    TXB2EID0    = 0x54
}TXBnEID0;

/*
 * TxBnDLC - Transmit buffer n Data Length Code
 * bit 7    - Reserved - 0
 * bit 6    - RTR: Remote Transmission Request bit; 1 - Message will be Remote Transmit Request ; 0 - Data Frame
 * bit [5:4] - Reserved - 0
 * bit [3:0] - DLC: Data Length Code [3:0] ; Number of data bytes to be transmitted (0 - 8 bytes)
 */
typedef enum {
    TXB0DLC     = 0x35,
    TXB1DLC     = 0x45,
    TXB2DLC     = 0x55
}TXBnDLC;

/*
 * TxB0Dm - Transmit Buffer 0 Data Byte m
 * bit [7:0] - Transmit Buffer 0 Data Field Bytes m
 */
typedef enum {
    TXB0D0      = 0x36,
    TXB0D1      = 0x37,
    TXB0D2      = 0x38,
    TXB0D3      = 0x39,
    TXB0D4      = 0x3A,
    TXB0D5      = 0x3B,
    TXB0D6      = 0x3C,
    TXB0D7      = 0x3D,
}TXB0Dm;

/*
 * TxB1Dm - Transmit Buffer 1 Data Byte m
 * bit [7:0] - Transmit Buffer 1 Data Field Bytes m
 */
typedef enum {
    TXB1D0      = 0x46,
    TXB1D1      = 0x47,
    TXB1D2      = 0x48,
    TXB1D3      = 0x49,
    TXB1D4      = 0x4A,
    TXB1D5      = 0x4B,
    TXB1D6      = 0x4C,
    TXB1D7      = 0x4D
}TXB1Dm;

/*
 * TxB2Dm - Transmit Buffer 2 Data Byte m
 * bit [7:0] - Transmit Buffer 2 Data Field Bytes m
 */
typedef enum {
    TXB2D0      = 0x56,
    TXB2D1      = 0x57,
    TXB2D2      = 0x58,
    TXB2D3      = 0x59,
    TXB2D4      = 0x5A,
    TXB2D5      = 0x5B,
    TXB2D6      = 0x5C,
    TXB2D7      = 0x5D
}TxB2Dm;


/*
 * RxB0CTRL - Receive Buffer 0 Control
 * bit 7    - Reserved - 0
 * bit [6:5]- RXM: Receive Buffer Operating Mode bits
 *              2'b11 - Turn mask/filter off; receive any message
 *              2'b10 - Receive only valid messages w/ extended identifier that meet filter criteria
 *              2'b01 - Receive only valid messages with standard identifier that meet filter criteria
 *              2'b00 - Receive all valid messages that meet filter criteria
 * bit 4    - Reserved - 0
 * bit 3    - RXRTR: Received Remote Transfer Request bit
 *              1'b1 - Remote Transfer Request Received
 *              1'b0 - No Remote Transfer Request Received
 * bit 2    - BUKT: Rollover Enable bit
 *              1'b1 - RXB0 message will rollover and be written to RXB1 if RXB0 is full
 *              1'b0 - Rollover disabled
 * bit 1    - BUKT1: Read-only Copy of BUKT bit (used internally by the MCP2515)
 * bit 0    - FILHIT: Filter Hit bit
 *              1'b1 - Acceptance Filter 1 (RXF1)
 *              1'b0 - Acceptance Filter 0 (RXF0)
 */
#define RXB0CTRL    0x60


/*
 * RxB1CTRL - Receive Buffer 1 Control
 * bit 7    - Reserved - 0
 * bit [6:5]- RXM: Receive Buffer operating mode bits
 *              2'b11 - Turn mask/filter off; receive any message
 *              2'b10 - Receive only valid messages w/ extended identifier that meet filter criteria
 *              2'b01 - Receive only valid messages with standard identifier that meet filter criteria
 *              2'b00 - Receive all valid messages that meet filter criteria
 * bit 4    - Reserved - 0
 * bit 3    - RXRTR: Received Remote Transfer Request bit
 *              1'b1 - Remote Transfer Request Received
 *              1'b0 - No Remote Transfer Request Received
 * bit [2:0]- FILHIT: Filter Hit bit
 *              3'b101 - Acceptance Filter 5 (RXF5)
 *              3'b100 - Acceptance Filter 3 (RXF4)
 *              3'b011 - Acceptance Filter 2 (RXF3)
 *              3'b010 - Acceptance Filter 1 (RXF2)
 *              3'b001 - Acceptance Filter 1 (RXF1) (Only if BUKT bit set in RXB0CTRL)
 *              3'b000 - Acceptance Filter 0 (RXF0) (Only if BUKT bit set in RXB0CTRL)
 */
#define RXB1CTRL    0x70

/*
 * BFPCTRL  - RXnBF Pin Control and Status Register
 * bit [7:6] - Reserved - 0
 * bit 5     - B1BFS -  RX1BFn pin State (Digital Output mode) ; Read as 0 when RX1BF configured as interrupt
 * bit 4     - B0BFS -  RX0BFn pin State (Digital Output mode) ; Read as 0 when RX0BF configured as interrupt
 * bit 3     - B1BFE -  RX1BFn pin function enable bit
 *                          1'b1 - enabled; mode determined by B1BFM bit
 *                          1'b0 - disabled; Hi-Z
 * bit 2     - B0RBFE - RX0BF pin function enable bit
 *                          1'b1 - enabled; mode determined by B0BFM bit
 *                          1'b0 - disabled; Hi-Z
 * bit 1     - B1RBFM - RX1BF Operation Mode bit
 *                          1'b1 - Pin used as interrupt when valid message loaded into RXB1
 *                          1'b0 - Digital Output mode
 * bit 0     - B0RBFM - RX0BF Operation Mode bit
 *                          1'b1 - Pin used as interrupt when valid message loaded into RXB0
 *                          1'b0 - Digital Output mode
 */
#define BFPCTRL     0x0C

/*
 * RxBnSIDH - Receive buffer n Standard Identifier High bits registers
 * bit [7:0] - SID: Standard Identifier bits [10:3]
 * Contain the eight MSB of Standard Identifier for the received message
 */
typedef enum {
    RXB0SIDH    = 0x61,
    RXB1SIDH    = 0x71
}RXBnSIDH;


/*
 * RxBnSIDL - Receive buffer n Standard Identifier Low bits registers
 * bit [7:5] - SID: Standard Identifier bits [2:0]
 * bit 4     - SRR: Standard Frame Remote Transmit Request bit (valid only if IDE bit = 0)
 *              1'b1 - Standard Frame Remote Transmit Request Received
 *              1'b0 - Standard Data Frame Received
 * bit 3     - IDE: Extended Identifier Flag bit - whether message was a Standard or Extended Frame
 *              1'b1 - Received message Extended frame
 *              1'b0 - Receive message Standard frame
 * bit 2     - Reserved - 0
 * bit [1:0] - EID: Extended Identifier bits [17:16] - contains two MSB of Extended Identifier for received
 *
 */
typedef enum {
    RXB0SIDL    = 0x62,
    RXB1SIDL    = 0x72
}RxBnSIDL;


/*
 * RxBnEID8 - Receive Buffer n Extended Identifier High bits registers
 * bit [7:0] - EID: Extended Identifier bits [15:8] ; holds EID [15:8]
 */
typedef enum {
    RXB0EID8    = 0x63,
    RXB1EID8    = 0x73
}RXBnEID8;

/*
 * RxBnEID0 - Receive Buffer n Extended Identifier Low bits registers
 * bit [7:0] - EID: Extended Identifier bits [7:0] ; holds EID [7:0]
 */
typedef enum {
    RXB0EID0    = 0x64,
    RXB1EID0    = 0x74
}RXBnEID0;


/*
 * RxBnDLC - Receive buffer n Data Length Code
 * bit 7    - Reserved - 0
 * bit 6    - RTR: Extended Frame Remote Transmission Request bit; valid only when RXBnSIDL.IDE = 1
 *               1'b1 - Extended Frame Message will be Remote Transmit Request Received
 *               1'b0 - Extended Data Frame
 * bit [5:4] - Reserved - 0
 * bit [3:0] - DLC: Data Length Code [3:0] ; Number of data bytes that were received (0 - 8 bytes)
 */
typedef enum {
    RXB0DLC     = 0x65,
    RXB1DLC     = 0x75
}RXBnDLC;

/*
 * RxB0Dm - Receive Buffer 0 Data Byte m
 * bit [7:0] - Receive buffer 0 Data Field Byte m - eight bytes containing data bytes for the received message
 */
typedef enum {
    RXB0D0      = 0x66,
    RXB0D1      = 0x67,
    RXB0D2      = 0x68,
    RXB0D3      = 0x69,
    RXB0D4      = 0x6A,
    RXB0D5      = 0x6B,
    RXB0D6      = 0x6C,
    RXB0D7      = 0x6D
}RXB0Dm;


/*
 * RxB1Dm - Receive Buffer 1 Data Byte m
 * bit [7:0] - Receive buffer 1 Data Field Byte m - eight bytes containing data bytes for the received message
 */
typedef enum {
    RXB1D0      = 0x76,
    RXB1D1      = 0x77,
    RXB1D2      = 0x78,
    RXB1D3      = 0x79,
    RXB1D4      = 0x7A,
    RXB1D5      = 0x7B,
    RXB1D6      = 0x7C,
    RXB1D7      = 0x7D
}RXB1Dm;


//***********************TO DO'S ****************************//


/*
 * CANSTAT - CAN status register
 * bit [7:5] - OPMODE - operation mode bits [2:0]
 *                  3'b000 - Normal Operation mode
 *                  3'b001 - Sleep mode
 *                  3'b010 - Loopback mode
 *                  3'b011 - Listen-only mode
 *                  3'b100 - Configuration mode
 *  bit 4    -  Reserved - 0
 *  bit [3:1]-  ICOD - Interrupt Flag Code bits [2:0]
 *                  3'b000 - No interrupt
 *                  3'b001 - Error interrupt
 *                  3'b010 - Wake-Up interrupt
 *                  3'b011 - TXB0 interrupt
 *                  3'b100 - TXB1 interrupt
 *                  3'b101 - TXB2 interrupt
 *                  3'b110 - RXB0 interrupt
 *                  3'b111 - RXB1 interrupt
 *  bit 0    - Reserved - 0
 */
#define CANSTAT     0x0E

/*
 * CANCTRL - CAN control register
 * bit [7:5] - REQOP - Request Operation Mode bits [2:0] - On power-up bits are 3'b111
 *                  3'b000 - Normal Operation mode
 *                  3'b001 - Sleep mode
 *                  3'b010 - Loopback mode
 *                  3'b011 - Listen-only mode
 *                  3'b100 - Configuration mode
 * bit 4     -  ABAT - Abort all pending transactions
 *                  1'b1 - Request abort all pending transmit buffers
 *                  1'b0 - Terminate request to abort all transaction
 * bit 3     -  OSM - One Shot Mode bit
 *                  1'b0 - Disabled, messages will reattempt transmission, if required
 *                  1'b1 - Enabled, message will only attempt to transmit one time
 * bit 2     -  CLKEN - CLKOUT Pin Enable bit
 *                  1'b0 -  CLKOUT Pin disabled - in Hi-z
 *                  1'b1 -  CLKOUT Pin enabled
 * bit [1:0] -  CLKPRE - CLKOUT Pin Prescaler bits [1:0]
 *                  2'b00 - Fclkout = System clock/1
 *                  2'b01 - Fclkout = System clock/2
 *                  2'b10 - Fclkout = System clock/4
 *                  2'b11 - Fclkout = System clock/8
 */
#define CANCTRL     0x0F



/*
 * TEC - Transmit Error Counter
 * bit [7:0] - Transmit Error Count bits
 */
#define TEC         0x1C

/*
 * REC - Receive Error Counter
 * bit [7:0] - Receive Error Count bits
 */
#define REC         0x1D

// =========== CNF3, CNF2, CNF1 registers control the bit timing for the CAN bus interface =========
// =========== Can only be configured in Configuration Mode =========
/*
 * CNF3 - Configuration 1
 * bit 7 - SOF - Start of Frame bit
 *      if CANCTRL.CLKEN == 1
 *          1'b0 - CLKOUT pin enabled for clockout function
 *          1'b1 - CLKOUT pin enabled for SOF signal
 *      else Bit is don't care
 *  bit 6 - WAKFIL - Wake-Up Filter bit
 *      1'b0 - Wake-Up filter disabled
 *      1'b1 - Wake-Up filter enabled
 *  bit [5:3] - Reserved - 0
 *  bit [2:0] - PHSEG2 - PS2 length bits [2:0]
 *      (PHSEG2 + 1) x Tq
 *      Minimum valid setting for PS2 is 2 Tq
 */
#define CNF3        0x28

/*
 * CNF2 - Configuration 1
 * bit 7 - BTLMODE - PS2 bit Time Length bit
 *      1'b0 - Length of PS2 is the greater of PS1 and IPT (2 Tq)
 *      1'b1 - Length of PS2 determined by [PHSEG22:PHSEG20] bits in CNF3
 * bit 6 - SAM - Sample Point Configuration bit
 *      1'b0 - Bus line is sampled once at the sample point
 *      1'b1 - Bus line is sampled 3 times at the sample point
 * bit [5:3] - PHSEG1 - PS1 Length bits [2:0] - (PHSEG1 + 1) x Tq
 * bit [2:0] - PRSEG - Propagation Segment Length bits [2:0] - (PRSEG + 1) x Tq
 *
 *
 */
#define CNF2        0x29

/*
 *CNF1 - Configuration 1
 * bit [7:6] - SJW - Synchronization Jump Width Length bits [1:0]
 *                  2'b11 - Length = 4 x Tq
 *                  2'b10 - Length = 3 x Tq
 *                  2'b01 - Length = 2 x Tq
 *                  2'b00 - Length = 1 x Tq
 * bit [5:0] - BRP - Baud Rate Prescaler bits [5:0]
 *                  Tq = 2 x (BRP + 1)/Fosc
 */
#define CNF1        0x2A

/*
 * CANINTE - Interrupt Enable - if bit x = 0 , Disabled
 * bit 7    - MERRE - Message Error Interrupt Enable bit
 *              1'b1 - Interrupt on error during message reception and transmission
 * bit 6    - WAKIE - Wake-up Interrupt Enable bit
 *              1'b1 - Interrupt on CAN bus activity
 * bit 5    - ERRIE - Error Interrupt Enable bit (multiple sources in EFLG register)
 *              1'b1 - Interrupt on EFLG error condition change
 * bit 4    - TX2IE - Transmit Buffer 2 Empty Interrupt Enable bit
 *              1'b1 - Interrupt on TXB2 becoming empty
 * bit 3    - TX1IE - Transmit Buffer 1 Empty Interrupt Enable bit
 *              1'b1 - Interrupt on TXB1 becoming empty
 * bit 2    - TX0IE - Transmit Buffer 0 Empty Interrupt Enable bit
 *              1'b1 - Interrupt on TXB0 becoming empty
 * bit 1    - RX1IE - Receive Buffer 1 Full Interrupt Enable bit
 *              1'b1 - Interrupt when message received in RXB1
 * bit 0    - RX0IE - Receive Buffer 0 Full Interrupt Enable bit
 *              1'b1 - Interrupt when message received in RXB1
 */
#define CANINTE     0x2B

/*
 * CANINTF - Interrupt Flag - if bit x = 0 , No interrupt pending ,
 *              if bit x = 1, Interrupt pending, must be cleared by MCU to reset interrupt condition
 * bit 7    - MERRF - Message Error Interrupt Flag bit
 * bit 6    - WAKIF - Wake-up Interrupt Flag bit
 * bit 5    - ERRIF - Error Interrupt Flag bit (multiple sources in EFLG register)
 * bit 4    - TX2IF - Transmit Buffer 2 Empty Interrupt Flag bit
 * bit 3    - TX1IF - Transmit Buffer 1 Empty Interrupt Flag bit
 * bit 2    - TX0IF - Transmit Buffer 0 Empty Interrupt Flag bit
 * bit 1    - RX1IF - Receive Buffer 1 Full Interrupt Flag bit
 * bit 0    - RX0IF - Receive Buffer 0 Full Interrupt Flag bit
 */
#define CANINTF     0x2C


/*
 * EFLG - Error Flag
 * bit 7    - RX1OVR -  Receive Buffer 1 overflow flag bit
 *                      - set when a valid message is received for RXB1 and CANINTF.RX1F = 1
 *                      - must be reset by MCU
 * bit 6    - RX0OVR -  Receive Buffer 0 overflow flag bit
 *                      - set when a valid message is received for RXB0 and CANINTF.RX0F = 1
 *                      - must be reset by MCU
 * bit 5    - TXB0 -    Bus-Off Error Flag bit
 *                      - Bit set when TEC reaches 255
 *                      - Reset after a successful bus recovery sequence
 * bit 4    - TEXP -    Transmit Error-Passive Flag bit
 *                      - Set when REC is equal or greater than 128
 *                      - Reset when TEC is less than 128
 * bit 3    - RXEP -    Receive Error-Passive Flag bit
 *                      - Set when REC is equal to or greater than 128
 *                      - Reset when REC is less than 128
 * bit 2    - TXWAR -   Transmit Error Warning Flag bit
 *                      - Set when TEC is equal to or greater than 96
 *                      - Reset when TEC is less than 96
 * bit 1    - RXWAR -   Receive Error Warning Flag bit
 *                      - Set when REC is equal to or greater than 96
 *                      - Reset when REC is less than 96
 * bit 0    - EWARN -   Error Warning Flag bit
 *                      - Set when TEC or REC is equal to or greater than 96 (TXWAR or TXWAR = 1)
 *                      - Reset when both REC and TEC are less than 96
 */
#define EFLG        0x2D




// CONTROL REGISTERS

// BFPCTRL : 0x0C
typedef enum {
    B1BFS       = 5,
    B0BFS       = 4,
    B1BFE       = 3,
    B0BFE       = 2,
    B1BFM       = 1,
    B0BFM       = 0
}BFPCTRL_bits;

// TXRTSCTRL : 0x0D
typedef enum {
    B2RTS       = 5,
    B1RTS       = 4,
    B0RTS       = 3,
    B2RTSM      = 2,
    B1RTSM      = 1,
    B0RTSM      = 0
}TXRTSCTRL_bits;

// CANSTAT : 0xE
typedef enum {
    OPMOD       = 5,
    ICOD        = 1,
}CANSTAT_bits;

// CANCTRL : 0xF
typedef enum {
    REQOP       = 5,
    ABAT        = 4,
    CLKEN       = 2,
    CLKPRE      = 0
}CANCTRL_bits;


// CNF3 : 0x28
typedef enum {
    SOF         = 7,
    WAKFIL      = 6,
    PHSEG2      = 0
}CNF3_bits;

// CNF2 : 0x29
typedef enum {
    BTLMODE     = 7,
    SAM         = 6,
    PHSEG1      = 3,
    PHSEG       = 0
}CNF2_bits;

// CNF1 : 0x2A
typedef enum {
    SJW     = 6,
    BRP0    = 0
}CNF1_bits;

// CANINTE : 0x2B
typedef enum {
    MERRE       = 7,
    WAKIE       = 6,
    ERRIE       = 5,
    TX2IE       = 4,
    TX1IE       = 3,
    TX0IE       = 2,
    RX1IE       = 1,
    RX0IE       = 0
}CANINTE_bits;

// CANINTF : 0x2C
typedef enum {
    MERRF       = 7,
    WAKIF       = 6,
    ERRIF       = 5,
    TX2IF       = 4,
    TX1IF       = 3,
    TX0IF       = 2,
    RX1IF       = 1,
    RX0IF       = 0
}CANINTF_bits;

// EFLG : 0x2D
typedef enum {
    RX1OVR      = 7,
    RX0OVR      = 6,
    TXB0        = 5,
    TXEP        = 4,
    RXEP        = 3,
    TXWAR       = 2,
    RXWAR       = 1,
    EWARN       = 0
}EFLG_bits;


// TXBnCTRL (n = 0, 1, 2) :  0x30 , 0x40 , 0x50
typedef enum {
    ABTF        = 6,
    MLOA        = 5,
    TXERR       = 4,
    TXREQ       = 3,
    TXP         = 0
}TXBnCTRL_bits;

// RXB0CTRL : 0x60
typedef enum {
    RXM0        = 5,
    RXRTR0      = 3,
    BUKT        = 2,
    BUKT1       = 1,
    FILHIT0     = 0
}RXB0CTRL_bits;

// TXBnSIDL (n = 0, 1) : 0x32, 0x42, 0x52
typedef enum {
    txSID           = 5,
    EXIDE           = 3,
    txEID           = 0
}TXBnSIDL_bits;


// RXB1CTRL : 0x70
typedef enum {
    RXM1        = 5,
    RXRTR1      = 3,
    FILHIT1     = 0
}RXB1CTRL_bits;

// RXBnSIDL (n = 0, 1) : 0x62, 0x72
typedef enum {
    rxSID       = 5,
    SRR         = 4,
    IDE         = 3,
    EID         = 0
}RXBnSIDL_bits;

//RXBnDLC (n = 0, 1) : 0x65, 0x75
typedef enum {
    RTR         = 6,
    DLC         = 0
}RXBnDLC_bits;




#endif /* L5_APPLICATION_CAN_MCP2515_DEF_HPP_ */
