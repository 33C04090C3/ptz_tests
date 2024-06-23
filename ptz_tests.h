/******************************************************************************/
/* PTZ TESTS                                                                  */
/******************************************************************************/
#pragma once

// macro to make the ugly pointer casting look nicer
#define UINT32(x)( *(uint32_t*)(x) )

// Most of these are taken from the Hi3520 data sheet as the Hi3521 data sheet is very difficult to find
#define HI3521_UART1_BASE 0x20090000

#define UART_DR           0x00
#define UART_RSR          0x04
#define UART_FR           0x18
#define UART_IBRD         0x24
#define UART_FBRD         0x28
#define UART_LCR_H        0x2C
#define UART_CR           0x30

#define UART_IFLS         0x34
#define UART_IMSC         0x38
#define UART_RIS          0x3C
#define UART_MIS          0x40
#define UART_ICR          0x44
#define UART_DMACR        0x48

// It appears to be necessary to set bit 0 at one of these addresses before writing to the PTZ serial port.
// However, which address is used will depend on an internal DVR configuration value.
#define ADDRESS_1         0x20210100
#define ADDRESS_2         0x201E0004
#define ADDRESS_3         0x201A0004
#define ADDRESS_4         0x201C0004