/*********************************************************************************************************************
 * The MIT License (MIT)                                                                                             *
 * Copyright (c) 2012-2016, 2022 Maksim Salau                                                                        *
 *                                                                                                                   *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated      *
 * documentation files (the "Software"), to deal in the Software without restriction, including without limitation   *
 * the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and  *
 * to permit persons to whom the Software is furnished to do so, subject to the following conditions:                *
 *                                                                                                                   *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions     *
 * of the Software.                                                                                                  *
 *                                                                                                                   *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO  *
 * THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE    *
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF         *
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS *
 * IN THE SOFTWARE.                                                                                                  *
 *********************************************************************************************************************/

#ifndef RL78_H__
#define RL78_H__

#define CMD_RESET               0x00
#define CMD_BLOCK_ERASE         0x22
#define CMD_PROGRAMMING         0x40
#define CMD_VERIFY              0x13
#define CMD_BLOCK_BLANK_CHECK   0x32
#define CMD_BAUD_RATE_SET       0x9A
#define CMD_SILICON_SIGNATURE   0xC0
#define CMD_SECURITY_SET        0xA0
#define CMD_SECURITY_GET        0xA1
#define CMD_SECURITY_RELEASE    0xA2
#define CMD_CHECKSUM            0xB0

#define STATUS_COMMAND_NUMBER_ERROR     0x04
#define STATUS_PARAMETER_ERROR          0x05
#define STATUS_ACK                      0x06
#define STATUS_CHECKSUM_ERROR           0x07
#define STATUS_VERIFY_ERROR             0x0F
#define STATUS_PROTECT_ERROR            0x10
#define STATUS_NACK                     0x15
#define STATUS_ERASE_ERROR              0x1A
#define STATUS_IVERIFY_BLANK_ERROR      0x1B
#define STATUS_WRITE_ERROR              0x1C

#define SOH 0x01
#define STX 0x02
#define ETB 0x17
#define ETX 0x03

#define RL78_RX 15
#define RL78_TX 16
#define RL78_RST 14

#define RL78_BAUD_115200     0x00
#define RL78_BAUD_250000     0x01
#define RL78_BAUD_500000     0x02
#define RL78_BAUD_1000000    0x03

#define CODE_OFFSET             (0U)
#define DATA_OFFSET             (0x000F1000U)

#define MAX_RESPONSE_LENGTH 32

#define RESPONSE_OK                     (0)
#define RESPONSE_CHECKSUM_ERROR         (-1)
#define RESPONSE_FORMAT_ERROR           (-2)
#define RESPONSE_EXPECTED_LENGTH_ERROR  (-3)

#define SET_MODE_1WIRE_UART 0x3A
#define SET_MODE_2WIRE_UART 0x00

#define RL78_MIN_VOLTAGE    1.8f
#define RL78_MAX_VOLTAGE    5.5f

#define MODE_UART         1
#define MODE_UART_1       0
#define MODE_UART_2       MODE_UART
#define MODE_RESET        2
#define MODE_RESET_DTR    0
#define MODE_RESET_RTS    MODE_RESET
#define MODE_MAX_VALUE    (MODE_UART | MODE_RESET)
#define MODE_MIN_VALUE    0
#define MODE_INVERT_RESET 0x80

#define PROTOCOL_VERSION_A 0 /* most RL78 chips */
/* Protocol B = ??? Is this the G10 protocol? */
#define PROTOCOL_VERSION_C 2 /* RL78/G23 */
#define PROTOCOL_VERSION_D 3 /* RL78/F24 */

#include <SoftwareSerial.h>

int rl78_reset_init( int wait, int baud, int mode, float voltage);
int rl78_reset( int mode);
int rl78_send_cmd( int cmd, const void *data, int len);
int rl78_send_data( const void *data, int len, int last);
int rl78_recv( void *data, int *len, int explen);
int rl78_cmd_reset(SoftwareSerial fd);
int rl78_cmd_baud_rate_set( int baud, float voltage);
int rl78_cmd_silicon_signature( char device_name[11], unsigned int *code_size, unsigned int *data_size);
int rl78_cmd_block_erase( unsigned int address);
int rl78_cmd_block_blank_check( unsigned int address_start, unsigned int address_end);
int rl78_cmd_checksum( unsigned int address_start, unsigned int address_end);
int rl78_cmd_programming( unsigned int address_start, unsigned int address_end, const void *rom, int proto_ver);
unsigned int rl78_checksum(const void *rom, unsigned int len);
int rl78_cmd_verify( unsigned int address_start, unsigned int address_end, const void *rom);
int rl78_program( unsigned int address, const void *data, unsigned int size, unsigned blksz, int proto_ver);
int rl78_erase( unsigned int start_address, unsigned int size, unsigned blksz);
int rl78_verify( unsigned int address, const void *data, unsigned int size, int blksz);

#endif  // RL78_H__
