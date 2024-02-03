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

#include <Arduino.h>

#include "rl78.h"

SoftwareSerial SerialRL78;

extern int verbose_level;
static unsigned char communication_mode;

void rs78flash(){
    SerialRL78.begin(115200, SWSERIAL_8N1, RL78_RX, RL78_TX);
    pinMode(RL78_RST,OUTPUT);
}

static void rl78_set_reset(uint8_t pin, int value)
{
    int level  = (MODE_INVERT_RESET) ? !value : value;
    digitalWrite(pin,level);
}

void serial_flush(SoftwareSerial fd){
    SerialRL78.flush();
}


int rl78_reset_init( int wait, int baud, int mode, float voltage)
{
    unsigned char r;
        r = SET_MODE_1WIRE_UART;
        communication_mode = 1;
    rl78_set_reset(RL78_RST, 0);                            /* RESET -> 0 */
    //serial_set_txd( 0);                                  /* TOOL0 -> 0 */
    if (wait)
    {
        printf("Turn MCU's power on and press any key...");
        delay(100);
        printf("\n");
    }
    SerialRL78.flush();
    delay(1);
    rl78_set_reset(RL78_RST, 1);                            /* RESET -> 1 */
    usleep(3000);
    //serial_set_txd( 1);                                  /* TOOL0 -> 1 */
    delay(1);
    SerialRL78.flush();
    if (3 <= verbose_level)
    {
        printf("Send 1-byte data for setting mode\n");
    }
    //serial_write( &r, 1);
    SerialRL78.write(&r,1);
    if (1 == communication_mode)
    {
        //serial_read( &r, 1);
        SerialRL78.readBytes((char *)&r, 1);
    }
    delay(1);
    return rl78_cmd_baud_rate_set( baud, voltage);
}

int rl78_reset( int mode)
{
    //serial_set_txd( 1);                                  /* TOOL0 -> 1 */
    rl78_set_reset(RL78_RST, 0);                            /* RESET -> 0 */
    delay(10);
    delay(10);
    rl78_set_reset(RL78_RST, 1);                            /* RESET -> 1 */
    return 0;
}

static
int checksum(unsigned char *data, int len)
{
    unsigned int sum = 0;
    const unsigned char *p = data;
    for (; len; --len)
    {
        sum -= *p++;
    }
    return sum & 0x00FF;
}

int rl78_send_cmd( int cmd, const void *data, int len)
{
    if (255 < len)
    {
        return -1;
    }
    if (!data && len)
    {
        return -2;
    }

    unsigned char buf[len + 5];
    buf[0] = SOH;
    buf[1] = (len + 1) & 0xFFU;
    buf[2] = cmd;
    if (len)
        memcpy(&buf[3], data, len);
    buf[len + 3] = checksum(&buf[1], len + 2);
    buf[len + 4] = ETX;
    //int ret = serial_write( buf, sizeof buf);
    int ret = SerialRL78.write(buf, sizeof(buf));
    // Read back echo
    if (1 == communication_mode)
    {
        SerialRL78.readBytes((char *)&buf, sizeof(buf));
    }
    return ret;
}

int rl78_send_data( const void *data, int len, int last)
{
    if (256 < len)
    {
        return -1;
    }
    unsigned char buf[len + 4];
    buf[0] = STX;
    buf[1] = len & 0xFFU;
    memcpy(&buf[2], data, len);
    buf[len + 2] = checksum(&buf[1], len + 1);
    buf[len + 3] = last ? ETX : ETB;
    //int ret = serial_write( buf, sizeof buf);
    int ret = SerialRL78.write(buf,sizeof(buf));
    // Read back echo
    if (1 == communication_mode)
    {
        //serial_read( buf, sizeof buf);
        SerialRL78.readBytes((char *)&buf, sizeof(buf));
    }
    return ret;
}

int rl78_recv( void *data, int *len, int explen)
{
    unsigned char in[MAX_RESPONSE_LENGTH];
    int data_len;
    // receive header
    //serial_read( in, 2);
    SerialRL78.readBytes((char *)&in, 2);
    data_len = in[1];
    if (0 == data_len)
    {
        data_len = 256;
    }
    if ((MAX_RESPONSE_LENGTH - 4) < data_len
        || STX != in[0])
    {
        return RESPONSE_FORMAT_ERROR;
    }
    if (explen != data_len)
    {
        return RESPONSE_EXPECTED_LENGTH_ERROR;
    }
    // receive data field, checksum and footer byte
    //serial_read( in + 2, data_len + 2);
    SerialRL78.readBytes((char *)&in+2, data_len + 2);
    switch (in[data_len + 3])
    {
    case ETB:
    case ETX:
        break;
    default:
        return RESPONSE_FORMAT_ERROR;
    }
    if (checksum(in + 1, data_len + 1) != in[data_len + 2])
    {
        return RESPONSE_CHECKSUM_ERROR;
    }
    memcpy(data, in + 2, data_len);
    *len = data_len;
    return RESPONSE_OK;
}

int rl78_cmd_reset(SoftwareSerial fd)
{
    if (3 <= verbose_level)
    {
        printf("Send \"Reset\" command\n");
    }
    rl78_send_cmd( CMD_RESET, NULL, 0);
    int len = 0;
    unsigned char data[3];
    int rc = rl78_recv( &data, &len, 1);
    if (RESPONSE_OK != rc)
    {
        fprintf(stderr, "FAILED\n");
        return rc;
    }
    if (STATUS_ACK != data[0])
    {
        fprintf(stderr, "ACK not received\n");
        return data[0];
    }
    else
    {
        if (3 <= verbose_level)
        {
            printf("\tOK\n");
        }
    }
    return 0;
}

int rl78_cmd_baud_rate_set( int baud, float voltage)
{
    unsigned char buf[2];
    int baud_code;
    switch (baud)
    {
    default:
        fprintf(stderr, "Unsupported baudrate %ubps. Using default baudrate 115200bps.\n", baud);
        baud = 115200;
        // fall through
    case 115200:
        baud_code = RL78_BAUD_115200;
        break;
    case 250000:
        baud_code = RL78_BAUD_250000;
        break;
    case 500000:
        baud_code = RL78_BAUD_500000;
        break;
    case 1000000:
        baud_code = RL78_BAUD_1000000;
        break;
    }
    buf[0] = baud_code;
    buf[1] = (int)(voltage * 10);
    if (3 <= verbose_level)
    {
        printf("Send \"Set Baud Rate\" command (baud=%ubps, voltage=%1.1fV)\n", baud, voltage);
    }
    rl78_send_cmd( CMD_BAUD_RATE_SET, buf, 2);
    int len = 0;
    unsigned char data[3];
    int rc = rl78_recv( &data, &len, 3);
    if (RESPONSE_OK != rc)
    {
        fprintf(stderr, "FAILED baud rate set\n");
        return rc;
    }
    if (STATUS_ACK != data[0])
    {
        fprintf(stderr, "ACK not received\n");
        return data[0];
    }
    if (3 <= verbose_level)
    {
        printf("\tOK\n");
        printf("\tFrequency: %u MHz\n", data[1]);
        printf("\tMode: %s\n", 0 == data[2] ? "full-speed mode" : "wide-voltage mode");
    }
    /* If no need to change baudrate, just exit */
    if (115200 == baud)
    {
        return 0;
    }
    //return serial_set_baud( baud);
    return 0;
}

int rl78_cmd_silicon_signature( char device_name[11], unsigned int *code_size, unsigned int *data_size)
{
    if (3 <= verbose_level)
    {
        printf("Send \"Get Silicon Signature\" command\n");
    }
    rl78_send_cmd( CMD_SILICON_SIGNATURE, NULL, 0);
    int len = 0;
    unsigned char data[22];
    int rc = rl78_recv( &data, &len, 1);
    if (RESPONSE_OK != rc)
    {
        fprintf(stderr, "FAILED\n");
        return rc;
    }
    if (STATUS_ACK != data[0])
    {
        fprintf(stderr, "ACK not received\n");
        return data[0];
    }
    rc = rl78_recv( &data, &len, 22);
    if (RESPONSE_OK != rc)
    {
        fprintf(stderr, "FAILED\n");
        return rc;
    }
    if (NULL != device_name)
    {
        memcpy(device_name, data + 3, 10);
        device_name[10] = '\0';
    }
    unsigned int rom_code_address = 0;
    unsigned int rom_data_address = 0;
    memcpy(&rom_code_address, data + 13, 3);
    memcpy(&rom_data_address, data + 16, 3);
    const unsigned int rom_code_size = (rom_code_address + 1);
    const unsigned int rom_data_size = (rom_data_address != 0) ? (rom_data_address - 0x000F1000U + 1) : 0;

    if (NULL != code_size)
    {
        *code_size = rom_code_size;
    }
    if (NULL != data_size)
    {
        *data_size = rom_data_size;
    }
    if (3 <= verbose_level)
    {
        printf("\tOK\n");
        printf("\tDevice code: %02X%02X%02X\n", data[0], data[1], data[2]);
        printf("\tDevice name: %s\n", device_name);
        printf("\tCode flash size: %ukB\n", rom_code_size / 1024);
        if (rom_data_size != 0)
        {
            printf("\tData flash size: %ukB\n", rom_data_size / 1024);
        }
        else
        {
            printf("\tData flash not present\n");
        }
        printf("\tFirmware version: %X.%X%X\n", data[19], data[20], data[21]);
    }
    return 0;
}

int rl78_cmd_block_erase( unsigned int address)
{
    if (3 <= verbose_level)
    {
        printf("Send \"Block Erase\" command (addres=%06X)\n", address);
    }
    rl78_send_cmd( CMD_BLOCK_ERASE, &address, 3);
    int len = 0;
    unsigned char data[1];
    int rc = rl78_recv( &data, &len, 1);
    if (RESPONSE_OK != rc)
    {
        fprintf(stderr, "FAILED\n");
        return rc;
    }
    if (STATUS_ACK != data[0])
    {
        fprintf(stderr, "ACK not received\n");
        return data[0];
    }
    if (3 <= verbose_level)
    {
        printf("\tOK\n");
    }
    return 0;
}

int rl78_cmd_block_blank_check( unsigned int address_start, unsigned int address_end)
{
    if (3 <= verbose_level)
    {
        printf("Send \"Block Blank Check\" command (range=%06X..%06X)\n", address_start, address_end);
    }
    unsigned char buf[7];
    memcpy(buf + 0, &address_start, 3);
    memcpy(buf + 3, &address_end, 3);
    buf[6] = 0;
    rl78_send_cmd( CMD_BLOCK_BLANK_CHECK, buf, sizeof buf);
    int len = 0;
    unsigned char data[1];
    int rc = rl78_recv( &data, &len, 1);
    if (RESPONSE_OK != rc)
    {
        fprintf(stderr, "FAILED\n");
        return rc;
    }
    if (STATUS_ACK != data[0]
        && STATUS_IVERIFY_BLANK_ERROR != data[0])
    {
        fprintf(stderr, "ACK not received\n");
        return data[0];
    }
    if (STATUS_ACK == data[0])
    {
        rc = 0;
    }
    else
    {
        rc = 1;
    }

    if (3 <= verbose_level)
    {
        printf("\tOK\n");
        if (0 == rc)
        {
            printf("\tBlock is empty\n");
        }
        else
        {
            printf("\tBlock is not empty\n");
        }
    }
    return rc;
}

int rl78_cmd_checksum( unsigned int address_start, unsigned int address_end)
{
    if (3 <= verbose_level)
    {
        printf("Send \"Checksum\" command (range=%06X..%06X)\n", address_start, address_end);
    }
    unsigned char buf[6];
    memcpy(buf + 0, &address_start, 3);
    memcpy(buf + 3, &address_end, 3);
    rl78_send_cmd( CMD_CHECKSUM, buf, sizeof buf);
    int len = 0;
    unsigned char data[2];
    int rc = rl78_recv( &data, &len, 1);
    if (RESPONSE_OK != rc)
    {
        fprintf(stderr, "FAILED\n");
        return rc;
    }
    if (STATUS_ACK != data[0])
    {
        fprintf(stderr, "ACK not received\n");
        return data[0];
    }
    rc = rl78_recv( &data, &len, 2);
    if (3 <= verbose_level)
    {
        printf("\tOK\n");
        printf("\tValue: %02X%02X\n", data[1], data[0]);
    }
    return rc;
}

int rl78_cmd_programming( unsigned int address_start, unsigned int address_end, const void *rom, int proto_ver)
{
    if (3 <= verbose_level)
    {
        printf("Send \"Programming\" command (range=%06X..%06X)\n", address_start, address_end);
    }
    unsigned char buf[6];
    memcpy(buf + 0, &address_start, 3);
    memcpy(buf + 3, &address_end, 3);
    rl78_send_cmd( CMD_PROGRAMMING, buf, sizeof buf);
    int len = 0;
    unsigned char data[2];
    int rc = rl78_recv( &data, &len, 1);
    if (RESPONSE_OK != rc)
    {
        fprintf(stderr, "FAILED (no response)\n");
        return rc;
    }
    if (STATUS_ACK != data[0])
    {
        fprintf(stderr, "ACK not received\n");
        return data[0];
    }
    unsigned int rom_length = address_end - address_start + 1;
    unsigned char *rom_p = (unsigned char*)rom;
    unsigned int address_current = address_start;
    unsigned int final_delay = (rom_length / 1024 + 1) * 1500;
    // Send data
    while (rom_length)
    {
        if (3 <= verbose_level)
        {
            printf("\tSend data to address %06X\n", address_current);
        }
        if (256 < rom_length)
        {
            // Not last data frame
            rl78_send_data( rom_p, 256, 0);
            address_current += 256;
            rom_p += 256;
            rom_length -= 256;
        }
        else
        {
            // Last data frame
            rl78_send_data( rom_p, rom_length, 1);
            address_current += rom_length;
            rom_p += rom_length;
            rom_length -= rom_length;
        }
        rc = rl78_recv( &data, &len, 2);
        if (RESPONSE_OK != rc)
        {
            fprintf(stderr, "FAILED (bad response for block)\n");
            return rc;
        }
        if (STATUS_ACK != data[0])
        {
            fprintf(stderr, "ACK not received\n");
            return data[0];
        }
        if (STATUS_ACK != data[1])
        {
            fprintf(stderr, "Data not written\n");
            return data[1];
        }
    }
    usleep(final_delay);
    // Receive status of completion
    if (proto_ver != PROTOCOL_VERSION_C)
    { /* Protocol A and D require this packet, C doesn't send it */
        rc = rl78_recv( &data, &len, 1);
        if (RESPONSE_OK != rc)
        {
            fprintf(stderr, "FAILED (response not ok)\n");
            return rc;
        }
        if (STATUS_ACK != data[0])
        {
            fprintf(stderr, "ACK not received\n");
            return data[0];
        }
    }
    if (3 <= verbose_level)
    {
        printf("\tOK\n");
    }
    return rc;
}

unsigned int rl78_checksum(const void *rom, unsigned int len)
{
    unsigned int sum = 0;
    unsigned char *p = (unsigned char*)rom;
    unsigned int i = len;
    for (; i; --i)
    {
        sum -= *p++;
    }
    return sum & 0x0000FFFFU;
}

int rl78_cmd_verify( unsigned int address_start, unsigned int address_end, const void *rom)
{
    if (3 <= verbose_level)
    {
        printf("Send \"Verify\" command (range=%06X..%06X)\n", address_start, address_end);
    }
    unsigned char buf[6];
    memcpy(buf + 0, &address_start, 3);
    memcpy(buf + 3, &address_end, 3);
    rl78_send_cmd( CMD_VERIFY, buf, sizeof buf);
    int len = 0;
    unsigned char data[2];
    int rc = rl78_recv( &data, &len, 1);
    if (RESPONSE_OK != rc)
    {
        fprintf(stderr, "FAILED\n");
        return rc;
    }
    if (STATUS_ACK != data[0])
    {
        fprintf(stderr, "ACK not received\n");
        return data[0];
    }
    unsigned int rom_length = address_end - address_start + 1;
    unsigned char *rom_p = (unsigned char*)rom;
    unsigned int address_current = address_start;
    // Send data
    while (rom_length)
    {
        if (3 <= verbose_level)
        {
            printf("\tSend data to address %06X\n", address_current);
        }
        if (256 < rom_length)
        {
            // Not last data frame
            rl78_send_data( rom_p, 256, 0);
            address_current += 256;
            rom_p += 256;
            rom_length -= 256;
        }
        else
        {
            // Last data frame
            rl78_send_data( rom_p, rom_length, 1);
            address_current += rom_length;
            rom_p += rom_length;
            rom_length -= rom_length;
        }
        delay(10);
        rc = rl78_recv( &data, &len, 2);
        if (RESPONSE_OK != rc)
        {
            fprintf(stderr, "FAILED\n");
            return rc;
        }
        if (STATUS_ACK != data[0])
        {
            fprintf(stderr, "ACK not received\n");
            return data[0];
        }
        if (STATUS_ACK != data[1])
        {
            fprintf(stderr, "Verify failed\n");
            return data[1];
        }
    }
    if (3 <= verbose_level)
    {
        printf("\tOK\n");
    }
    return rc;
}

static
int allFFs(const void *mem, unsigned int size)
{
    unsigned char *p = (unsigned char*)mem;
    unsigned int i = size;
    for (; i; --i)
    {
        if (0xFF != *p++)
        {
            return 0;
        }
    }
    return 1;
}

int rl78_program( unsigned int address, const void *data, unsigned int size, unsigned blksz, int proto_ver)
{
    // Make sure size is aligned to flash block boundary
    unsigned int i = size & ~(blksz - 1);
    const unsigned char *mem = (const unsigned char*)data;
    int rc = 0;;
    for (; i; i -= blksz)
    {
        if (!allFFs(mem, blksz))
        {
            if (3 <= verbose_level)
            {
                printf("Program block %06X\n", address);
            }
            // Check if block is ready to program new content
            rc = rl78_cmd_block_blank_check( address, address + blksz - 1);
            if (0 > rc)
            {
                fprintf(stderr, "Block Blank Check failed (%06X)\n", address);
                break;
            }
            if (0 < rc)
            {
                // If block is not empty - erase it
                rc = rl78_cmd_block_erase( address);
                if (0 > rc)
                {
                    fprintf(stderr, "Block Erase failed (%06X)\n", address);
                    break;
                }
            }
            // Write new content
            rc = rl78_cmd_programming( address, address + blksz - 1, mem, proto_ver);
            if (0 > rc)
            {
                fprintf(stderr, "Programming failed (%06X)\n", address);
                break;
            }
            if (2 == verbose_level)
            {
                printf("*");
                fflush(stdout);
            }
        }
        else
        {
            if (3 <= verbose_level)
            {
                printf("No data at block %06X\n", address);
            }
        }
        mem += blksz;
        address += blksz;
    }
    if (2 == verbose_level)
    {
        printf("\n");
    }
    return rc;
}

int rl78_erase( unsigned int start_address, unsigned int size, unsigned blksz)
{
    // Make sure size is aligned to flash block boundary
    unsigned int i = size & ~(blksz - 1);
    unsigned int address = start_address;
    int rc = 0;
    for (; i; i -= blksz)
    {
        rc = rl78_cmd_block_blank_check( address, address + blksz - 1);
        if (0 > rc)
        {
            fprintf(stderr, "Block Blank Check failed (%06X)\n", address);
            break;
        }
        if (0 < rc)
        {
            // If block is not empty
            rc = rl78_cmd_block_erase( address);
            if (0 > rc)
            {
                fprintf(stderr, "Block Erase failed (%06X)\n", address);
                break;
            }
            if (2 == verbose_level)
            {
                printf("*");
                fflush(stdout);
            }
        }
        else
        {
            // if block is already empty
            if (2 == verbose_level)
            {
                printf(".");
                fflush(stdout);
            }
        }
        address += blksz;
    }
    if (2 == verbose_level)
    {
        printf("\n");
    }
    return rc;
}

int rl78_verify( unsigned int address, const void *data, unsigned int size, int blksz)
{
    // Make sure size is aligned to flash block boundary
    unsigned int i = size & ~(blksz - 1);
    const unsigned char *mem = (const unsigned char*)data;
    int rc = 0;
    for (; i; i -= blksz)
    {
        if (3 <= verbose_level)
        {
            printf("Verify block %06X\n", address);
        }
        if (allFFs(mem, blksz))
        {
            // Check if block is blank
            rc = rl78_cmd_block_blank_check( address, address + blksz - 1);
            if (0 > rc)
            {
                fprintf(stderr, "Block Blank Check failed (%06X)\n", address);
                break;
            }
            if (0 < rc)
            {
                fprintf(stderr, "Block content does not match (%06X)\n", address);
                break;
            }
            if (2 == verbose_level)
            {
                printf(".");
                fflush(stdout);
            }
        }
        else
        {
            // If block is not blank
            rc = rl78_cmd_verify( address, address + blksz - 1, mem);
            if (0 != rc)
            {
                fprintf(stderr, "Block content does not match (%06X)\n", address);
                break;
            }
            if (2 == verbose_level)
            {
                printf("*");
                fflush(stdout);
            }
        }
        mem += blksz;
        address += blksz;
    }
    if (2 == verbose_level)
    {
        printf("\n");
    }
    return rc;
}
