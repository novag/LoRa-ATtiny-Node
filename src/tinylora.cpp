/*
 * Original work Copyright (c) 2015, 2016 Ideetron B.V.
 * Modified work Copyright (c) 2018 Hendrik Hagendorn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http:// www.gnu.org/licenses/>.
 */
#ifdef DEBUG
#include "basicserial.h"
#endif

#include <avr/pgmspace.h>
#include <string.h>
#include <util/delay.h>

#include "config.h"
#include "pins.h"
#include "tinylora.h"
#include "tinyspi.h"

extern uint8_t NwkSKey[16];
extern uint8_t AppSKey[16];
extern uint8_t DevAddr[4];
extern TinySPI SPI;

/*
*****************************************************************************************
* Description: S_Table used for AES encription
*****************************************************************************************
*/
const unsigned char PROGMEM TinyLoRa::S_Table[16][16] = {
    {0x63, 0x7C, 0x77, 0x7B, 0xF2, 0x6B, 0x6F, 0xC5, 0x30, 0x01, 0x67, 0x2B, 0xFE, 0xD7, 0xAB, 0x76},
    {0xCA, 0x82, 0xC9, 0x7D, 0xFA, 0x59, 0x47, 0xF0, 0xAD, 0xD4, 0xA2, 0xAF, 0x9C, 0xA4, 0x72, 0xC0},
    {0xB7, 0xFD, 0x93, 0x26, 0x36, 0x3F, 0xF7, 0xCC, 0x34, 0xA5, 0xE5, 0xF1, 0x71, 0xD8, 0x31, 0x15},
    {0x04, 0xC7, 0x23, 0xC3, 0x18, 0x96, 0x05, 0x9A, 0x07, 0x12, 0x80, 0xE2, 0xEB, 0x27, 0xB2, 0x75},
    {0x09, 0x83, 0x2C, 0x1A, 0x1B, 0x6E, 0x5A, 0xA0, 0x52, 0x3B, 0xD6, 0xB3, 0x29, 0xE3, 0x2F, 0x84},
    {0x53, 0xD1, 0x00, 0xED, 0x20, 0xFC, 0xB1, 0x5B, 0x6A, 0xCB, 0xBE, 0x39, 0x4A, 0x4C, 0x58, 0xCF},
    {0xD0, 0xEF, 0xAA, 0xFB, 0x43, 0x4D, 0x33, 0x85, 0x45, 0xF9, 0x02, 0x7F, 0x50, 0x3C, 0x9F, 0xA8},
    {0x51, 0xA3, 0x40, 0x8F, 0x92, 0x9D, 0x38, 0xF5, 0xBC, 0xB6, 0xDA, 0x21, 0x10, 0xFF, 0xF3, 0xD2},
    {0xCD, 0x0C, 0x13, 0xEC, 0x5F, 0x97, 0x44, 0x17, 0xC4, 0xA7, 0x7E, 0x3D, 0x64, 0x5D, 0x19, 0x73},
    {0x60, 0x81, 0x4F, 0xDC, 0x22, 0x2A, 0x90, 0x88, 0x46, 0xEE, 0xB8, 0x14, 0xDE, 0x5E, 0x0B, 0xDB},
    {0xE0, 0x32, 0x3A, 0x0A, 0x49, 0x06, 0x24, 0x5C, 0xC2, 0xD3, 0xAC, 0x62, 0x91, 0x95, 0xE4, 0x79},
    {0xE7, 0xC8, 0x37, 0x6D, 0x8D, 0xD5, 0x4E, 0xA9, 0x6C, 0x56, 0xF4, 0xEA, 0x65, 0x7A, 0xAE, 0x08},
    {0xBA, 0x78, 0x25, 0x2E, 0x1C, 0xA6, 0xB4, 0xC6, 0xE8, 0xDD, 0x74, 0x1F, 0x4B, 0xBD, 0x8B, 0x8A},
    {0x70, 0x3E, 0xB5, 0x66, 0x48, 0x03, 0xF6, 0x0E, 0x61, 0x35, 0x57, 0xB9, 0x86, 0xC1, 0x1D, 0x9E},
    {0xE1, 0xF8, 0x98, 0x11, 0x69, 0xD9, 0x8E, 0x94, 0x9B, 0x1E, 0x87, 0xE9, 0xCE, 0x55, 0x28, 0xDF},
    {0x8C, 0xA1, 0x89, 0x0D, 0xBF, 0xE6, 0x42, 0x68, 0x41, 0x99, 0x2D, 0x0F, 0xB0, 0x54, 0xBB, 0x16}
};

void TinyLoRa::init() {
    // Sleep
    RFM_Write(RFM_REG_OP_MODE, 0x00);

    // LoRa mode
    RFM_Write(RFM_REG_OP_MODE, 0x80);

    // PA pin (maximal power)
    RFM_Write(RFM_REG_PA_CONFIG, 0xFF);

    // Rx timeout: 37 symbols
    RFM_Write(RFM_REG_SYMB_TIMEOUT_LSB, 0x25);

    // Preamble length: 8 symbols
    // 0x0008 + 4 = 12
    RFM_Write(RFM_REG_PREAMBLE_MSB, 0x00);
    RFM_Write(RFM_REG_PREAMBLE_LSB, 0x08);

    // Low datarate optimization off, AGC auto on
    RFM_Write(RFM_REG_MODEM_CONFIG_3, 0x0C);

    // LoRa sync word
    RFM_Write(RFM_REG_SYNC_WORD, 0x34);

    // Set IQ to normal values
    RFM_Write(RFM_REG_INVERT_IQ, 0x27);
    RFM_Write(0x3B, 0x1D);

    // FIFO pointers
    RFM_Write(RFM_REG_FIFO_TX_BASE_ADDR, 0x80);
    RFM_Write(RFM_REG_FIFO_RX_BASE_ADDR, 0x00);

    uint16_t frame_counter = 0;
}

/*
*****************************************************************************************
* Description : Function for sending a package with the RFM
*
* Arguments   : *RFM_Tx_Package Pointer to arry with data to be send
*               Package_Length  Length of the package to send
*****************************************************************************************
*/
void TinyLoRa::RFM_Send_Package(unsigned char *tx_package, unsigned char tx_package_length) {
    // unsigned char TxDone = 0x00;

    // Set RFM in Standby mode wait on mode ready
    RFM_Write(RFM_REG_OP_MODE, 0x81);

    // wait for standby mode
    _delay_ms(10);

    unsigned char res = RFM_Read(RFM_REG_OP_MODE);
#ifdef DEBUG
    ssend("incoming: ");
    TxByte(res);
    ssend(" ok\n");
#endif

    // change the channel of the RFM module
    //switch (random_num) {
    switch (0) {
        case 0x00: // Channel 0 868.100 MHz / 61.035 Hz = 14222987 = 0xD9068B
            RFM_Write(RFM_REG_FR_MSB, 0xD9);
            RFM_Write(RFM_REG_FR_MID, 0x06);
            RFM_Write(RFM_REG_FR_LSB, 0x8B);
            break;
        case 0x01: // Channel 1 868.300 MHz / 61.035 Hz = 14226264 = 0xD91358
            RFM_Write(RFM_REG_FR_MSB, 0xD9);
            RFM_Write(RFM_REG_FR_MID, 0x13);
            RFM_Write(RFM_REG_FR_LSB, 0x58);
            break;
        case 0x02: // Channel 2 868.500 MHz / 61.035 Hz = 14229540 = 0xD92024
            RFM_Write(RFM_REG_FR_MSB, 0xD9);
            RFM_Write(RFM_REG_FR_MID, 0x20);
            RFM_Write(RFM_REG_FR_LSB, 0x24);
            break;
        case 0x03: // Channel 3 867.100 MHz / 61.035 Hz = 14206603 = 0xD8C68B
            RFM_Write(RFM_REG_FR_MSB, 0xD8);
            RFM_Write(RFM_REG_FR_MID, 0xC6);
            RFM_Write(RFM_REG_FR_LSB, 0x8B);
            break;
    }

    // SF7 BW 125 kHz
    //RFM_Write(RFM_REG_MODEM_CONFIG_2, 0x74); // SF7 CRC On

    // SF10 BW 125 kHz
    RFM_Write(RFM_REG_MODEM_CONFIG_2, 0xA4); // SF10 CRC On
    RFM_Write(RFM_REG_MODEM_CONFIG_1, 0x72); // 125 kHz 4/5 coding rate explicit header mode
    RFM_Write(RFM_REG_MODEM_CONFIG_3, 0x04); // Low datarate optimization off AGC auto on
    //

    // Set payload length to the right length
    RFM_Write(RFM_REG_PAYLOAD_LENGTH, tx_package_length);

    // Set SPI pointer to start of Tx part in FiFo
    RFM_Write(RFM_REG_FIFO_ADDR_PTR, 0x80);

    // Write Payload to FiFo
    for (unsigned char i = 0; i < tx_package_length; i++) {
        RFM_Write(RFM_REG_FIFO, *tx_package);
        tx_package++;
    }

    // Switch RFM to Tx
    RFM_Write(RFM_REG_OP_MODE, 0x83);

    // Wait for TxDone in the RegIrqFlags register
    while ((RFM_Read(RFM_REG_IRQ_FLAGS) & 0x08) != 0x08);

#ifdef DEBUG
    ssend("TXDONE\n");
#endif

    // Clear interrupt
    RFM_Write(RFM_REG_IRQ_FLAGS, 0x08);

    // Switch RFM to sleep
    RFM_Write(RFM_REG_OP_MODE, 0x00);
}

/*
*****************************************************************************************
* Description : Funtion that writes a register from the RFM
*
* Arguments   : address Address of register to be written
*
*               data    Data to be written
*****************************************************************************************
*/
void TinyLoRa::RFM_Write(unsigned char address, unsigned char data) {
    // Set NSS pin Low to start communication
    PRT_RFM_NSS &= ~(1 << PB_RFM_NSS);

    // Send addres with MSB 1 to write
    SPI.Transfer(address | 0x80);
    // Send Data
    SPI.Transfer(data);

    // Set NSS pin High to end communication
    PRT_RFM_NSS |= (1 << PB_RFM_NSS);
}

/*
*****************************************************************************************
* Description : Funtion that reads a register from the RFM and returns the value
*
* Arguments   : address Address of register to be read
*
* Returns   : Value of the register
*****************************************************************************************
*/
unsigned char TinyLoRa::RFM_Read(unsigned char address) {
    unsigned char data;

    // Set NSS pin low to start SPI communication
    PRT_RFM_NSS &= ~(1 << PB_RFM_NSS);

    // Send Address
    SPI.Transfer(address);
    // Send 0x00 to be able to receive the answer from the RFM
    data = SPI.Transfer(0x00);

    // Set NSS high to end communication
    PRT_RFM_NSS |= (1 << PB_RFM_NSS);

    // Return received data
    return data;
}

/*
*****************************************************************************************
* Description : Function contstructs a LoRaWAN package and sends it
*
* Arguments   : *payload pointer to the array of data that will be transmitted
*               payload_length nuber of bytes to be transmitted
*               tx_frame_counter  Frame counter of upstream frames
*****************************************************************************************
*/
void TinyLoRa::transmit(unsigned char *payload, unsigned char payload_length, unsigned int tx_frame_counter) {
    // Direction up
    unsigned char direction = 0x00;

    unsigned char rfm_data[64];
    unsigned char rfm_data_length;

    unsigned char mic[4];

    unsigned char mac_header = LORAWAN_MTYPE_UNCONFIRMED_DATA_UP;

    unsigned char frame_control = 0x00;
    unsigned char frame_port = 0x01;

    // Encrypt the data
    encrypt_payload(payload, payload_length, tx_frame_counter, direction);


    // Build the Radio Package
    rfm_data[0] = mac_header;

    rfm_data[1] = DevAddr[3];
    rfm_data[2] = DevAddr[2];
    rfm_data[3] = DevAddr[1];
    rfm_data[4] = DevAddr[0];

    rfm_data[5] = frame_control;

    rfm_data[6] = (tx_frame_counter & 0x00FF);
    rfm_data[7] = ((tx_frame_counter >> 8) & 0x00FF);

    rfm_data[8] = frame_port;

    rfm_data_length = 9;

    // Load Data
    for (unsigned char i = 0; i < payload_length; i++) {
        rfm_data[rfm_data_length + i] = payload[i];
    }
    rfm_data_length += payload_length;

    // Calculate MIC
    calculate_mic(rfm_data, mic, rfm_data_length, tx_frame_counter, direction);
    // Load MIC in package
    for (unsigned char i = 0; i < 4; i++) {
        rfm_data[i + rfm_data_length] = mic[i];
    }
    rfm_data_length += 4;

    // Send Package
    RFM_Send_Package(rfm_data, rfm_data_length);
}

/*
*****************************************************************************************
* Description : Function used to encrypt and decrypt the data in a LoRaWAN data message
*
* Arguments   : *payload pointer to the data to de/encrypt
*               payload_length nuber of bytes to be transmitted
*               frame_counter  Frame counter of upstream frames
*               direction of msg is up
*****************************************************************************************
*/
void TinyLoRa::encrypt_payload(unsigned char *payload, unsigned char payload_length, unsigned int frame_counter, unsigned char direction) {
    unsigned char block_count = 0x00;
    unsigned char incomplete_block_size = 0x00;

    unsigned char block_a[16];

    // Calculate number of blocks
    block_count = payload_length / 16;
    incomplete_block_size = payload_length % 16;
    if (incomplete_block_size != 0) {
        block_count++;
    }

    for (unsigned char i = 1; i <= block_count; i++) {
        block_a[0] = 0x01;
        block_a[1] = 0x00;
        block_a[2] = 0x00;
        block_a[3] = 0x00;
        block_a[4] = 0x00;

        block_a[5] = direction;

        block_a[6] = DevAddr[3];
        block_a[7] = DevAddr[2];
        block_a[8] = DevAddr[1];
        block_a[9] = DevAddr[0];

        block_a[10] = (frame_counter & 0x00FF);
        block_a[11] = ((frame_counter >> 8) & 0x00FF);

        block_a[12] = 0x00; // Frame counter upper Bytes
        block_a[13] = 0x00;

        block_a[14] = 0x00;

        block_a[15] = i;

        // Calculate S
        AES_Encrypt(block_a, AppSKey);


        // Check for last block
        if (i != block_count) {
            for (unsigned char j = 0; j < 16; j++) {
                *payload ^= block_a[j];
                payload++;
            }
        } else {
            if (incomplete_block_size == 0) {
                incomplete_block_size = 16;
            }

            for (unsigned char j = 0; j < incomplete_block_size; j++) {
                *payload ^= block_a[j];
                payload++;
            }
        }
    }
}

/*
*****************************************************************************************
* Description : Function used to calculate the MIC of data
*
* Arguments   : *data pointer to the data to de/encrypt
*               data_length nuber of bytes to be transmitted
*               final_mic Array of 4 bytes
*               frame_counter  Frame counter of upstream frames
*               direction of msg is up
*****************************************************************************************
*/
void TinyLoRa::calculate_mic(unsigned char *data, unsigned char *final_mic, unsigned char data_length, unsigned int frame_counter, unsigned char direction) {
    unsigned char block_b[16];

    unsigned char key1[16] = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    unsigned char key2[16] = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };

    unsigned char old_data[16] = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    unsigned char new_data[16] = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };

    unsigned char block_count = 0x00;
    unsigned char incomplete_block_size = 0x00;
    unsigned char block_counter = 0x01;

    // Create block_b
    block_b[0] = 0x49;
    block_b[1] = 0x00;
    block_b[2] = 0x00;
    block_b[3] = 0x00;
    block_b[4] = 0x00;

    block_b[5] = direction;

    block_b[6] = DevAddr[3];
    block_b[7] = DevAddr[2];
    block_b[8] = DevAddr[1];
    block_b[9] = DevAddr[0];

    block_b[10] = (frame_counter & 0x00FF);
    block_b[11] = ((frame_counter >> 8) & 0x00FF);

    block_b[12] = 0x00; // Frame counter upper bytes
    block_b[13] = 0x00;

    block_b[14] = 0x00;
    block_b[15] = data_length;

    // Calculate number of Blocks and blocksize of last block
    block_count = data_length / 16;
    incomplete_block_size = data_length % 16;

    if (incomplete_block_size != 0) {
        block_count++;
    }

    generate_keys(key1, key2);

    // Preform Calculation on Block B0

    // Preform AES encryption
    AES_Encrypt(block_b, NwkSKey);

    // Copy block_b to old_data
    for (unsigned char i = 0; i < 16; i++) {
        old_data[i] = block_b[i];
    }

    // Preform full calculating until n-1 messsage blocks
    while (block_counter < block_count) {
        // Copy data into array
        for (unsigned char i = 0; i < 16; i++) {
            new_data[i] = *data;
            data++;
        }

        // Preform xor_data with old data
        xor_data(new_data, old_data);

        // Preform AES encryption
        AES_Encrypt(new_data, NwkSKey);

        // Copy new_data to old_data
        for (unsigned char i = 0; i < 16; i++) {
            old_data[i] = new_data[i];
        }

        // Raise Block counter
        block_counter++;
    }

    // Perform calculation on last block
    // Check if Datalength is a multiple of 16
    if (incomplete_block_size == 0) {
        // Copy last data into array
        for (unsigned char i = 0; i < 16; i++) {
            new_data[i] = *data;
            data++;
        }

        // Preform xor_data with Key 1
        xor_data(new_data, key1);

        // Preform xor_data with old data
        xor_data(new_data, old_data);

        // Preform last AES routine
        // read NwkSKey from PROGMEM
        AES_Encrypt(new_data, NwkSKey);
    } else {
        // Copy the remaining data and fill the rest
        for (unsigned char i = 0; i < 16; i++) {
            if (i < incomplete_block_size) {
                new_data[i] = *data;
                data++;
            }

            if (i == incomplete_block_size) {
                new_data[i] = 0x80;
            }

            if (i > incomplete_block_size) {
                new_data[i] = 0x00;
            }
        }

        // Preform xor_data with Key 2
        xor_data(new_data, key2);

        // Preform xor_data with Old data
        xor_data(new_data, old_data);

        // Preform last AES routine
        AES_Encrypt(new_data, NwkSKey);
    }

    final_mic[0] = new_data[0];
    final_mic[1] = new_data[1];
    final_mic[2] = new_data[2];
    final_mic[3] = new_data[3];

    random_num = final_mic[3] & 0x03;
}

/*
*****************************************************************************************
* Description : Function used to generate keys for the MIC calculation
*
* Arguments   : *key1 pointer to Key1
*               *key2 pointer ot Key2
*****************************************************************************************
*/
void TinyLoRa::generate_keys(unsigned char *key1, unsigned char *key2) {
    unsigned char msb_key;

    // Encrypt the zeros in key1 with the NwkSKey
    AES_Encrypt(key1, NwkSKey);

    // Create key1
    // Check if MSB is 1
    if ((key1[0] & 0x80) == 0x80) {
        msb_key = 1;
    } else {
        msb_key = 0;
    }

    // Shift key1 one bit left
    shift_left_data(key1);

    // if MSB was 1
    if (msb_key == 1) {
        key1[15] = key1[15] ^ 0x87;
    }

    // Copy key1 to key2
    for (unsigned char i = 0; i < 16; i++) {
        key2[i] = key1[i];
    }

    // Check if MSB is 1
    if ((key2[0] & 0x80) == 0x80) {
        msb_key = 1;
    } else {
        msb_key = 0;
    }

    // Shift key2 one bit left
    shift_left_data(key2);

    // Check if MSB was 1
    if (msb_key == 1) {
        key2[15] = key2[15] ^ 0x87;
    }
}

void TinyLoRa::shift_left_data(unsigned char *data) {
    unsigned char overflow = 0;

    for (unsigned char i = 0; i < 16; i++) {
        // Check for overflow on next byte except for the last byte
        if (i < 15) {
            // Check if upper bit is one
            if ((data[i + 1] & 0x80) == 0x80) {
                overflow = 1;
            } else {
                overflow = 0;
            }
        } else {
            overflow = 0;
        }

        // Shift one left
        data[i] = (data[i] << 1) + overflow;
    }
}

void TinyLoRa::xor_data(unsigned char *new_data, unsigned char *old_data) {
    for (unsigned char i = 0; i < 16; i++) {
        new_data[i] = new_data[i] ^ old_data[i];
    }
}

/*
*****************************************************************************************
* Title        : AES_Encrypt
* Description  :
*****************************************************************************************
*/
void TinyLoRa::AES_Encrypt(unsigned char *data, unsigned char *key) {
    unsigned char round;
    unsigned char round_key[16];
    unsigned char state[4][4];

    // Copy input to state arry
    for (unsigned char column = 0; column < 4; column++) {
        for (unsigned char row = 0; row < 4; row++) {
            state[row][column] = data[row + (column << 2)];
        }
    }

    // Copy key to round key
    memcpy(&round_key[0], &key[0], 16);

    // Add round key
    AES_Add_Round_Key(round_key, state);

    // Preform 9 full rounds with mixed collums
    for (round = 1; round < 10; round++) {
        // Perform Byte substitution with S table
        for (unsigned char column = 0; column < 4; column++) {
            for (unsigned char row = 0; row < 4; row++) {
                state[row][column] = AES_Sub_Byte(state[row][column]);
            }
        }

        // Perform row Shift
        AES_Shift_Rows(state);

        // Mix Collums
        AES_Mix_Collums(state);

        // Calculate new round key
        AES_Calculate_Round_Key(round, round_key);

        // Add the round key to the round_key
        AES_Add_Round_Key(round_key, state);
    }

    // Perform Byte substitution with S table whitout mix collums
    for (unsigned char column = 0; column < 4; column++) {
        for (unsigned char row = 0; row < 4; row++) {
            state[row][column] = AES_Sub_Byte(state[row][column]);
        }
    }

    // Shift rows
    AES_Shift_Rows(state);

    // Calculate new round key
    AES_Calculate_Round_Key(round, round_key);

    // Add round key
    AES_Add_Round_Key(round_key, state);

    // Copy the state into the data array
    for (unsigned char column = 0; column < 4; column++) {
        for (unsigned char row = 0; row < 4; row++) {
            data[row + (column << 2)] = state[row][column];
        }
    }
}

/*
*****************************************************************************************
* Title       : AES_Add_Round_Key
* Description :
*****************************************************************************************
*/
void TinyLoRa::AES_Add_Round_Key(unsigned char *round_key, unsigned char (*state)[4]) {
    for (unsigned char column = 0; column < 4; column++) {
        for (unsigned char row = 0; row < 4; row++) {
            state[row][column] ^= round_key[row + (column << 2)];
        }
    }
}

/*
*****************************************************************************************
* Title       : AES_Sub_Byte
* Description :
*****************************************************************************************
*/
unsigned char TinyLoRa::AES_Sub_Byte(unsigned char byte) {
    // unsigned char S_Row, S_Collum;
    // unsigned char S_Byte;

    // S_Row    = ((byte >> 4) & 0x0F);
    // S_Collum = ((byte >> 0) & 0x0F);
    // S_Byte   = S_Table[S_Row][S_Collum];

    // return S_Table[((byte >> 4) & 0x0F)][((byte >> 0) & 0x0F)]; // original
    return pgm_read_byte(&(S_Table[((byte >> 4) & 0x0F)][((byte >> 0) & 0x0F)]));
}

/*
*****************************************************************************************
* Title       : AES_Shift_Rows
* Description :
*****************************************************************************************
*/
void TinyLoRa::AES_Shift_Rows(unsigned char (*state)[4]) {
    unsigned char buffer;

    // Store firt byte in buffer
    buffer      = state[1][0];
    // Shift all bytes
    state[1][0] = state[1][1];
    state[1][1] = state[1][2];
    state[1][2] = state[1][3];
    state[1][3] = buffer;

    buffer      = state[2][0];
    state[2][0] = state[2][2];
    state[2][2] = buffer;
    buffer      = state[2][1];
    state[2][1] = state[2][3];
    state[2][3] = buffer;

    buffer      = state[3][3];
    state[3][3] = state[3][2];
    state[3][2] = state[3][1];
    state[3][1] = state[3][0];
    state[3][0] = buffer;
}

/*
*****************************************************************************************
* Title       : AES_Mix_Collums
* Description :
*****************************************************************************************
*/
void TinyLoRa::AES_Mix_Collums(unsigned char (*state)[4]) {
    unsigned char a[4], b[4];

    for (unsigned char column = 0; column < 4; column++) {
        for (unsigned char row = 0; row < 4; row++) {
            a[row] =  state[row][column];
            b[row] = (state[row][column] << 1);

            if ((state[row][column] & 0x80) == 0x80) {
                b[row] ^= 0x1B;
            }
        }

        state[0][column] = b[0] ^ a[1] ^ b[1] ^ a[2] ^ a[3];
        state[1][column] = a[0] ^ b[1] ^ a[2] ^ b[2] ^ a[3];
        state[2][column] = a[0] ^ a[1] ^ b[2] ^ a[3] ^ b[3];
        state[3][column] = a[0] ^ b[0] ^ a[1] ^ a[2] ^ b[3];
    }
}

/*
*****************************************************************************************
* Title       : AES_Calculate_Round_Key
* Description :
*****************************************************************************************
*/
void TinyLoRa::AES_Calculate_Round_Key(unsigned char round, unsigned char *round_key) {
    unsigned char tmp[4];

    // Calculate rcon
    unsigned char rcon = 0x01;
    while (round != 1) {
        unsigned char b = rcon & 0x80;
        rcon = rcon << 1;

        if (b == 0x80) {
            rcon ^= 0x1b;
        }
        round--;
    }

    // Calculate first tmp
    // Copy laste byte from previous key and subsitute the byte, but shift the array contents around by 1.
    tmp[0] = AES_Sub_Byte(round_key[12 + 1]);
    tmp[1] = AES_Sub_Byte(round_key[12 + 2]);
    tmp[2] = AES_Sub_Byte(round_key[12 + 3]);
    tmp[3] = AES_Sub_Byte(round_key[12 + 0]);

    // XOR with rcon
    tmp[0] ^= rcon;

    // Calculate new key
    for (unsigned char i = 0; i < 4; i++) {
        for (unsigned char j = 0; j < 4; j++) {
            round_key[j + (i << 2)] ^= tmp[j];
            tmp[j] = round_key[j + (i << 2)];
        }
    }
}
