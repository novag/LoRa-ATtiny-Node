#ifndef TINY_LORA_H
#define TINY_LORA_H

// RFM
#define RFM_REG_FIFO                0x00
#define RFM_REG_OP_MODE             0x01
#define RFM_REG_FR_MSB              0x06
#define RFM_REG_FR_MID              0x07
#define RFM_REG_FR_LSB              0x08
#define RFM_REG_PA_CONFIG           0x09
#define RFM_REG_FIFO_ADDR_PTR       0x0D
#define RFM_REG_FIFO_TX_BASE_ADDR   0x0E
#define RFM_REG_FIFO_RX_BASE_ADDR   0x0F
#define RFM_REG_IRQ_FLAGS           0x12
#define RFM_REG_MODEM_CONFIG_1      0x1D
#define RFM_REG_MODEM_CONFIG_2      0x1E
#define RFM_REG_SYMB_TIMEOUT_LSB    0x1F
#define RFM_REG_PREAMBLE_MSB        0x20
#define RFM_REG_PREAMBLE_LSB        0x21
#define RFM_REG_PAYLOAD_LENGTH      0x22
#define RFM_REG_MODEM_CONFIG_3      0x26
// Only in SX1276 datasheet
#define RFM_REG_INVERT_IQ    0x33
#define RFM_REG_SYNC_WORD    0x39

// LoRaWAN
#define LORAWAN_MTYPE_JOIN_REQUEST          0x00
#define LORAWAN_MTYPE_JOIN_ACCEPT           0x20
#define LORAWAN_MTYPE_UNCONFIRMED_DATA_UP   0x40
#define LORAWAN_MTYPE_UNCONFIRMED_DATA_DOWN 0x60
#define LORAWAN_MTYPE_CONFIRMED_DATA_UP     0x80
#define LORAWAN_MTYPE_CONFIRMED_DATA_DOWN   0xA0
#define LORAWAN_MTYPE_RFU                   0xC0
#define LORAWAN_MTYPE_PROPRIETARY           0xE0

class TinyLoRa {
  public:
    uint16_t frame_counter;
    void init(void);
    void transmit(unsigned char *payload, unsigned char payload_length, unsigned int tx_frame_counter);

  private:
    uint8_t random_num;
    static const unsigned char S_Table[16][16];
    void RFM_Send_Package(unsigned char *tx_package, unsigned char tx_package_length);
    void RFM_Write(unsigned char address, unsigned char data);
    unsigned char RFM_Read(unsigned char address);
    void encrypt_payload(unsigned char *payload, unsigned char payload_length, unsigned int frame_counter, unsigned char direction);
    void calculate_mic(unsigned char *data, unsigned char *final_mic, unsigned char data_length, unsigned int frame_counter, unsigned char direction);
    void generate_keys(unsigned char *key1, unsigned char *key2);
    void shift_left_data(unsigned char *data);
    void xor_data(unsigned char *new_data, unsigned char *old_data);
    void AES_Encrypt(unsigned char *data, unsigned char *key);
    void AES_Add_Round_Key(unsigned char *round_key, unsigned char (*state)[4]);
    unsigned char AES_Sub_Byte(unsigned char byte);
    void AES_Shift_Rows(unsigned char (*state)[4]);
    void AES_Mix_Collums(unsigned char (*state)[4]);
    void AES_Calculate_Round_Key(unsigned char round, unsigned char *round_key);
};

#endif
