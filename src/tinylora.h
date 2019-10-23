#ifndef TINY_LORA_H
#define TINY_LORA_H

#include <stddef.h>
#include <stdint.h>

// RFM
#define RFM_REG_FIFO                    0x00
#define RFM_REG_OP_MODE                 0x01
#define RFM_REG_FR_MSB                  0x06
#define RFM_REG_FR_MID                  0x07
#define RFM_REG_FR_LSB                  0x08
#define RFM_REG_PA_CONFIG               0x09
#define RFM_REG_FIFO_ADDR_PTR           0x0D
#define RFM_REG_FIFO_TX_BASE_ADDR       0x0E
#define RFM_REG_FIFO_RX_BASE_ADDR       0x0F
#define RFM_REG_FIFO_RX_CURRENT_ADDR    0x10
#define RFM_REG_IRQ_FLAGS_MASK          0x11
#define RFM_REG_IRQ_FLAGS               0x12
#define RFM_REG_RX_NB_BYTES             0x13
#define RFM_REG_MODEM_CONFIG_1          0x1D
#define RFM_REG_MODEM_CONFIG_2          0x1E
#define RFM_REG_SYMB_TIMEOUT_LSB        0x1F
#define RFM_REG_PREAMBLE_MSB            0x20
#define RFM_REG_PREAMBLE_LSB            0x21
#define RFM_REG_PAYLOAD_LENGTH          0x22
#define RFM_REG_MODEM_CONFIG_3          0x26
// Only in SX1276 datasheet
#define RFM_REG_INVERT_IQ               0x33
#define RFM_REG_SYNC_WORD               0x39
// UNDOCUMENTED
#define RFM_REG_INVERT_IQ_2             0x3B

// RFM status
#define RFM_STATUS_TX_DONE              0x08
#define RFM_STATUS_RX_DONE              0x40
#define RFM_STATUS_RX_DONE_CRC_ERROR    0x60
#define RFM_STATUS_RX_TIMEOUT           0x80

#define RFM_ERROR_RX_TIMEOUT    -1
#define RFM_ERROR_CRC           -2
#define RFM_ERROR_UNKNOWN       -3

// LoRaWAN
#define LORAWAN_MTYPE_JOIN_REQUEST          0x00
#define LORAWAN_MTYPE_JOIN_ACCEPT           0x20
#define LORAWAN_MTYPE_UNCONFIRMED_DATA_UP   0x40
#define LORAWAN_MTYPE_UNCONFIRMED_DATA_DOWN 0x60
#define LORAWAN_MTYPE_CONFIRMED_DATA_UP     0x80
#define LORAWAN_MTYPE_CONFIRMED_DATA_DOWN   0xA0
#define LORAWAN_MTYPE_RFU                   0xC0
#define LORAWAN_MTYPE_PROPRIETARY           0xE0

// LoRaWAN Join packet sizes
#define LORAWAN_JOIN_REQUEST_SIZE       18
#define LORAWAN_JOIN_ACCEPT_MAX_SIZE    28

// LoRaWAN delays in seconds
#define LORAWAN_RECEIVE_DELAY1      1
#define LORAWAN_RECEIVE_DELAY2      2
#define LORAWAN_JOIN_ACCEPT_DELAY1  5
#define LORAWAN_JOIN_ACCEPT_DELAY2  6

// LoRaWAN Error
#define LORAWAN_ERROR_NO_PACKET_RECEIVED    -1
#define LORAWAN_ERROR_SIZE_EXCEEDED         -2
#define LORAWAN_ERROR_UNEXPECTED_MTYPE      -3
#define LORAWAN_ERROR_INVALID_FRAME_COUNTER -4
#define LORAWAN_ERROR_UNEXPECTED_DEV_ADDR   -5
#define LORAWAN_ERROR_INVALID_MIC           -6
#define LORAWAN_ERROR_INVALID_JOIN_NONCE    -7

// LoRaWAN spreading factors
#define SF7BW125    0
#define SF7BW250    1
#define SF8BW125    2
#define SF9BW125    3
#define SF10BW125   4
#define SF11BW125   5
#define SF12BW125   6

class TinyLoRa {
  public:
    uint16_t mTxFrameCounter = 0;
    void Init(void);
    bool HasJoined(void);
    int8_t Join();
    void Transmit(uint8_t fport, uint8_t *payload, uint8_t payload_length);

  private:
    bool mHasJoined = false;
    uint8_t mRandomNumber;
    static const uint8_t FrequencyTable[9][3];
    static const uint8_t SFTable[7][3];
    static const uint8_t S_Table[16][16];
    void InitTimer1();
    int8_t RfmReceivePacket(uint8_t *packet, size_t packet_max_length, int8_t channel, int8_t sf, uint8_t delay);
    void RfmSendPacket(uint8_t *packet, uint8_t packet_length, bool start_counter);
    void RfmWrite(uint8_t address, uint8_t data);
    uint8_t RfmRead(uint8_t address);
    void ProcessJoinAccept1_0(uint8_t *rfm_data, uint8_t *mic, uint8_t rfm_data_length);
    void ProcessJoinAccept1_1(uint8_t *rfm_data, uint8_t *mic, uint8_t rfm_data_length);
    int8_t ProcessJoinAccept();
    void EncryptPayload(uint8_t *payload, uint8_t payload_length, unsigned int frame_counter, uint8_t direction);
    void CalculateMic(const uint8_t *key, uint8_t *data, uint8_t *initial_block, uint8_t *final_mic, uint8_t data_length);
    void CalculateUplinkMic(uint8_t *data, uint8_t *final_mic, uint8_t data_length, unsigned int frame_counter, uint8_t direction);
    void GenerateKeys(const uint8_t *key, uint8_t *key1, uint8_t *key2);
    void ShiftLeftData(uint8_t *data);
    void XorData(uint8_t *new_data, uint8_t *old_data);
    void AesEncrypt(const uint8_t *key, uint8_t *data);
    void AesAddRoundKey(uint8_t *round_key, uint8_t (*state)[4]);
    uint8_t AesSubByte(uint8_t byte);
    void AesShiftRows(uint8_t (*state)[4]);
    void AesMixCollums(uint8_t (*state)[4]);
    void AesCalculateRoundKey(uint8_t round, uint8_t *round_key);

    // EEPROM
    void GetDevAddr(uint8_t *dev_addr);
    void SetDevAddr(uint8_t *dev_addr);
    uint16_t GetDevNonce();
    void SetDevNonce(uint16_t dev_nonce);
    uint32_t GetJoinNonce();
    void SetJoinNonce(uint8_t *data);
    void GetAppSKey(uint8_t *key);
    void SetAppSKey(uint8_t *key);
    void GetFNwkSIntKey(uint8_t *key);
    void SetFNwkSIntKey(uint8_t *key);
    void GetSNwkSIntKey(uint8_t *key);
    void SetSNwkSIntKey(uint8_t *key);
    void GetNwkSEncKey(uint8_t *key);
    void SetNwkSEncKey(uint8_t *key);
};

#endif
