#ifndef SLIM_LORA_H
#define SLIM_LORA_H

#include <stddef.h>
#include <stdint.h>

#include "timing.h"

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
#define RFM_REG_IF_FREQ_2               0x2F
#define RFM_REG_IF_FREQ_1               0x30
#define RFM_REG_DETECT_OPTIMIZE         0x31
#define RFM_REG_INVERT_IQ               0x33
#define RFM_REG_SYNC_WORD               0x39
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

#define LORAWAN_FCTRL_ADR                   0x80
#define LORAWAN_FCTRL_ADR_ACK_REQ           0x40
#define LORAWAN_FCTRL_ACK                   0x20

#define LORAWAN_DIRECTION_UP                0
#define LORAWAN_DIRECTION_DOWN              1

#define LORAWAN_UPLINK_CHANNEL_COUNT        8

// LoRaWAN frame options
#define LORAWAN_FOPT_LINK_CHECK_REQ         0x02
#define LORAWAN_FOPT_LINK_CHECK_ANS         0x02
#define LORAWAN_FOPT_LINK_ADR_REQ           0x03
#define LORAWAN_FOPT_LINK_ADR_ANS           0x03
#define LORAWAN_FOPT_DUTY_CYCLE_REQ         0x04
#define LORAWAN_FOPT_DUTY_CYCLE_ANS         0x04
#define LORAWAN_FOPT_RX_PARAM_SETUP_REQ     0x05
#define LORAWAN_FOPT_RX_PARAM_SETUP_ANS     0x05
#define LORAWAN_FOPT_DEV_STATUS_REQ         0x06
#define LORAWAN_FOPT_DEV_STATUS_ANS         0x06
#define LORAWAN_FOPT_NEW_CHANNEL_REQ        0x07
#define LORAWAN_FOPT_NEW_CHANNEL_ANS        0x07
#define LORAWAN_FOPT_RX_TIMING_SETUP_REQ    0x08
#define LORAWAN_FOPT_RX_TIMING_SETUP_ANS    0x08
#define LORAWAN_FOPT_TX_PARAM_SETUP_REQ     0x09
#define LORAWAN_FOPT_TX_PARAM_SETUP_ANS     0x09
#define LORAWAN_FOPT_DL_CHANNEL_REQ         0x0A
#define LORAWAN_FOPT_DL_CHANNEL_ANS         0x0A
#define LORAWAN_FOPT_DEVICE_TIME_REQ        0x0D
#define LORAWAN_FOPT_DEVICE_TIME_ANS        0x0D
// Proprietary
#define LORAWAN_FOPT_PROP_DISABLE_ADR       0x90
#define LORAWAN_FOPT_PROP_ENABLE_ADR        0x91

// LoRaWAN Join packet sizes
#define LORAWAN_JOIN_REQUEST_SIZE       18
#define LORAWAN_JOIN_ACCEPT_MAX_SIZE    28

// LoRaWAN delays in seconds
#define LORAWAN_RECEIVE_DELAY1_TICKS        1 * TICKS_PER_SECOND
#define LORAWAN_RECEIVE_DELAY2_TICKS        2 * TICKS_PER_SECOND
#define LORAWAN_JOIN_ACCEPT_DELAY1_TICKS    5 * TICKS_PER_SECOND
#define LORAWAN_JOIN_ACCEPT_DELAY2_TICKS    6 * TICKS_PER_SECOND

#define LORAWAN_RX_ERROR_TICKS              10 * TICKS_PER_SECOND / 1000        // 10 ms
#define LORAWAN_RX_MARGIN_TICKS             2000 * TICKS_PER_SECOND / 1000000   // 2000 us
#define LORAWAN_RX_SETUP_TICKS              2000 * TICKS_PER_SECOND / 1000000   // 2000 us
#define LORAWAN_RX_MIN_SYMBOLS              6

// LoRaWAN ADR
#define LORAWAN_ADR_ACK_LIMIT   64
#define LORAWAN_ADR_ACK_DELAY   32

// LoRaWAN Error
#define LORAWAN_ERROR_NO_PACKET_RECEIVED    -1
#define LORAWAN_ERROR_SIZE_EXCEEDED         -2
#define LORAWAN_ERROR_UNEXPECTED_MTYPE      -3
#define LORAWAN_ERROR_INVALID_FRAME_COUNTER -4
#define LORAWAN_ERROR_UNEXPECTED_DEV_ADDR   -5
#define LORAWAN_ERROR_INVALID_MIC           -6
#define LORAWAN_ERROR_INVALID_JOIN_NONCE    -7

// LoRaWAN spreading factors
#define SF7BW250    6
#define SF7BW125    5
#define SF8BW125    4
#define SF9BW125    3
#define SF10BW125   2
#define SF11BW125   1
#define SF12BW125   0


typedef struct {
    uint8_t length;
    uint8_t fopts[15];
} fopts_t;

class SlimLoRa {
  public:
    void Init(void);
    bool HasJoined(void);
    int8_t Join();
    void SendData(uint8_t fport, uint8_t *payload, uint8_t payload_length);
    void SetAdrEnabled(bool enabled);

  private:
    uint8_t mChannel = 0;
    uint8_t mDataRate = SF10BW125;
    uint8_t mRx2DataRate;
    bool mHasJoined = false;
    bool mAdrEnabled = false;
    uint16_t mTxFrameCounter = 0;
    uint16_t mRxFrameCounter = 0;
    uint8_t mAdrAckCounter = 0;
    uint8_t mPseudoByte;
    fopts_t mPendingFopts = {0};
    uint8_t mRxSymbols = LORAWAN_RX_MIN_SYMBOLS;
    uint32_t mTxDoneTickstamp;
    static const uint8_t FrequencyTable[9][3];
    static const uint8_t DataRateTable[7][3];
    static const uint16_t DRTicksPerHalfSymbol[7];
    static const uint8_t S_Table[16][16];
    int8_t RfmReceivePacket(uint8_t *packet, uint8_t packet_max_length, uint8_t channel, uint8_t dri, uint32_t rx_tickstamp);
    void RfmSendPacket(uint8_t *packet, uint8_t packet_length, uint8_t channel, uint8_t dri, bool start_timer);
    inline void RfmWrite(uint8_t address, uint8_t data);
    inline uint8_t RfmRead(uint8_t address);
    inline uint32_t CaluclateDriftAdjustment(uint32_t delay, uint16_t ticks_per_half_symbol);
    inline int32_t CalculateRxWindowOffset(int16_t ticks_per_half_symbol);
    uint32_t CalculateRxDelay(uint8_t data_rate, uint32_t delay);
    inline bool CheckMic(uint8_t *cmic, uint8_t *rmic);
    bool ProcessJoinAccept1_0(uint8_t *rfm_data, uint8_t rfm_data_length);
    bool ProcessJoinAccept1_1(uint8_t *rfm_data, uint8_t rfm_data_length);
    int8_t ProcessJoinAccept(uint8_t window);
    void ProcessFrameOptions(uint8_t *options, uint8_t f_options_length);
    int8_t ProcessDownlink(uint8_t window);
    void Transmit(uint8_t fport, uint8_t *payload, uint8_t payload_length);
    void EncryptPayload(uint8_t *payload, uint8_t payload_length, unsigned int frame_counter, uint8_t direction);
    void CalculateMic(const uint8_t *key, uint8_t *data, uint8_t *initial_block, uint8_t *final_mic, uint8_t data_length);
    void CalculateMessageMic(uint8_t *data, uint8_t *final_mic, uint8_t data_length, unsigned int frame_counter, uint8_t direction);
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
    inline uint16_t GetTxFrameCounter();
    inline void SetTxFrameCounter(uint16_t count);
    inline uint16_t GetRxFrameCounter();
    inline void SetRxFrameCounter(uint16_t count);
    inline uint8_t GetRx2DataRate();
    inline void SetRx2DataRate(uint8_t value);

    inline void GetDevAddr(uint8_t *dev_addr);
    inline void SetDevAddr(uint8_t *dev_addr);
    inline uint16_t GetDevNonce();
    inline void SetDevNonce(uint16_t dev_nonce);
    inline uint32_t GetJoinNonce();
    inline void SetJoinNonce(uint32_t join_nonce);
    inline void GetAppSKey(uint8_t *key);
    inline void SetAppSKey(uint8_t *key);
    inline void GetFNwkSIntKey(uint8_t *key);
    inline void SetFNwkSIntKey(uint8_t *key);
    inline void GetSNwkSIntKey(uint8_t *key);
    inline void SetSNwkSIntKey(uint8_t *key);
    inline void GetNwkSEncKey(uint8_t *key);
    inline void SetNwkSEncKey(uint8_t *key);
};

#endif
