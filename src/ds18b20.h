/*
 * Copyright (c) 2018 Hendrik Hagendorn
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
#ifndef DS18B20_H
#define DS18B20_H

#include <stdint.h>

#define CMD_DS18B20_CONVERT_T           0x44
#define CMD_DS18B20_COPY_SCRATCHPAD     0x48
#define CMD_DS18B20_WRITE_SCRATCHPAD    0x4E
#define CMD_DS18B20_READ_SCRATCHPAD     0xBE
#define CMD_DS18B20_SKIP_ROM            0xCC

#define RES_DS18B20_9_BIT               0x1F
#define RES_DS18B20_10_BIT              0x3F
#define RES_DS18B20_11_BIT              0x5F
#define RES_DS18B20_12_BIT              0x7F

#define DS18B20_CHECKSUM_ERROR          0x8000

class DS18B20 {
  public:
    uint16_t MeasureTemperature();
  private:
    bool mConfigurationValid = false;
    bool Reset();
    void WriteBit(bool bit);
    void WriteByte(uint8_t data);
    bool ReadBit();
    uint8_t ReadByte();
    void SkipROM();
    void CopyScratchpad();
    void WriteScratchpad();
    void ReadScratchpad(uint8_t *data);
    uint8_t Crc8(const uint8_t *data, uint8_t length);
};

#endif
