/*
 * Copyright 2018 Hendrik Hagendorn
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
#include "debug.h"
#endif

#include <avr/sleep.h>
#include <avr/wdt.h>
#include <stdint.h>
#include <util/delay.h>

#include "config.h"
#include "error.h"
#include "pins.h"
#include "ports.h"
#include "tinyi2cmaster.h"
#include "slimlora.h"
#include "tinyspi.h"
#include "utils.h"

#if ENABLE_DHT22
#include "dht22.h"
#elif ENABLE_SI7021
#include "si7021.h"
#elif ENABLE_DS18B20
#include "ds18b20.h"
#else
#warning "No sensor enabled. Using fake temp."
#endif

volatile uint8_t wakeup_count = SLEEP_TOTAL;

ISR(WDT_vect) {
    wakeup_count++;
}

void setup_watchdog() {
    /*
     * WDP3 - WDP2 - WPD1 - WDP0 - time
     * 0      0      0      0      16 ms
     * 0      0      0      1      32 ms
     * 0      0      1      0      64 ms
     * 0      0      1      1      0.125 s
     * 0      1      0      0      0.25 s
     * 0      1      0      1      0.5 s
     * 0      1      1      0      1.0 s
     * 0      1      1      1      2.0 s
     * 1      0      0      0      4.0 s
     * 1      0      0      1      8.0 s
     */

    // Clear watchdog reset flag
    CLEARBIT(MCUSR, WDRF);

    cli();

    // Set watchdog change enable and watchdog enable bit
    SETBITS(WDTCR, BIT(WDCE) | BIT(WDE));

    // Set watchdog interrupt enable and watchdog timeout value to 8 seconds
    WDTCR = BIT(WDIE) | BIT(WDP3) | BIT(WDP0);

    sei();
}

void sleep() {
    // Disabling ADC saves ~230uA
    CLEARBIT(ADCSRA, ADEN);

    setup_watchdog();

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);

    wdt_reset();
    sleep_mode();

    wdt_disable();

    // Enable ADC
    SETBIT(ADCSRA, ADEN);
}

/*
 * Read voltage of the rail (Vcc)
 * output mV (2 bytes)
 */
uint16_t read_voltage() {
    uint8_t low, high;
    uint16_t result;

    // read 1.1V reference against AVcc
    // set the reference to Vcc and the measurement to the internal 1.1V reference
    // default ADMUX REFS1 and REFS0 = 0

    // Vref=Vcc : Vbg=1.1V
    ADMUX = BIT(MUX3) | BIT(MUX2);

    // wait for Vref to settle (1ms should be enough but ...)
    _delay_ms(2);

    // start conversion
    SETBIT(ADCSRA, ADSC);
    // wait
    while (BITVAL(ADCSRA, ADSC) == 1);

    // reading ADCL locks ADCH
    low  = ADCL;
    // reading ADCH unlocks both
    high = ADCH;

    result = (high << 8) | low;

    result = 1125300L / result; // calculate Vcc (in mV); 1125300 = 1.1*1023*1000

    return result;

}

int main() {
    /*
     * 0[7-5]: Status
     * 0[4-0]: Voltage H
     * 1[7-0]: Voltage L
     * 2[7-0]: Temperature H
     * 3[7-0]: Temperature L
     * 4[7-0]: Humidity H
     * 5[7-0]: Humidity L
     */
    uint8_t payload[6], payload_length;
    uint8_t fport, status;
    uint16_t voltage, temperature, humidity;
    SlimLoRa lora;
#if ENABLE_DHT22
    DHT22 dht22;
#elif ENABLE_SI7021
    Si7021 si7021(0x40);
#elif ENABLE_DS18B20
    DS18B20 ds18b20;
#endif

#ifdef DEBUG
    debug(DSTR_BOOTED);
#endif

    _delay_ms(1000);

#if ENABLE_I2C_MASTER
    SETBIT(DDR_I2C_SCL, PB_I2C_SCL);
    CLEARBIT(PRT_I2C_SCL, PB_I2C_SCL);
#endif

    SETBIT(DDR_RFM_NSS, PB_RFM_NSS);
    SETBIT(PRT_RFM_NSS, PB_RFM_NSS);

    SPI.SetDataMode(SPI_MODE0);
    SPI.Init();

    lora.Init();

#if OTAA
    while (!lora.HasJoined()) {
#ifdef DEBUG
        debug(DSTR_JOINING);
#endif
        lora.Join();
        _delay_ms(1000);
    }
#endif // OTAA

#ifdef DEBUG
    debug(DSTR_JOINED);
#endif

#if ENABLE_I2C_MASTER
    SPI.End();
#endif

    for (;;) {
        if (wakeup_count >= SLEEP_TOTAL) {
#ifdef DEBUG
            debug(DSTR_SENDING);
#endif
            payload_length = sizeof(payload);

            CLEARBIT(status, SENSOR_ERROR);
            CLEARBIT(status, CHECKSUM_ERROR);

#if ENABLE_DHT22
            switch (dht22.ReadData()) {
                case -1:
                    SETBIT(status, CHECKSUM_ERROR);
                    // fallthrough
                case 0:
                    humidity = dht22.GetHumidity();
                    temperature = dht22.GetTemperature();
                    break;
                default:
                    SETBIT(status, SENSOR_ERROR);
                    humidity = temperature = 0xFFFF;
                    break;
            }

            fport = FPORT_DHT22;
#elif ENABLE_SI7021
            TinyI2C.Init();
            si7021.Init();

            humidity = si7021.MeasureHumidity();
            switch (humidity) {
                case SI7021_CHECKSUM_ERROR:
                    SETBIT(status, CHECKSUM_ERROR);
                    break;
                case SI7021_TIMEOUT_ERROR:
                    SETBIT(status, SENSOR_ERROR);
                    humidity = 0xFFFF;
                    break;
            }

            temperature = si7021.ReadTemperature();
            switch (temperature) {
                case SI7021_CHECKSUM_ERROR:
                    SETBIT(status, CHECKSUM_ERROR);
                    break;
                case SI7021_TIMEOUT_ERROR:
                    SETBIT(status, SENSOR_ERROR);
                    temperature = 0xFFFF;
                    break;
            }

            TinyI2C.End();

            fport = FPORT_SI7021;
#elif ENABLE_DS18B20
            temperature = ds18b20.MeasureTemperature();
            if (temperature == DS18B20_CHECKSUM_ERROR) {
                SETBIT(status, CHECKSUM_ERROR);
            }

            humidity = 0xFFFF;
            payload_length -= 2;

            fport = FPORT_DS18B20;
#else
            humidity = temperature = 0xFFFF;

            fport = FPORT_GENERAL;
#endif

            voltage = read_voltage();
            payload[0] = status;
            payload[0] |= (voltage >> 8) & 0xFF;
            payload[1] = (voltage & 0xFF);
            payload[2] = (temperature >> 8) & 0xFF;
            payload[3] = temperature & 0xFF;
            payload[4] = (humidity >> 8) & 0xFF;
            payload[5] = humidity & 0xFF;

#if ENABLE_I2C_MASTER
            SPI.SetDataMode(SPI_MODE0);
            SPI.Init();
#endif
            lora.Transmit(fport, payload, payload_length);
#if ENABLE_I2C_MASTER
            SPI.End();
#endif

            wakeup_count = 0;
        }

        sleep();
    }
}
