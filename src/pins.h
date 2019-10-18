#ifndef PINS_H
#define PINS_H

/*** ATtiny85 ***/
#define DDR_I2C_SCL DDRB
#define PRT_I2C_SCL PORTB
#define PIN_I2C_SCL PINB
#define PB_I2C_SCL  PB4

#define DDR_RFM_NSS DDRB
#define PRT_RFM_NSS PORTB
#define PIN_RFM_NSS PINB
#define PB_RFM_NSS  PB3

#define DDR_DHT22   DDRB
#define PRT_DHT22   PORTB
#define PIN_DHT22   PINB
#define PB_DHT22    PB4

#endif
