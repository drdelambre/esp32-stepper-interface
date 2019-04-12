#include "spi.h"
#include "max31855.h"

// returns the temperature in celsius
double max31855_read(spi_device_handle_t* spi) {
    int32_t buff;
    uint8_t _buff[4];

    spi_read(_buff, spi);

    buff = _buff[0];
    buff <<= 8;
    buff |= _buff[1];
    buff <<= 8;
    buff |= _buff[2];
    buff <<= 8;
    buff |= _buff[3];

    // something is wrong with the sensor
    if (buff & 0x7) {
        return (__builtin_nanf(""));
    }

    if (buff & 0x80000000) {
        buff = 0xFFFFC000 | ((buff >> 18) & 0x00003FFFF);
    } else {
        buff >>= 18;
    }

    return (double)buff * 0.25;
}
