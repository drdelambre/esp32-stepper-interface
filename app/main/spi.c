#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stdatomic.h"

#include "spi.h"

static const char *TAG = "SPI";

#define NO_CS 3
typedef typeof(SPI2.clock) spi_clock_reg_t;
typedef struct spi_device_t spi_device_t;

// struct to hold private transaction data (like tx and rx buffer for DMA).
typedef struct {
    spi_transaction_t   *trans;
    const uint32_t *buffer_to_send;   //equals to tx_data, if SPI_TRANS_USE_RXDATA is applied; otherwise if original buffer wasn't in DMA-capable memory, this gets the address of a temporary buffer that is;
    //otherwise sets to the original buffer or NULL if no buffer is assigned.
    uint32_t *buffer_to_rcv;    // similar to buffer_to_send
} spi_trans_priv_t;

typedef struct {
    _Atomic(spi_device_t*) device[NO_CS];
    intr_handle_t intr;
    spi_dev_t *hw;
    spi_trans_priv_t cur_trans_buf;
    int cur_cs;     //current device doing transaction
    int prev_cs;    //last device doing transaction, used to avoid re-configure registers if the device not changed
    atomic_int acquire_cs; //the device acquiring the bus, NO_CS if no one is doing so.
    bool polling;   //in process of a polling, avoid of queue new transactions into ISR
    bool isr_free;  //the isr is not sending transactions
    bool bus_locked;//the bus is controlled by a device
    lldesc_t *dmadesc_tx;
    lldesc_t *dmadesc_rx;
    uint32_t flags;
    int dma_chan;
    int max_transfer_sz;
    spi_bus_config_t bus_cfg;
#ifdef CONFIG_PM_ENABLE
    esp_pm_lock_handle_t pm_lock;
#endif
} spi_host_t;

typedef struct {
    spi_clock_reg_t reg;
    int eff_clk;
    int dummy_num;
    int miso_delay;
} clock_config_t;

struct spi_device_t {
    int id;
    QueueHandle_t trans_queue;
    QueueHandle_t ret_queue;
    spi_device_interface_config_t cfg;
    clock_config_t clk_cfg;
    spi_host_t *host;
    SemaphoreHandle_t semphr_polling;   //semaphore to notify the device it claimed the bus
    bool        waiting;                //the device is waiting for the exclusive control of the bus
};

void spi_read(uint8_t *data, spi_device_handle_t *spi) {
    esp_err_t res = ESP_OK;
    spi_transaction_t t;
    uint8_t buff[4] = { 0 };
    uint8_t rxbuff[4] = { 0 };

    memset(&t, 0, sizeof(t));
    t.length = 32;
    t.rxlength = 32;
    t.rx_buffer = rxbuff;
    t.tx_buffer = buff;
    t.user = (void*)1;

    gpio_set_level((*spi)->cfg.spics_io_num, 0);
    res = spi_device_polling_transmit(*spi, &t);
    gpio_set_level((*spi)->cfg.spics_io_num, 1);

    memcpy(data, rxbuff, 4);

    ESP_ERROR_CHECK(res);
}

void spi_read_addressed(uint8_t addr, uint8_t *data, spi_device_handle_t *spi) {
    esp_err_t res = ESP_OK;
    spi_transaction_t t;
    uint8_t buff[4] = { 0 };
    uint8_t rxbuff[4] = { 0 };

    memset(&t, 0, sizeof(t));
    t.length = 32;
    t.addr = addr;
    t.rxlength = 32;
    t.rx_buffer = rxbuff;
    t.tx_buffer = buff;
    t.user = (void*)1;

    gpio_set_level((*spi)->cfg.spics_io_num, 0);
    res = spi_device_polling_transmit(*spi, &t);
    gpio_set_level((*spi)->cfg.spics_io_num, 1);

    memcpy(data, rxbuff, 4);

    ESP_ERROR_CHECK(res);
}

void spi_send_addressed(uint8_t addr, uint8_t *data, spi_device_handle_t *spi) {
    esp_err_t res = ESP_OK;
    spi_transaction_t t;

    uint8_t buff[4] = { 0 };

    memcpy(buff, data, 4);

    memset(&t, 0, sizeof(t));
    t.length = 32;
    t.tx_buffer=buff;
    t.addr = addr;
    t.user=(void*)0;

    gpio_set_level((*spi)->cfg.spics_io_num, 0);
    res = spi_device_polling_transmit(*spi, &t);
    gpio_set_level((*spi)->cfg.spics_io_num, 1);

    ESP_ERROR_CHECK(res);
}

void spi_init_bus(void) {
    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=0,
        .flags=SPICOMMON_BUSFLAG_MASTER
    };

    ESP_ERROR_CHECK(spi_bus_initialize(VSPI_HOST, &buscfg, 1));
}
