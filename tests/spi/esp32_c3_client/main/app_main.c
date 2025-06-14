/* SPI Slave example, receiver (uses SPI Slave driver to communicate with sender)

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"

/*
SPI receiver (slave) example.

This example is supposed to work together with the SPI sender. It uses the standard SPI pins (MISO, MOSI, SCLK, CS) to
transmit data over in a full-duplex fashion, that is, while the master puts data on the MOSI pin, the slave puts its own
data on the MISO pin.

This example uses one extra pin: GPIO_HANDSHAKE is used as a handshake pin. After a transmission has been set up and we're
ready to send/receive data, this code uses a callback to set the handshake pin high. The sender will detect this and start
sending a transaction. As soon as the transaction is done, the line gets set low again.
*/

//////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////// Please update the following configuration according to your HardWare spec /////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
#define RCV_HOST    SPI2_HOST

#define GPIO_HANDSHAKE      3 // high  indicates master that slave is ready
#define GPIO_MOSI           7//3 //12
#define GPIO_MISO           2//4 //13
#define GPIO_SCLK           6//5 //15
#define GPIO_CS             10//6 

static const char *TAG = "ESP-C3";


typedef enum {
    SPI_DUMMY_BYTE = 0xDD,
    SPI_CMD_ACK = 0xAA,
    SPI_LEN_ACK = 0xAB,
    SPI_MSG_ACK = 0xAC
} spi_test_005_command_t;

//Called after a transaction is queued and ready for pickup by master. We use this to set the handshake line high.
void my_post_setup_cb(spi_slave_transaction_t *trans)
{
    gpio_set_level(GPIO_HANDSHAKE, 1);
}

//Called after transaction is sent/received. We use this to set the handshake line low.
void my_post_trans_cb(spi_slave_transaction_t *trans)
{
    gpio_set_level(GPIO_HANDSHAKE, 0);
}

//Main application
void app_main(void)
{
    int n = 0;
    esp_err_t ret;

    //Configuration for the SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    //Configuration for the SPI slave interface
    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = GPIO_CS,
        .queue_size = 3,
        .flags = 0,
        .post_setup_cb = my_post_setup_cb,
        .post_trans_cb = my_post_trans_cb
    };

    //Configuration for the handshake line
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = BIT64(GPIO_HANDSHAKE),
    };

    //Configure handshake line as output
    gpio_config(&io_conf);
    //Enable pull-ups on SPI lines so we don't detect rogue pulses when no master is connected.
    gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ONLY);

    //Initialize SPI slave interface
    ret = spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    assert(ret == ESP_OK);

    char *sendbuf = spi_bus_dma_memory_alloc(RCV_HOST, 129, 0);
    char *recvbuf = spi_bus_dma_memory_alloc(RCV_HOST, 129, 0);
    char msg_to_print[129];
    assert(sendbuf && recvbuf);
    spi_slave_transaction_t t = {0};
    ESP_LOGI(TAG, "BOOTED");
    uint8_t msg_len = 0;
    while (1) {
        //Clear receive buffer, set send buffer to something sane
        memset(recvbuf, 0xA5, 129);
        sprintf(sendbuf, "This is the receiver, sending data for transmission number %04d.", n);
        ESP_LOGI(TAG, "Slave receiving 1 byte");
        sendbuf[0] = SPI_DUMMY_BYTE;
        t.length = 1 * 8;//128 * 8;
        t.tx_buffer = sendbuf;
        t.rx_buffer = recvbuf;
        ret = spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY); /* 1000*/
        ESP_LOGI(TAG, "Slave receive done %x, transaction len = %lu", recvbuf[0], t.trans_len);
        if (recvbuf[0] != 0xFF)
            continue;
        ESP_LOGI(TAG, "Slave transmitting 1 byte");
        sendbuf[0] = SPI_CMD_ACK;
        t.length = 1 * 8;//128 * 8;
        t.tx_buffer = sendbuf;
        t.rx_buffer = recvbuf;
        ret = spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY); /* 1000*/
        ESP_LOGI(TAG, "Slave transmit done %x, transaction len = %lu", sendbuf[0], t.trans_len);

        ESP_LOGI(TAG, "Slave receiving 1 byte length");
        sendbuf[0] = SPI_DUMMY_BYTE;
        t.length = 1 * 8;//128 * 8;
        t.tx_buffer = sendbuf;
        t.rx_buffer = recvbuf;
        ret = spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY); /* 1000*/
        ESP_LOGI(TAG, "Slave receive done %x, transaction len = %lu", recvbuf[0], t.trans_len);
        msg_len = recvbuf[0];

        ESP_LOGI(TAG, "Slave transmitting 1 byte (len ack)");
        sendbuf[0] = SPI_LEN_ACK;
        t.length = 1 * 8;//128 * 8;
        t.tx_buffer = sendbuf;
        t.rx_buffer = recvbuf;
        ret = spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY); /* 1000*/
        ESP_LOGI(TAG, "Slave transmit done %x, transaction len = %lu", sendbuf[0], t.trans_len);

        ESP_LOGI(TAG, "Slave receiving msg");
        t.length = msg_len * 8;//128 * 8;
        sendbuf[0] = SPI_DUMMY_BYTE;
        t.tx_buffer = sendbuf;
        t.rx_buffer = recvbuf;
        ret = spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY); /* 1000*/
        memcpy(msg_to_print, recvbuf, msg_len); /* end character is not transmitted, add it here */
        msg_to_print[msg_len] = '\0';
        ESP_LOGI(TAG, "Slave receive done %s, transaction len = %lu", msg_to_print, t.trans_len);

        ESP_LOGI(TAG, "Slave transmitting 1 byte (msg ack)");
        sendbuf[0] = SPI_MSG_ACK;
        t.length = 1 * 8;//128 * 8;
        t.tx_buffer = sendbuf;
        t.rx_buffer = recvbuf;
        ret = spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY); /* 1000*/
        ESP_LOGI(TAG, "Slave transmit done %x, transaction len = %lu", sendbuf[0], t.trans_len);

        //Set up a transaction of 128 bytes to send/receive
        //t.length = 128 * 8;//128 * 8;
        //t.tx_buffer = sendbuf;
        //t.rx_buffer = recvbuf;
        /* This call enables the SPI slave interface to send/receive to the sendbuf and recvbuf. The transaction is
        initialized by the SPI master, however, so it will not actually happen until the master starts a hardware transaction
        by pulling CS low and pulsing the clock etc. In this specific example, we use the handshake line, pulled up by the
        .post_setup_cb callback that is called as soon as a transaction is ready, to let the master know it is free to transfer
        data.
        */
        //ESP_LOGI(TAG, "Slave transmitting");
        
        //ESP_LOGI(TAG, "Slave transmitting done");
        //spi_slave_transmit does not return until the master has done a transmission, so by here we have sent our data and
        //received data from the master. Print it.
        //if (ret == ESP_OK)
        //    printf("Received: %s\n", recvbuf);
        //else
        //    ESP_LOGI(TAG, "Reached timeout waiting master");

        //pause the slave to save power, transaction will also be paused
        //ret = spi_slave_disable(RCV_HOST);
        //if (ret == ESP_OK) {
        //    printf("slave paused ...\n");
        //}
        //vTaskDelay(100);    //now is able to sleep or do something to save power, any following transaction will be ignored
        //ret = spi_slave_enable(RCV_HOST);
        //if (ret == ESP_OK) {
        //    printf("slave ready !\n");
        //}
        n++;
        // Ping master:
        //gpio_set_level(GPIO_HANDSHAKE, 1);
        //gpio_set_level(GPIO_HANDSHAKE, 0);
    }
}
