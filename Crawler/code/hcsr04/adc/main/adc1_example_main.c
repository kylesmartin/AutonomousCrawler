/* servo motor control example
 
 This example code is in the Public Domain (or CC0 licensed, at your option.)
 
 Unless required by applicable law or agreed to in writing, this
 software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 CONDITIONS OF ANY KIND, either express or implied.
 */

//stdlib headers
#include <stdio.h>
#include <math.h>
#include <string.h>

//freeRTOS headers
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

//drivers
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "driver/i2c.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/pcnt.h"

//system headers
#include "esp_types.h"
#include "esp_attr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"

//motor control
#include "soc/mcpwm_periph.h"

//networking
#include "nvs_flash.h"
#include "tcpip_adapter.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

//custom headers
//#include "./ADXL343.h"
//#include "displaychars.h"

//ir comm
#include "driver/rmt.h"

//RMT
#include "freertos/semphr.h"

static const char* NEC_TAG = "NEC";

//CHOOSE SELF TEST OR NORMAL TEST
#define RMT_RX_SELF_TEST   1

#if RMT_RX_SELF_TEST
#define RMT_RX_ACTIVE_LEVEL  1   /*!< Data bit is active high for self test mode */
#define RMT_TX_CARRIER_EN    0   /*!< Disable carrier for self test mode  */
#else
//Test with infrared LED, we have to enable carrier for transmitter
//When testing via IR led, the receiver waveform is usually active-low.
#define RMT_RX_ACTIVE_LEVEL  0   /*!< If we connect with a IR receiver, the data is active low */
#define RMT_TX_CARRIER_EN    1   /*!< Enable carrier for IR transmitter test with IR led */
#endif

// side
#define RMT_TX_CHANNEL_S    1     /*!< RMT channel for transmitter */
#define RMT_TX_GPIO_NUM_S  17     /*!< GPIO number for transmitter signal */
#define RMT_RX_CHANNEL_S    0     /*!< RMT channel for receiver */
#define RMT_RX_GPIO_NUM_S  16     /*!< GPIO number for receiver */

// front
#define RMT_TX_CHANNEL_F    3     /*!< RMT channel for transmitter */
#define RMT_TX_GPIO_NUM_F  26     /*!< GPIO number for transmitter signal */
#define RMT_RX_CHANNEL_F    2     /*!< RMT channel for receiver */
#define RMT_RX_GPIO_NUM_F  25     /*!< GPIO number for receiver */

#define RMT_CLK_DIV      100    /*!< RMT counter clock divider */
#define RMT_TICK_10_US    (80000000/RMT_CLK_DIV/100000)   /*!< RMT counter value for 10 us.(Source clock is APB clock) */

#define NEC_HEADER_HIGH_US    9000                         /*!< NEC protocol header: positive 9ms */
#define NEC_HEADER_LOW_US     4500                         /*!< NEC protocol header: negative 4.5ms*/
#define NEC_BIT_ONE_HIGH_US    560                         /*!< NEC protocol data bit 1: positive 0.56ms */
#define NEC_BIT_ONE_LOW_US    (2250-NEC_BIT_ONE_HIGH_US)   /*!< NEC protocol data bit 1: negative 1.69ms */
#define NEC_BIT_ZERO_HIGH_US   560                         /*!< NEC protocol data bit 0: positive 0.56ms */
#define NEC_BIT_ZERO_LOW_US   (1120-NEC_BIT_ZERO_HIGH_US)  /*!< NEC protocol data bit 0: negative 0.56ms */
#define NEC_BIT_END            560                         /*!< NEC protocol end: positive 0.56ms */
#define NEC_BIT_MARGIN         20                          /*!< NEC parse margin time */

#define NEC_ITEM_DURATION(d)  ((d & 0x7fff)*10/RMT_TICK_10_US)  /*!< Parse duration time from memory register value */
#define NEC_DATA_ITEM_NUM   34  /*!< NEC code item number: header + 32bit data + end */
#define RMT_TX_DATA_NUM  100    /*!< NEC tx test data number */
#define rmt_item32_tIMEOUT_US  100000   /*!< RMT receiver timeout value(us) */

// RMT /////////////////////////////////////////////////////////////////////////////

/*
 * @brief Build register value of waveform for NEC one data bit
 */
static inline void nec_fill_item_level(rmt_item32_t* item, int high_us, int low_us)
{
    item->level0 = 1;
    item->duration0 = (high_us) / 10 * RMT_TICK_10_US;
    item->level1 = 0;
    item->duration1 = (low_us) / 10 * RMT_TICK_10_US;
}

/*
 * @brief Generate NEC header value: active 9ms + negative 4.5ms
 */
static void nec_fill_item_header(rmt_item32_t* item)
{
    nec_fill_item_level(item, NEC_HEADER_HIGH_US, NEC_HEADER_LOW_US);
}

/*
 * @brief Generate NEC data bit 1: positive 0.56ms + negative 1.69ms
 */
static void nec_fill_item_bit_one(rmt_item32_t* item)
{
    nec_fill_item_level(item, NEC_BIT_ONE_HIGH_US, NEC_BIT_ONE_LOW_US);
}

/*
 * @brief Generate NEC data bit 0: positive 0.56ms + negative 0.56ms
 */
static void nec_fill_item_bit_zero(rmt_item32_t* item)
{
    nec_fill_item_level(item, NEC_BIT_ZERO_HIGH_US, NEC_BIT_ZERO_LOW_US);
}

/*
 * @brief Generate NEC end signal: positive 0.56ms
 */
static void nec_fill_item_end(rmt_item32_t* item)
{
    nec_fill_item_level(item, NEC_BIT_END, 0x7fff);
}

/*
 * @brief Check whether duration is around target_us
 */
inline bool nec_check_in_range(int duration_ticks, int target_us, int margin_us)
{
    if(( NEC_ITEM_DURATION(duration_ticks) < (target_us + margin_us))
       && ( NEC_ITEM_DURATION(duration_ticks) > (target_us - margin_us))) {
        return true;
    } else {
        return false;
    }
}

/*
 * @brief Check whether this value represents an NEC header
 */
static bool nec_header_if(rmt_item32_t* item)
{
    if((item->level0 == RMT_RX_ACTIVE_LEVEL && item->level1 != RMT_RX_ACTIVE_LEVEL)
       && nec_check_in_range(item->duration0, NEC_HEADER_HIGH_US, NEC_BIT_MARGIN)
       && nec_check_in_range(item->duration1, NEC_HEADER_LOW_US, NEC_BIT_MARGIN)) {
        return true;
    }
    return false;
}

/*
 * @brief Check whether this value represents an NEC data bit 1
 */
static bool nec_bit_one_if(rmt_item32_t* item)
{
    if((item->level0 == RMT_RX_ACTIVE_LEVEL && item->level1 != RMT_RX_ACTIVE_LEVEL)
       && nec_check_in_range(item->duration0, NEC_BIT_ONE_HIGH_US, NEC_BIT_MARGIN)
       && nec_check_in_range(item->duration1, NEC_BIT_ONE_LOW_US, NEC_BIT_MARGIN)) {
        return true;
    }
    return false;
}

/*
 * @brief Check whether this value represents an NEC data bit 0
 */
static bool nec_bit_zero_if(rmt_item32_t* item)
{
    if((item->level0 == RMT_RX_ACTIVE_LEVEL && item->level1 != RMT_RX_ACTIVE_LEVEL)
       && nec_check_in_range(item->duration0, NEC_BIT_ZERO_HIGH_US, NEC_BIT_MARGIN)
       && nec_check_in_range(item->duration1, NEC_BIT_ZERO_LOW_US, NEC_BIT_MARGIN)) {
        return true;
    }
    return false;
}


/*
 * @brief Parse NEC 32 bit waveform to address and command.
 */
static int nec_parse_items(rmt_item32_t* item, int item_num, uint16_t* addr, uint16_t* data)
{
    int w_len = item_num;
    if(w_len < NEC_DATA_ITEM_NUM) {
        return -1;
    }
    int i = 0, j = 0;
    if(!nec_header_if(item++)) {
        return -1;
    }
    uint16_t addr_t = 0;
    for(j = 0; j < 16; j++) {
        if(nec_bit_one_if(item)) {
            addr_t |= (1 << j);
        } else if(nec_bit_zero_if(item)) {
            addr_t |= (0 << j);
        } else {
            return -1;
        }
        item++;
        i++;
    }
    uint16_t data_t = 0;
    for(j = 0; j < 16; j++) {
        if(nec_bit_one_if(item)) {
            data_t |= (1 << j);
        } else if(nec_bit_zero_if(item)) {
            data_t |= (0 << j);
        } else {
            return -1;
        }
        item++;
        i++;
    }
    *addr = addr_t;
    *data = data_t;
    return i;
}

/*
 * @brief Build NEC 32bit waveform.
 */
static int nec_build_items(int channel, rmt_item32_t* item, int item_num, uint16_t addr, uint16_t cmd_data)
{
    int i = 0, j = 0;
    if(item_num < NEC_DATA_ITEM_NUM) {
        return -1;
    }
    nec_fill_item_header(item++);
    i++;
    for(j = 0; j < 16; j++) {
        if(addr & 0x1) {
            nec_fill_item_bit_one(item);
        } else {
            nec_fill_item_bit_zero(item);
        }
        item++;
        i++;
        addr >>= 1;
    }
    for(j = 0; j < 16; j++) {
        if(cmd_data & 0x1) {
            nec_fill_item_bit_one(item);
        } else {
            nec_fill_item_bit_zero(item);
        }
        item++;
        i++;
        cmd_data >>= 1;
    }
    nec_fill_item_end(item);
    i++;
    return i;
}

/*
 * @brief RMT transmitter initialization
 */
static void nec_tx_init_F(void)
{
    // front
    rmt_config_t rmt_tx_f;
    rmt_tx_f.channel = RMT_TX_CHANNEL_F;
    rmt_tx_f.gpio_num = RMT_TX_GPIO_NUM_F;
    rmt_tx_f.mem_block_num = 1;
    rmt_tx_f.clk_div = RMT_CLK_DIV;
    rmt_tx_f.tx_config.loop_en = false;
    rmt_tx_f.tx_config.carrier_duty_percent = 50;
    rmt_tx_f.tx_config.carrier_freq_hz = 38000;
    rmt_tx_f.tx_config.carrier_level = 1;
    rmt_tx_f.tx_config.carrier_en = RMT_TX_CARRIER_EN;
    rmt_tx_f.tx_config.idle_level = 0;
    rmt_tx_f.tx_config.idle_output_en = true;
    rmt_tx_f.rmt_mode = 0;
    rmt_config(&rmt_tx_f);
    rmt_driver_install(rmt_tx_f.channel, 0, 0);
}

static void nec_tx_init_S(void)
{
    // side
    rmt_config_t rmt_tx_s;
    rmt_tx_s.channel = RMT_TX_CHANNEL_S;
    rmt_tx_s.gpio_num = RMT_TX_GPIO_NUM_S;
    rmt_tx_s.mem_block_num = 1;
    rmt_tx_s.clk_div = RMT_CLK_DIV;
    rmt_tx_s.tx_config.loop_en = false;
    rmt_tx_s.tx_config.carrier_duty_percent = 50;
    rmt_tx_s.tx_config.carrier_freq_hz = 38000;
    rmt_tx_s.tx_config.carrier_level = 1;
    rmt_tx_s.tx_config.carrier_en = RMT_TX_CARRIER_EN;
    rmt_tx_s.tx_config.idle_level = 0;
    rmt_tx_s.tx_config.idle_output_en = true;
    rmt_tx_s.rmt_mode = 0;
    rmt_config(&rmt_tx_s);
    rmt_driver_install(rmt_tx_s.channel, 0, 0);
}


/*
 * @brief RMT receiver initialization
 */
static void nec_rx_init_F(void)
{
    // front
    rmt_config_t rmt_rx_f;
    rmt_rx_f.channel = RMT_RX_CHANNEL_F;
    rmt_rx_f.gpio_num = RMT_RX_GPIO_NUM_F;
    rmt_rx_f.clk_div = RMT_CLK_DIV;
    rmt_rx_f.mem_block_num = 1;
    rmt_rx_f.rmt_mode = RMT_MODE_RX;
    rmt_rx_f.rx_config.filter_en = true;
    rmt_rx_f.rx_config.filter_ticks_thresh = 100;
    rmt_rx_f.rx_config.idle_threshold = rmt_item32_tIMEOUT_US / 10 * (RMT_TICK_10_US);
    rmt_config(&rmt_rx_f);
    rmt_driver_install(rmt_rx_f.channel, 1000, 0);
}

static void nec_rx_init_S(void)
{
    // side
    rmt_config_t rmt_rx_s;
    rmt_rx_s.channel = RMT_RX_CHANNEL_S;
    rmt_rx_s.gpio_num = RMT_RX_GPIO_NUM_S;
    rmt_rx_s.clk_div = RMT_CLK_DIV;
    rmt_rx_s.mem_block_num = 1;
    rmt_rx_s.rmt_mode = RMT_MODE_RX;
    rmt_rx_s.rx_config.filter_en = true;
    rmt_rx_s.rx_config.filter_ticks_thresh = 100;
    rmt_rx_s.rx_config.idle_threshold = rmt_item32_tIMEOUT_US / 10 * (RMT_TICK_10_US);
    rmt_config(&rmt_rx_s);
    rmt_driver_install(rmt_rx_s.channel, 1000, 0);
}


// rx for FRONT
static void rmt_example_nec_rx_task_F(void *arg)
{
    int channel = RMT_RX_CHANNEL_F;
    nec_rx_init_F();
    RingbufHandle_t rb = NULL;
    //get RMT RX ringbuffer
    rmt_get_ringbuf_handle(channel, &rb);
    rmt_rx_start(channel, 1);
    double distance = 0;
    rmt_item32_t item;
    item.level0 = 1;
    item.duration0 = RMT_TICK_10_US;
    item.level1 = 0;
    item.duration1 = RMT_TICK_10_US; // for one pulse this doesn't matter
    while(rb) {
        size_t rx_size = 0;
        //try to receive data from ringbuffer.
        //RMT driver will push all the data it receives to its ringbuffer.
        //We just need to parse the value and return the spaces of ringbuffer.
        rmt_item32_t* item = (rmt_item32_t*) xRingbufferReceive(rb, &rx_size, 1000);
        if(item) {
            distance = 340.29 * NEC_ITEM_DURATION(item->duration0) / (1000 * 1000 * 2); // meters
            printf("FRONT: %f cm\n", distance * 100); // distance in centimeters
            vRingbufferReturnItem(rb, (void*) item);
        } else {
            break;
        }
    }
    vTaskDelete(NULL);
}

// tx for FRONT
static void rmt_example_nec_tx_task_F(void *arg)
{
    vTaskDelay(10);
    nec_tx_init_F();
    esp_log_level_set(NEC_TAG, ESP_LOG_INFO);
    int channel = RMT_TX_CHANNEL_F;
    uint16_t cmd = 0x0;
    uint16_t addr = 0x11;
    int nec_tx_num = RMT_TX_DATA_NUM;
    for(;;) {
        // ESP_LOGI(NEC_TAG, "RMT TX DATA");
        size_t size = (sizeof(rmt_item32_t) * NEC_DATA_ITEM_NUM * nec_tx_num);
        //each item represent a cycle of waveform.
        rmt_item32_t* item = (rmt_item32_t*) malloc(size);
        int item_num = NEC_DATA_ITEM_NUM * nec_tx_num;
        
        //To send data according to the waveform items.
        rmt_write_items(channel, item, item_num, true);
        //Wait until sending is done.
        rmt_wait_tx_done(channel, portMAX_DELAY);
        //before we free the data, make sure sending is already done.
        free(item);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

// rx for SIDE
static void rmt_example_nec_rx_task_S(void *arg)
{
    int channel = RMT_RX_CHANNEL_S;
    nec_rx_init_S();
    RingbufHandle_t rb = NULL;
    //get RMT RX ringbuffer
    rmt_get_ringbuf_handle(channel, &rb);
    rmt_rx_start(channel, 1);
    double distance = 0;
    rmt_item32_t item;
    item.level0 = 1;
    item.duration0 = RMT_TICK_10_US;
    item.level1 = 0;
    item.duration1 = RMT_TICK_10_US; // for one pulse this doesn't matter
    while(rb) {
        size_t rx_size = 0;
        //try to receive data from ringbuffer.
        //RMT driver will push all the data it receives to its ringbuffer.
        //We just need to parse the value and return the spaces of ringbuffer.
        rmt_item32_t* item = (rmt_item32_t*) xRingbufferReceive(rb, &rx_size, 1000);
        if(item) {
            distance = 340.29 * NEC_ITEM_DURATION(item->duration0) / (1000 * 1000 * 2); // meters
            printf("SIDE: %f cm\n", distance * 100); // distance in centimeters
            vRingbufferReturnItem(rb, (void*) item);
        } else {
            break;
        }
    }
    vTaskDelete(NULL);
}

// tx for SIDE
static void rmt_example_nec_tx_task_S(void *arg)
{
    vTaskDelay(10);
    nec_tx_init_S();
    esp_log_level_set(NEC_TAG, ESP_LOG_INFO);
    int channel = RMT_TX_CHANNEL_S;
    uint16_t cmd = 0x0;
    uint16_t addr = 0x11;
    int nec_tx_num = RMT_TX_DATA_NUM;
    for(;;) {
        // ESP_LOGI(NEC_TAG, "RMT TX DATA");
        size_t size = (sizeof(rmt_item32_t) * NEC_DATA_ITEM_NUM * nec_tx_num);
        //each item represent a cycle of waveform.
        rmt_item32_t* item = (rmt_item32_t*) malloc(size);
        int item_num = NEC_DATA_ITEM_NUM * nec_tx_num;
        
        //To send data according to the waveform items.
        rmt_write_items(channel, item, item_num, true);
        //Wait until sending is done.
        rmt_wait_tx_done(channel, portMAX_DELAY);
        //before we free the data, make sure sending is already done.
        free(item);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}


void app_main(void)
{
    xTaskCreate(rmt_example_nec_rx_task_F, "rmt_nec_rx_task_F", 2048, NULL, 10, NULL);
    xTaskCreate(rmt_example_nec_tx_task_F, "rmt_nec_tx_task_F", 2048, NULL, 10, NULL);
    xTaskCreate(rmt_example_nec_rx_task_S, "rmt_nec_rx_task_S", 2048, NULL, 10, NULL);
    xTaskCreate(rmt_example_nec_tx_task_S, "rmt_nec_tx_task_S", 2048, NULL, 10, NULL);
}
