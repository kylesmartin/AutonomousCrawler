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
#include "./ADXL343.h"
#include "displaychars.h"

//ir comm
#include "driver/rmt.h"

// Timer Init //////////////////////////////////////////////////////////////////////
double dt = 1;

#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds

#define TIMER_INTERVAL0_SEC   (dt) // sample test interval for the first timer

#define TEST_WITHOUT_RELOAD   0        // testing will be done without auto reload

typedef struct {
    int type;  // the type of timer's event
    int timer_group;
    int timer_idx;
    uint64_t timer_counter_value;
} timer_event_t;

xQueueHandle timer_queue;

////PCNT SETUP///////////////////////////////////////////////////////////////////

//pcnt settings
#define PCNT_TEST_UNIT      PCNT_UNIT_0
#define PCNT_H_LIM_VAL      100
#define PCNT_L_LIM_VAL     -10
#define PCNT_THRESH1_VAL    100
#define PCNT_THRESH0_VAL    0.20F
#define PCNT_INPUT_SIG_IO   34  // Pulse Input GPIO
#define PCNT_INPUT_CTRL_IO  5  // Control GPIO HIGH=count up, LOW=count down
#define LEDC_OUTPUT_IO      18 // Output GPIO of a sample 1 Hz pulse generator

int timeCounter = 0;
int blackStripeCount = 0;

xQueueHandle pcnt_evt_queue;   // A queue to handle pulse counter events
pcnt_isr_handle_t user_isr_handle = NULL; //user's ISR service handle

/* A sample structure to pass events from the PCNT
 * interrupt handler to the main program.
 */
typedef struct {
    int unit;  // the PCNT unit that originated an interrupt
    uint32_t status; // information on the event type that caused the interrupt
} pcnt_evt_t;

////DRIVING SETUP///////////////////////////////////////////////////////////////////

//You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define DRIVE_MIN_PULSEWIDTH 900 //Minimum pulse width in microsecond
#define DRIVE_MAX_PULSEWIDTH 1900 //Maximum pulse width in microsecond
#define DRIVE_MAX_DEGREE 180 //Maximum angle in degree upto which servo can rotate
#define STEERING_MIN_PULSEWIDTH 700 //Minimum pulse width in microsecond
#define STEERING_MAX_PULSEWIDTH 2100 //Maximum pulse width in microsecond
#define STEERING_MAX_DEGREE 180 //Maximum angle in degree upto which servo can rotate

uint32_t angle_duty = 90; // actuation
uint32_t drive_duty = 1400; // actuation
char split[100];
bool auto_state = 0;
int cycles = 0;

////I2C SETUP///////////////////////////////////////////////////////////////////

// Master I2C
#define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO          23   // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000     // i2c master clock freq
#define WRITE_BIT                          I2C_MASTER_WRITE // i2c master write
#define READ_BIT                           I2C_MASTER_READ  // i2c master read
#define ACK_CHECK_EN                       true // i2c master will check ack
#define ACK_CHECK_DIS                      false// i2c master will not check ack
#define ACK_VAL                            0x00 // i2c ack value
#define NACK_VAL                           0xFF // i2c nack value

// ADXL343
#define ACCEL_ADDR                         ADXL343_ADDRESS // 0x53

// 14-Segment Display
#define ALPHA_ADDR                         0x70 // alphanumeric address
#define OSC                                0x21 // oscillator cmd
#define HT16K33_BLINK_DISPLAYON            0x01 // Display on cmd
#define HT16K33_BLINK_OFF                  0    // Blink off cmd
#define HT16K33_BLINK_CMD                  0x80 // Blink cmd
#define HT16K33_CMD_BRIGHTNESS             0xE0 // Brightness cmd

////UART SETUP///////////////////////////////////////////////////////////////////
static const int RX_BUF_SIZEir = 120;
static const int RX_BUF_SIZEus = 120;

#define TXD_PIN_FRONT (GPIO_NUM_17)
#define RXD_PIN_FRONT (GPIO_NUM_16)
#define TXD_PIN_SIDE  (GPIO_NUM_4)
#define RXD_PIN_SIDE  (GPIO_NUM_36)
#define IR_RX (GPIO_NUM_39)
#define IR_TX (GPIO_NUM_32) //unused

// Default ID/color
#define ID 3
#define COLOR 'R'
#define FULLSPEED 1270
#define LOWSPEED 1280
#define STOPPED 1400

//control flags
int prevId = 1; //last beacon seen
bool isAutonomous = true;
int inputArr[3] = {0, 0, 1};

// Variables for my ID, minVal and status plus string fragments
char start = 0x1B;
char myID = (char) ID;
char myColor = (char) COLOR;
int len_out = 10;

float base_x_acceleration;

////WIFI & SOCKET SETUP///////////////////////////////////////////////////////////////////

//socket variables
  #define HOST_IP_ADDR "192.168.43.116"                    //target server ip
#define PORT 3333                                       //target server port
char rx_buffer[128];
char addr_str[128];
int addr_family;
int ip_protocol;
int sock;                                               //socket id?
struct sockaddr_in dest_addr;                           //socket destination info

//wifi variables
#define EXAMPLE_ESP_WIFI_SSID "Group_2"
#define EXAMPLE_ESP_WIFI_PASS "smartkey"
#define EXAMPLE_ESP_MAXIMUM_RETRY 10                    //CONFIG_ESP_MAXIMUM_RETRY
static EventGroupHandle_t s_wifi_event_group;           //FreeRTOS event group to signal when we are connected
const int WIFI_CONNECTED_BIT = BIT0;                    //The event group allows multiple bits for each event, but we only care about one event - are we connected to the AP with an IP?
static const char *TAG = "meep";
static int s_retry_num = 0;

bool running = true;
bool mode = 0; // automatic = 0, manual = 1

//function headers
void pid_speed();
void pid_steering();
void manual_speed_adj(int mode);
void manual_steer_adj(int mode);
void adjust(int front, int side);

//RMT
#include "freertos/semphr.h"

static const char* NEC_TAG = "NEC";

// RMT /////////////////////////////////////////////////////////////////////////////

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

float front = 50.0;
float side = 50.0;

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
            front = distance * 100; // distance in centimeters
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
            side = distance * 100; // distance in centimeters
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

////WIFI SETUP/////////////////////////////////////////////////////////////////////

//wifi event handler
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:%s",
                 ip4addr_ntoa(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

//connect to wifi
void wifi_init_sta(void) {

    //Initialize NVS
    printf("Wifi setup part 1\n");
    esp_err_t ret = nvs_flash_init();
    printf("Wifi setup part 2\n");
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }

    printf("Wifi setup part 3\n");

    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");

    s_wifi_event_group = xEventGroupCreate();
    tcpip_adapter_init();

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");
    ESP_LOGI(TAG, "connect to ap SSID:%s password:%s",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);

}


////SOCKET SETUP/////////////////////////////////////////////////////////////////////

static void udp_init() {

    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;

    //socket setup struct
    dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);
    inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);

    //establish socket connection
    sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
    }
    ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);
}

static void udp_client_receive() {

    while(1) {

        while(1) {

            printf("waiting for a message\n");

            struct sockaddr_in source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            printf("got one!\n");

            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                //ESP_LOGI(TAG, "%s", rx_buffer);
                printf("received string: %s\n", rx_buffer);

                // Extract the first token
                char * token = strtok(rx_buffer, " ");

                // loop through the string to extract all other tokens
                int i = 0;
                while(token != NULL) {
                    int dir = atoi(token);
                    printf( " %d\n", dir ); //printing each token
                    inputArr[i] = dir;
                    token = strtok(NULL, " ");
                    i++;
                }

                /*
                if (strstr(rx_buffer, "start") != NULL) { //0 if equal
                    running = true;
                } else if (strstr(rx_buffer, "stop") != NULL) {
                    running = false;
                } else if (strstr(rx_buffer, "automatic") != NULL) {
                    mode = 0;
                } else if (strstr(rx_buffer, "manual") != NULL) {
                    mode = 1;
                } else if (strstr(rx_buffer, "speedup") != NULL) {
                    if (mode) {
                        // increase speed = 1
                        manual_speed_adj(1);
                    }
                } else if (strstr(rx_buffer, "slowdown") != NULL) {
                    if (mode) {
                        // decrease speed = 0
                        manual_speed_adj(0);
                    }
                } else if (strstr(rx_buffer, "right") != NULL) {
                    if (mode) {
                        // right = 1
                        manual_steer_adj(1);
                    }
                } else if (strstr(rx_buffer, "left") != NULL) {
                    if (mode) {
                        // left = 0
                        manual_steer_adj(0);
                    }
                }
                */

            }

        }

        //shut down socket if anything failed
        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
            udp_init();
        }

    }

    vTaskDelete(NULL);

}

static void udp_client_send(char* message) {

    //assuming ip4v only
    //printf("%s\n", message);

    //send message, and print error if anything failed
    int err = sendto(sock, message, strlen(message), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
    }
}

////PCNT FUNCTIONS///////////////////////////////////////////////////////////////////

/* Decode what PCNT's unit originated an interrupt
 * and pass this information together with the event type
 * the main program using a queue.
 */
static void IRAM_ATTR pcnt_example_intr_handler(void *arg)
{
    uint32_t intr_status = PCNT.int_st.val;
    int i;
    pcnt_evt_t evt;
    portBASE_TYPE HPTaskAwoken = pdFALSE;

    for (i = 0; i < PCNT_UNIT_MAX; i++) {
        if (intr_status & (BIT(i))) {
            evt.unit = i;
            /* Save the PCNT event type that caused an interrupt
               to pass it to the main program */
            evt.status = PCNT.status_unit[i].val;
            PCNT.int_clr.val = BIT(i);
            xQueueSendFromISR(pcnt_evt_queue, &evt, &HPTaskAwoken);
            if (HPTaskAwoken == pdTRUE) {
                portYIELD_FROM_ISR();
            }
        }
    }
}

/* Initialize PCNT functions:
 *  - configure and initialize PCNT
 *  - set up the input filter
 *  - set up the counter events to watch
 */
static void pcnt_example_init(void)
{

    /* Initialize PCNT event queue and PCNT functions */
    pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
    /* Prepare configuration for the PCNT unit */
    pcnt_config_t pcnt_config = {
        // Set PCNT input signal and control GPIOs
        .pulse_gpio_num = PCNT_INPUT_SIG_IO,
        .ctrl_gpio_num = PCNT_INPUT_CTRL_IO,
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_TEST_UNIT,
        // What to do on the positive / negative edge of pulse input?
        .pos_mode = PCNT_COUNT_INC,   // Count up on the positive edge
        .neg_mode = PCNT_COUNT_DIS,   // Keep the counter value on the negative edge
        // What to do when control input is low or high?
        .lctrl_mode = PCNT_MODE_REVERSE, // Reverse counting direction if low
        .hctrl_mode = PCNT_MODE_KEEP,    // Keep the primary counter mode if high
        // Set the maximum and minimum limit values to watch
        .counter_h_lim = PCNT_H_LIM_VAL,
        .counter_l_lim = PCNT_L_LIM_VAL,
    };
    /* Initialize PCNT unit */
    pcnt_unit_config(&pcnt_config);

    /* Configure and enable the input filter */
    pcnt_set_filter_value(PCNT_TEST_UNIT, 100);
    pcnt_filter_enable(PCNT_TEST_UNIT);

    /* Set threshold 0 and 1 values and enable events to watch */
    pcnt_set_event_value(PCNT_TEST_UNIT, PCNT_EVT_THRES_1, PCNT_THRESH1_VAL);
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_THRES_1);
    pcnt_set_event_value(PCNT_TEST_UNIT, PCNT_EVT_THRES_0, PCNT_THRESH0_VAL);
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_THRES_0);
    /* Enable events on zero, maximum and minimum limit values */
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_ZERO);
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_H_LIM);
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_L_LIM);

    /* Initialize PCNT's counter */
    pcnt_counter_pause(PCNT_TEST_UNIT);
    pcnt_counter_clear(PCNT_TEST_UNIT);

    /* Register ISR handler and enable interrupts for PCNT unit */
    pcnt_isr_register(pcnt_example_intr_handler, NULL, 0, &user_isr_handle);
    pcnt_intr_enable(PCNT_TEST_UNIT);

    /* Everything is set up, now go to counting */
    pcnt_counter_resume(PCNT_TEST_UNIT);
}

static int pcnt_read(int delay) {

    pcnt_example_init();

    int16_t count = 0;
    pcnt_evt_t evt;
    portBASE_TYPE res;
    /* Wait for the event information passed from PCNT's interrupt handler.
     * Once received, decode the event type and print it on the serial monitor.
     */
    res = xQueueReceive(pcnt_evt_queue, &evt, delay / portTICK_PERIOD_MS);
    if (res == pdTRUE) {
        pcnt_get_counter_value(PCNT_TEST_UNIT, &count);
        printf("Event PCNT unit[%d]; cnt: %d\n", evt.unit, count);
        if (evt.status & PCNT_STATUS_THRES1_M) {
            printf("THRES1 EVT\n");
        }
        if (evt.status & PCNT_STATUS_THRES0_M) {
            printf("THRES0 EVT\n");
        }
        if (evt.status & PCNT_STATUS_L_LIM_M) {
            printf("L_LIM EVT\n");
        }
        if (evt.status & PCNT_STATUS_H_LIM_M) {
            printf("H_LIM EVT\n");
        }
        if (evt.status & PCNT_STATUS_ZERO_M) {
            printf("ZERO EVT\n");
        }
    } else {
        pcnt_get_counter_value(PCNT_TEST_UNIT, &count);
        //printf("Current counter value :%d\n", count);
    }

    //clear the internal counter for the next reading
    pcnt_counter_clear(PCNT_TEST_UNIT);

    if(user_isr_handle) {
        //Free the ISR service handle.
        esp_intr_free(user_isr_handle);
        user_isr_handle = NULL;
    }

    return count;
}

////I2C FUNCTIONS///////////////////////////////////////////////////////////////////

// Function to initiate i2c -- note the MSB declaration!
static void i2c_master_init(){
    // Debug
    printf("\n>> i2c Config\n");
    int err;

    // Port configuration
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;

    /// Define I2C configurations
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;                              // Master mode
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;              // Default SDA pin
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;              // Default SCL pin
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;       // CLK frequency
    err = i2c_param_config(i2c_master_port, &conf);           // Configure
    if (err == ESP_OK) {printf("- parameters: ok\n");}

    // Install I2C driver
    err = i2c_driver_install(i2c_master_port, conf.mode,
                     I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                     I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
    if (err == ESP_OK) {printf("- initialized: yes\n");}

    // Data in MSB mode
    i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

// Utility function to test for I2C device address -- not used in deploy
int testConnection(uint8_t devAddr, int32_t timeout) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

// Utility function to scan for i2c device
static void i2c_scanner() {
    int32_t scanTimeout = 1000;
    printf("\n>> I2C scanning ..."  "\n");
    uint8_t count = 0;
    for (uint8_t i = 1; i < 127; i++) {
        // printf("0x%X%s",i,"\n");
        if (testConnection(i, scanTimeout) == ESP_OK) {
            printf( "- Device found at address: 0x%X%s", i, "\n");
            count++;
        }
    }
    if (count == 0)
        printf("- No I2C devices found!" "\n");
    printf("\n");
}

// Alphanumeric Functions //////////////////////////////////////////////////////

// Turn on oscillator for alpha display
int alpha_oscillator() {
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( ALPHA_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, OSC, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    vTaskDelay(200 / portTICK_RATE_MS);
    return ret;
}

// Set blink rate to off
int no_blink() {
    int ret;
    i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
    i2c_master_start(cmd2);
    i2c_master_write_byte(cmd2, ( ALPHA_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd2, HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (HT16K33_BLINK_OFF << 1), ACK_CHECK_EN);
    i2c_master_stop(cmd2);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd2, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd2);
    vTaskDelay(200 / portTICK_RATE_MS);
    return ret;
}

// Set Brightness
int set_brightness_max(uint8_t val) {
    int ret;
    i2c_cmd_handle_t cmd3 = i2c_cmd_link_create();
    i2c_master_start(cmd3);
    i2c_master_write_byte(cmd3, ( ALPHA_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd3, HT16K33_CMD_BRIGHTNESS | val, ACK_CHECK_EN);
    i2c_master_stop(cmd3);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd3, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd3);
    vTaskDelay(200 / portTICK_RATE_MS);
    return ret;
}

static void alpha_init() {
    // Debug
    int ret;
    printf(">> Test Alphanumeric Display: \n");

    // Set up routines
    // Turn on alpha oscillator
    ret = alpha_oscillator();
    if(ret == ESP_OK) {printf("- oscillator: ok \n");}
    // Set display blink off
    ret = no_blink();
    if(ret == ESP_OK) {printf("- blink: off \n");}
    ret = set_brightness_max(0xF);
    if(ret == ESP_OK) {printf("- brightness: max \n");}

}

//NOT THREAD SAFE
void alpha_write(double number) {
    int i, ret;

    uint16_t displaybuffer[8];
    char strIn[317]; //input string is 317 chars because largest possible sprintf output is 317 chars

    sprintf(strIn, "%04f", number);

    //fill displaybuffer with spaces to make display blank
    for (i = 0; i < 4; ++i) displaybuffer[i] = alphafonttable[' '];

    //populate display buffer with string contents
    i = 0;
    while (i < 4 && strIn[i] != '\0') {
        displaybuffer[i] = alphafonttable[(int)strIn[i]];
        ++i;
    }

    // Send commands characters to display over I2C
    i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
    i2c_master_start(cmd4);
    i2c_master_write_byte(cmd4, ( ALPHA_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
    for (uint8_t i=0; i<8; i++) {
        i2c_master_write_byte(cmd4, displaybuffer[i] & 0xFF, ACK_CHECK_EN);
        i2c_master_write_byte(cmd4, displaybuffer[i] >> 8, ACK_CHECK_EN);
    }
    i2c_master_stop(cmd4);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd4);

    if(ret == ESP_OK) {
        //printf("- wrote: T.D.C.L. \n\n");
    }
}

////UART FUNCTIONS///////////////////////////////////////////////////////////////////

void uart_init(void) {

    const uart_config_t ir_config = {
        .baud_rate = 1200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    //IR UART
    uart_param_config(UART_NUM_2, &ir_config);
    uart_set_pin(UART_NUM_2, IR_TX, IR_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_set_rts(UART_NUM_2, 1);
    uart_set_line_inverse(UART_NUM_2, UART_INVERSE_RXD);
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_2, RX_BUF_SIZEir * 2, 0, 0, NULL, 0);
}

//IR STUFF

bool checkCheckSum(uint8_t *p, int len) {
  char temp = (char) 0;
  bool isValid;
  for (int i = 0; i < len-1; i++){
    temp = temp^p[i];
  }
  // printf("Check: %02X ", temp);
  if (temp == p[len-1]) {
    isValid = true; }
  else {
    isValid = false; }
  return isValid;
}

// Receives task -- looks for Start byte then stores received values
char ir_rx_task() {
  // Buffer for input data
  uint8_t *data_in = (uint8_t *) malloc(RX_BUF_SIZEir);
  //while (1) {
    int len_in = uart_read_bytes(UART_NUM_2, data_in, RX_BUF_SIZEir, 20 / portTICK_RATE_MS);
    if (len_in >0) {
        printf("%d\n", len_in);
      //if (data_in[0] == start) {
        //if (checkCheckSum(data_in,len_out)) {

        int i;

        //find first "start" byte
        for (i = 0; i < len_in; i++) {
            if (data_in[i] == 0x1b) {
                break;
            }
        }

        char lightColor = data_in[i+1];
        uint8_t id = data_in[i+2];

        if (lightColor == 'G' || lightColor == 'Y' || lightColor == 'R') {

            //udp_client_send("received message!");

            if (checkCheckSum(data_in, 4)) {
                printf("Checksum checks out! Light is '%c', ID is %d\n", lightColor, id);
                printf("Speed: %d\tSteer: %d\n", inputArr[0], inputArr[1]);

                if (id != prevId) {

                    char timeBuf[10];


                    double splitTime;
                    timer_get_counter_time_sec(TIMER_GROUP_0, TIMER_0, &splitTime);

                    sprintf(timeBuf, "%f", splitTime);
                    printf("TIME: %s\n", timeBuf);
                    alpha_write(splitTime);
                    udp_client_send(timeBuf);

                    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
                    timer_start(TIMER_GROUP_0, TIMER_0);
                    prevId = id;
                }

                if (id == 3) {
                    printf("Switching to manual...");
                    isAutonomous = false;
                }


            }

            return lightColor;


        } else {
            printf("Invalid checksum!");
        }


        //ESP_LOG_BUFFER_HEXDUMP(TAG, data_in, len_out, ESP_LOG_INFO);
        //}
     //}
    }
    else{
      // printf("Nothing received.\n");
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
    //}
    free(data_in);

    return '\0';

}

int uart_sendData_front(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

int uart_sendData_side(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_2, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

static void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1) {
        uart_sendData_front(TX_TASK_TAG, "Hello world");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

int rx_task_front()
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZEus+1);
    char num[100];
    int i = 0;
    int sum = 0;
    int count = 0;
    //read raw input
    const int rxBytes = uart_read_bytes(UART_NUM_0, data, RX_BUF_SIZEus, 50 / portTICK_RATE_MS);
    //if got data back
    if (rxBytes > 0) {
        while(data[i] != 0x52) {
            i = i + 1;
        }
        for (int j = i; j < rxBytes; j = j + 5) {
            sprintf(num, "%c%c%c", (char)data[j+1], (char)data[j+2], (char)data[j+3]);
            sum = sum + atoi(num);
            count = count + 1;
        }
    }
    free(data);
    return (sum - atoi(num))/(count - 1);
}

int rx_task_side()
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZEus+1);
    char num[100];
    int i = 0;
    int sum = 0;
    int count = 0;
    //read raw input
    const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZEus, 50 / portTICK_RATE_MS);
    //if got data back
    if (rxBytes > 0) {
        while(data[i] != 0x52) {
            i = i + 1;
        }
        for (int j = i; j < rxBytes; j = j + 5) {
            sprintf(num, "%c%c%c", (char)data[j+1], (char)data[j+2], (char)data[j+3]);
            sum = sum + atoi(num);
            count = count + 1;
        }
    }
    free(data);
    return (sum - atoi(num))/(count - 1);
}

static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm servo control gpio......\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 12);    //Set GPIO 12 as PWM0A, to which drive wheels are connected
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, 19);    //Set GPIO 19 as PWM0A, to which steering servo is connected
}

/**
 * @brief Use this function to calcute pulse width for per degree rotation
 *
 * @param  degree_of_rotation the angle in degree to which servo has to rotate
 *
 * @return
 *     - calculated pulse width
 */
static uint32_t drive_per_degree_init(uint32_t degree_of_rotation)
{

    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (DRIVE_MIN_PULSEWIDTH + (((DRIVE_MAX_PULSEWIDTH - DRIVE_MIN_PULSEWIDTH) * (degree_of_rotation)) / (DRIVE_MAX_DEGREE)));
    return cal_pulsewidth;
}

/**
 * @brief Use this function to calcute pulse width for per degree rotation
 *
 * @param  degree_of_rotation the angle in degree to which servo has to rotate
 *
 * @return
 *     - calculated pulse width
 */
static uint32_t steering_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (STEERING_MIN_PULSEWIDTH + (((STEERING_MAX_PULSEWIDTH - STEERING_MIN_PULSEWIDTH) * (degree_of_rotation)) / (STEERING_MAX_DEGREE)));
    return cal_pulsewidth;
}

void pwm_init() {
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();

    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm......\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
}

void calibrateESC() {
    vTaskDelay(3000 / portTICK_PERIOD_MS);  // Give yourself time to turn on crawler
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 2100); // HIGH signal in microseconds - backwards
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 700);  // LOW signal in microseconds - forwards
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); // NEUTRAL signal in microseconds
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); // reset the ESC to neutral (non-moving) value
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

// Automatic Control ///////////////////////////////////////////////////////

void control(void *arg)
{


    double setpoint_st = 90; // cm from side wall
    uint32_t angle;
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, FULLSPEED);
    while (1) {

        running = inputArr[2]; //index 2 is running or not

        if (running) {

            


            char color = ir_rx_task(); //also handles split time


            if (isAutonomous) {

                if (color == 'R') {
                    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, STOPPED);
                } else {

                    if (color == 'Y') {
                        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, LOWSPEED);
                    } else if (color == 'G') {
                        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, FULLSPEED);
                    }

                    //mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, drive_duty - 130);


                    float curr_front = front;
                    float curr_side = side;
                    printf("FRONT: %.2f\n", curr_front);
                    printf("SIDE: %.2f\n", curr_side);
                    printf("\n");
                    //uint32_t angle = steering_per_degree_init(angle_duty);
                    //mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
                    adjust(curr_front, curr_side);
                    vTaskDelay(200/portTICK_RATE_MS);
                }

            } else {
                //manual mode

                //printf("Speed: %d\tSteer: %d\n", inputArr[0], inputArr[1]);

                manual_speed_adj(inputArr[0]);
                manual_steer_adj(inputArr[1]);



                

            }

            char dataBuf[100];

            sprintf(dataBuf, "Speed: %d\tSteer: %d\n", inputArr[0], inputArr[1]);

            //udp_client_send(dataBuf);
        } else {

            manual_speed_adj(0);

        }

        vTaskDelay(50/portTICK_RATE_MS);



        /*
        int avgDistFront = 0;//rx_task_front();
        adjust(setpoint_st, avgDistFront, avgDistSide);
        angle = steering_per_degree_init(angle_duty);
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
        vTaskDelay(100/portTICK_RATE_MS);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
        */
    }

    vTaskDelete(NULL);
}

void adjust(int f, int s) {
    float setpoint = 70.0; // cm from side wall
    float walldist = 150;
    uint32_t angle;
    // in all cases, adjust angle duty
    //if (f > 90 && s > 10) {

    if (auto_state == 0) {
        if (f > walldist && f < walldist+10) {
            // execute right turn
            auto_state = 1;
            printf("f < 100\n");
        }

        else if (s > 10) {
            printf("SIDE VALID\n");
            if (s < (setpoint - 10)) {
                // execute right tilt
                angle = steering_per_degree_init(angle_duty - 45);
                mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
                printf("s < (setpoint - 10))\n");
            } else if (s > (setpoint + 10)) {
                // execute left tilt
                angle = steering_per_degree_init(angle_duty + 45);
                mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
                printf("s > (setpoint + 10))\n");
            } else {
                // maintain orientation
                angle = steering_per_degree_init(angle_duty);
                mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
                printf("else\n");
            }

        } else {
            printf("INVALID\n");
        }
    }

    else {
        printf("TURNING\n");
        if (cycles > 20) {
            auto_state = 0;
            angle = steering_per_degree_init(angle_duty);
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
            cycles = 0;
        } else {
            angle = steering_per_degree_init(angle_duty - 90);
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
            cycles = cycles + 1;
        }
    }
    /*} else {
        printf("INVALID\n");
    }*/
}

// Manual Control //////////////////////////////////////////////////////

void manual_speed_adj(int multiplier) {

    int adjustedSpeed = drive_duty + (-30*multiplier);

    printf("pwm signal: %d\n", adjustedSpeed);


    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, adjustedSpeed);

}

void manual_steer_adj(int multiplier) {
    int angle = steering_per_degree_init(angle_duty + (30*multiplier));
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
}

// Timer setup //////////////////////////////////////////////////////////////

static void inline print_timer_counter(uint64_t counter_value)
{
    printf("Time   : %.8f s\n", (double) counter_value / TIMER_SCALE);
}

/*
 * Timer group0 ISR handler
 *
 * Note:
 * We don't call the timer API here because they are not declared with IRAM_ATTR.
 * If we're okay with the timer irq not being serviced while SPI flash cache is disabled,
 * we can allocate this interrupt without the ESP_INTR_FLAG_IRAM flag and use the normal API.
 */
void IRAM_ATTR timer_group0_isr(void *para)
{

    int timer_idx = (int) para;

    /* Retrieve the interrupt status and the counter value
     from the timer that reported the interrupt */
    timer_intr_t timer_intr = timer_group_intr_get_in_isr(TIMER_GROUP_0);
    uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(TIMER_GROUP_0, timer_idx);

    /* Prepare basic event data
     that will be then sent back to the main program task */
    timer_event_t evt;
    evt.timer_group = 0;
    evt.timer_idx = timer_idx;
    evt.timer_counter_value = timer_counter_value;

    /* Clear the interrupt
     and update the alarm time for the timer with without reload */
    if (timer_intr & TIMER_INTR_T0) {
        evt.type = TEST_WITHOUT_RELOAD;
        timer_group_intr_clr_in_isr(TIMER_GROUP_0, TIMER_0);
        timer_counter_value += (uint64_t) (TIMER_INTERVAL0_SEC * TIMER_SCALE);
        timer_group_set_alarm_value_in_isr(TIMER_GROUP_0, timer_idx, timer_counter_value);
    } else {
        evt.type = -1; // not supported even type
    }

    /* After the alarm has been triggered
     we need enable it again, so it is triggered the next time */
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, timer_idx);

    /* Now just send the event data back to the main program task */
    xQueueSendFromISR(timer_queue, &evt, NULL);
}

/*
 * Initialize selected timer of the timer group 0
 *
 * timer_idx - the timer number to initialize
 * auto_reload - should the timer auto reload on alarm?
 * timer_interval_sec - the interval of alarm to set
 */
static void example_tg0_timer_init(int timer_idx,
                                   bool auto_reload, double timer_interval_sec)
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = auto_reload;
    timer_init(TIMER_GROUP_0, timer_idx, &config);

    /* Timer's counter will initially start from value below.
     Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_sec * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, timer_idx);
    timer_isr_register(TIMER_GROUP_0, timer_idx, timer_group0_isr, (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);

    timer_start(TIMER_GROUP_0, timer_idx);

}

void app_main(void)
{

    // networking startup routines
    wifi_init_sta();
    udp_init();
    
    // I2C display startup routine
    i2c_master_init();
    i2c_scanner();
    alpha_init();
    alpha_write(0.0); //set default speed reading to 0

    
    // timer
    timer_queue = xQueueCreate(10, sizeof(timer_event_t));
    example_tg0_timer_init(TIMER_0, TEST_WITHOUT_RELOAD, TIMER_INTERVAL0_SEC);
        // Drive & steering startup routine
    printf("starting pwm\n");
    pwm_init();

    printf("starting uart\n");
    uart_init();
    printf("Calibrating motors...");
    calibrateESC();


    // wait for "start" signal from Node.js UDP server
    printf("Waiting for start signal...\n");
    udp_client_send("Test message");
    xTaskCreate(udp_client_receive, "udp_client_receive", 4096, NULL, 6, NULL); //also used later for getting stop signal

    //mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1200);

    // start up driving tasks
    printf("Starting up!\n\n\n\n\n\n\n\n\n\n\n\n");

    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    timer_start(TIMER_GROUP_0, TIMER_0);

    xTaskCreate(control, "control", 4096, NULL, 5, NULL);
    xTaskCreatePinnedToCore(rmt_example_nec_rx_task_F, "rmt_nec_rx_task_F", 2048, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(rmt_example_nec_tx_task_F, "rmt_nec_tx_task_F", 2048, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(rmt_example_nec_rx_task_S, "rmt_nec_rx_task_S", 2048, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(rmt_example_nec_tx_task_S, "rmt_nec_tx_task_S", 2048, NULL, 1, NULL, 1);

}
