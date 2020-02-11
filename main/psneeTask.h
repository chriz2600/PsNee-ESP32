#ifndef PSNEE_TASK
#define PSNEE_TASK

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#if CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ != 240
    #warning "ESP32 PsNee only tested with 240MHz CPU frequency"
#endif

#if CONFIG_FREERTOS_HZ < 250
    #error "ESP32 PsNee requires a freertos tick rate of at least 250 Hz"
#elif CONFIG_FREERTOS_HZ < 1000
    #warning "ESP32 PsNee only tested with a freertos tick rate of at least 1000 Hz"
#endif

#ifndef DEBUG1
    #define DEBUG1(...) printf(__VA_ARGS__)
#endif
#ifndef PSNEE_SQCK
    #define PSNEE_SQCK GPIO_NUM_4
#endif
#ifndef PSNEE_SUBQ
    #define PSNEE_SUBQ GPIO_NUM_21
#endif
#ifndef PSNEE_DATA
    #define PSNEE_DATA GPIO_NUM_13
#endif
#ifndef PSNEE_GATE_WFCK
    #define PSNEE_GATE_WFCK GPIO_NUM_12
#endif
#ifndef DELAY_BETWEEN_BITS
    #define DELAY_BETWEEN_BITS 4
#endif
#ifndef DELAY_BETWEEN_INJECTIONS
    #define DELAY_BETWEEN_INJECTIONS 90
#endif

#define PSNEE_INJECT(r, next) \
    if (psnee_inject_SCEX(r, bitcounter)) { \
        bitcounter++; \
        xDelay = DELAY_BETWEEN_BITS / portTICK_PERIOD_MS; \
    } else { \
        bitcounter = 0; \
        xDelay = DELAY_BETWEEN_INJECTIONS / portTICK_PERIOD_MS; \
        state = next; \
    }

typedef struct {
    xQueueHandle gpio_evt_queue;
    bool injection_running;
    int64_t event_tx_prev;
    bool pu22inject;
    bool pu22mode;
    bool modeDetected;
    unsigned int wfck_highs;
    unsigned int wfck_lows;
} PsNee;

enum PsNee_States { WaitForMode, WaitForQueue, StartInject, InjectSCEE, InjectSCEA, InjectSCEI, EndInject };
static PsNee psnee = { NULL, false, 0, false, false, false, 0, 0 };

static void IRAM_ATTR psnee_wfck_handler(void* arg) {
    if (!psnee.modeDetected) {
        if (gpio_get_level(PSNEE_GATE_WFCK)) {
            psnee.wfck_highs++;
        } else {
            psnee.wfck_lows++;
        }
    } else if (psnee.pu22inject) {
        gpio_set_level(PSNEE_DATA, gpio_get_level(PSNEE_GATE_WFCK)); // output wfck signal on PSNEE_DATA pin
    }
}

static void IRAM_ATTR psnee_sqck_handler(void* arg) {

    static int hysteresis = 0;
    static uint8_t scbuf [12] = { 0 }; // We will be capturing PSX "SUBQ" packets, there are 12 bytes per valid read.
    static uint8_t bitbuf = 0; // SUBQ bit storage
    static bool sample = 0;
    static uint8_t bitpos = 0;
    static uint8_t scpos = 0; // scbuf position

    uint32_t gpio_num = (uint32_t) arg;
    if ((esp_timer_get_time() - psnee.event_tx_prev) > 180) {
        scpos = 0;  // reset SUBQ packet stream
        bitbuf = 0;
        bitpos = 0;
    }

    sample = gpio_get_level(PSNEE_SUBQ);
    bitbuf |= sample << bitpos;
    
    if (bitpos < 7) {
        bitpos++;
    } else {
        scbuf[scpos] = bitbuf;
        bitbuf = 0;
        bitpos = 0;

        if (scpos < 11) {
            scpos++;
        } else {
            scpos = 0;
            bool isDataSector = (((scbuf[0] & 0x40) == 0x40) && (((scbuf[0] & 0x10) == 0) && ((scbuf[0] & 0x80) == 0)));
            
            if ( (isDataSector &&  scbuf[1] == 0x00 &&  scbuf[6] == 0x00) &&   // [0] = 41 means psx game disk. the other 2 checks are garbage protection
                 (scbuf[2] == 0xA0 || scbuf[2] == 0xA1 || scbuf[2] == 0xA2 ||  // if [2] = A0, A1, A2 ..
                 (scbuf[2] == 0x01 && (scbuf[3] >= 0x98 || scbuf[3] <= 0x02))) // .. or = 01 but then [3] is either > 98 or < 02
            ) {
                hysteresis++;
            }
            else if ( hysteresis > 0 &&
                    ((scbuf[0] == 0x01 || isDataSector) && (scbuf[1] == 0x00 /*|| scbuf[1] == 0x01*/) &&  scbuf[6] == 0x00)
                    ) {  // This CD has the wobble into CD-DA space. (started at 0x41, then went into 0x01)
                hysteresis++;
            }
            else if (hysteresis > 0) {
                hysteresis--; // None of the above. Initial detection was noise. Decrease the counter.
            }

            // hysteresis value "optimized" using very worn but working drive on ATmega328 @ 16Mhz
            // should be fine on other MCUs and speeds, as the PSX dictates SUBQ rate
            if (hysteresis >= 14) {
                hysteresis = 11;
                if (!psnee.injection_running) {
                    xQueueSendFromISR(psnee.gpio_evt_queue, &gpio_num, NULL);
                }
            }
        }
    }
    psnee.event_tx_prev = esp_timer_get_time();
}


// borrowed from AttyNee. Bitmagic to get to the SCEX strings stored in flash (because Harvard architecture)
bool psnee_readBit(int index, const unsigned char *ByteSet)
{
    int byte_index = index >> 3;
    uint8_t bits = ByteSet[byte_index];
    int bit_index = index & 0x7; // same as (index - byte_index<<3) or (index%8)
    uint8_t mask = 1 << bit_index;
    return (0 != (bits & mask));
}

bool psnee_inject_SCEX(char region, uint8_t bit_counter)
{
    //SCEE: 1 00110101 00, 1 00111101 00, 1 01011101 00, 1 01011101 00
    //SCEA: 1 00110101 00, 1 00111101 00, 1 01011101 00, 1 01111101 00
    //SCEI: 1 00110101 00, 1 00111101 00, 1 01011101 00, 1 01101101 00
    static const unsigned char SCEEData[] = {0b01011001, 0b11001001, 0b01001011, 0b01011101, 0b11101010, 0b00000010};
    static const unsigned char SCEAData[] = {0b01011001, 0b11001001, 0b01001011, 0b01011101, 0b11111010, 0b00000010};
    static const unsigned char SCEIData[] = {0b01011001, 0b11001001, 0b01001011, 0b01011101, 0b11011010, 0b00000010};

    if (bit_counter < 44) {
        if (psnee_readBit(bit_counter, region == 'e' ? SCEEData : region == 'a' ? SCEAData : SCEIData) == 0) {
            gpio_set_direction(PSNEE_DATA, GPIO_MODE_OUTPUT);
            gpio_set_level(PSNEE_DATA, 0); // PSNEE_DATA low
        } else {
            if (psnee.pu22mode) {
                gpio_set_direction(PSNEE_DATA, GPIO_MODE_OUTPUT);
                psnee.pu22inject = true;
            } else { // PU-18 or lower mode
                gpio_set_direction(PSNEE_DATA, GPIO_MODE_INPUT);
            }
        }
        return true;
    } else { 
        gpio_set_direction(PSNEE_DATA, GPIO_MODE_OUTPUT);
        gpio_set_level(PSNEE_DATA, 0); // pull PSNEE_DATA low
        return false;
    }
}

void psnee_injector_task(void * pvParameters) {

    enum PsNee_States state = WaitForMode;
    int loopcounter;
    uint32_t io_num;
    TickType_t xDelay = DELAY_BETWEEN_BITS / portTICK_PERIOD_MS;
    uint8_t bitcounter;
    int64_t now;

    while (true) {
        psnee.pu22inject = false;
        switch (state) {
            case WaitForMode:
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                psnee.modeDetected = true;
                psnee.pu22mode = psnee.wfck_lows > 100;
                DEBUG1(">> PsNee detected %s\n", psnee.pu22mode ? "PU-22 or newer" : "PU-20 or older");
                state = WaitForQueue;
                break;
            case WaitForQueue:
                if(xQueueReceive(psnee.gpio_evt_queue, &io_num, portMAX_DELAY)) {
                    DEBUG1(">> PsNee injection starting ...\n");
                    state = StartInject;
                    psnee.injection_running = true;
                    now = esp_timer_get_time();
                }
                break;
            case StartInject:
                gpio_set_direction(PSNEE_DATA, GPIO_MODE_OUTPUT);
                gpio_set_level(PSNEE_DATA, 0); // pull PSNEE_DATA low
                if (!psnee.pu22mode) {
                    gpio_set_direction(PSNEE_GATE_WFCK, GPIO_MODE_OUTPUT);
                    gpio_set_level(PSNEE_GATE_WFCK, 0);
                }
                xDelay = DELAY_BETWEEN_INJECTIONS / portTICK_PERIOD_MS;
                state = InjectSCEE;
                loopcounter = 0;
                bitcounter = 0;
                break;
            case InjectSCEE:
                PSNEE_INJECT('e', InjectSCEA);
                break;
            case InjectSCEA:
                PSNEE_INJECT('a', InjectSCEI);
                break;
            case InjectSCEI:
                PSNEE_INJECT('i', (++loopcounter < 2 ? InjectSCEE : EndInject));
                break;
            case EndInject:
                if (!psnee.pu22mode) {
                    gpio_set_direction(PSNEE_GATE_WFCK, GPIO_MODE_INPUT); // high-z the line, we're done
                }
                gpio_set_direction(PSNEE_DATA, GPIO_MODE_INPUT); // high-z the line, we're done
                xDelay = DELAY_BETWEEN_BITS / portTICK_PERIOD_MS;
                state = WaitForQueue;
                psnee.injection_running = false;
                DEBUG1(">> PsNee ... injection done (%lld us)!\n", (esp_timer_get_time() - now));
                break;
        }
        vTaskDelay(xDelay);
    }
}

void psnee_start() {
    gpio_pad_select_gpio(PSNEE_DATA);
    gpio_pad_select_gpio(PSNEE_GATE_WFCK);
    gpio_pad_select_gpio(PSNEE_SUBQ);
    gpio_pad_select_gpio(PSNEE_SQCK);

    gpio_set_direction(PSNEE_DATA, GPIO_MODE_INPUT);
    gpio_set_direction(PSNEE_GATE_WFCK, GPIO_MODE_INPUT);
    gpio_set_direction(PSNEE_SUBQ, GPIO_MODE_INPUT); // PSX subchannel bits
    gpio_set_direction(PSNEE_SQCK, GPIO_MODE_INPUT); // PSX subchannel clock

    psnee.gpio_evt_queue = xQueueCreate(1, sizeof(uint32_t));
    psnee.event_tx_prev = esp_timer_get_time();

    //xTaskCreatePinnedToCore(psnee_injector_task, "PsNee", 2048, NULL, 10, NULL, 0);
    xTaskCreate(psnee_injector_task, "PsNee", 2048, NULL, 10, NULL);
    gpio_set_intr_type((gpio_num_t) PSNEE_SQCK, GPIO_INTR_POSEDGE);
    gpio_set_intr_type((gpio_num_t) PSNEE_GATE_WFCK, GPIO_INTR_ANYEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add((gpio_num_t) PSNEE_SQCK, psnee_sqck_handler, (void*) PSNEE_SQCK);
    gpio_isr_handler_add((gpio_num_t) PSNEE_GATE_WFCK, psnee_wfck_handler, (void*) PSNEE_GATE_WFCK);

    DEBUG1(">> PsNee starting...\n");
}

#endif