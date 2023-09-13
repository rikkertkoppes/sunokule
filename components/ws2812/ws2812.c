/**
 * Glitch-free I2S-based WS2812 driver with parallel outputs.
 *
 * Copyright (C) 2023 Martin Poelstra
 *
 * License: MIT
 */

// #define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE

#include <stdlib.h>
#include <string.h>
#include "sdkconfig.h"
#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/queue.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/portmacro.h"
#include "soc/soc_caps.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_intr_alloc.h"
#include "esp_heap_caps.h"
#include "esp_pm.h"
#include "esp_rom_gpio.h"
#include "esp_timer.h"
#include "hal/dma_types.h"
#include "hal/gpio_hal.h"
#include "driver/gpio.h"
#include "esp_clk_tree.h"
#include "esp_private/i2s_platform.h"
#include "soc/lcd_periph.h"
#include "hal/i2s_hal.h"
#include "hal/i2s_ll.h"
#include "hal/i2s_types.h"

#include "ws2812.h"

static const char *TAG = "ws2812";

#define PERIPH_CLOCK_PRE_SCALE (2) // This is the minimum divider that can be applied to LCD peripheral

// This definition is missing in current I2S LL headers
#ifndef I2S_LL_EVENT_TX_DONE
#define I2S_LL_EVENT_TX_DONE BIT(11)
#endif

typedef struct encoder_s *encoder_handle;

typedef struct frame_s
{
    int16_t remaining_pixels; // in each strand
    int16_t num_strands;
    ws2812_pixel_t *strands[WS2812_MAX_STRANDS];
} frame_t;

#if SOC_I2S_TRANS_SIZE_ALIGN_WORD
static const size_t slice_size_bytes = 2; // 2 bytes per slice (one byte ignored by I2S due to word alignment)
#else
static const size_t slice_size_bytes = 1;
#endif
static const size_t bit_size_bytes = 4 * slice_size_bytes; // 4 slices per bit
static const size_t pixel_size_bytes = 24 * bit_size_bytes;

#define WS2812_INTERRUPT_MASK (I2S_LL_EVENT_TX_EOF | I2S_LL_EVENT_TX_DONE | I2S_LL_EVENT_TX_DSCR_ERR)
#define WS2812_BIT_CLOCK_HZ (800 * 1000)
#define WS2812_BIT_SLICE_CLOCK_HZ (WS2812_BIT_CLOCK_HZ * 4)
#define WS2812_BUS_WIDTH WS2812_MAX_STRANDS

#define SPACE_TIME_US (400)                                                      /*!< Time in us to latch a frame */
#define SPACE_SLICES ((size_t)(SPACE_TIME_US * WS2812_BIT_SLICE_CLOCK_HZ / 1e6)) /*!< Equivalent number of ws2812 bit slices */
#define SPACE_BYTES (SPACE_SLICES * slice_size_bytes)                            /*!< Number of zero-bytes to output */

/**
 * Round down given maximum buffer size to the maximum number of bytes
 * we'll ever fill.
 *
 * The return value is the size of the (DMA) buffer you'll need to allocate
 * (per DMA descriptor).
 */
static size_t frame_get_buffer_size(size_t max_size, size_t max_pixels)
{
    return MIN((size_t)(max_size / pixel_size_bytes) * pixel_size_bytes, max_pixels * pixel_size_bytes);
}

static inline size_t frame_get_space_size()
{
    return SPACE_BYTES;
}

static inline int frame_get_buffer_duration(size_t bytes)
{
    return (int)((bytes / slice_size_bytes) * (1e6 / WS2812_BIT_SLICE_CLOCK_HZ));
}

static void frame_prepare_buffer(uint8_t *buffer, size_t buffer_size)
{
    size_t length = 0;
    while (length + pixel_size_bytes <= buffer_size)
    {
        for (int b = 0; b < 24; b++)
        {
#if SOC_I2S_TRANS_SIZE_ALIGN_WORD
            buffer[4] = 0x00;
            buffer[2] = 0xff;
#else
            buffer[0] = 0xff;
            buffer[3] = 0x00;
#endif
            buffer += bit_size_bytes;
            length += bit_size_bytes;
        }
    }
}

static IRAM_ATTR void frame_load_strands(frame_t *frame, ws2812_pixel_t *strands[], int num_strands, int max_num_pixels)
{
    assert(num_strands <= WS2812_MAX_STRANDS);
    frame->remaining_pixels = max_num_pixels;
    frame->num_strands = num_strands;
    // TODO Check whether it's faster to do this pointer updating, or just count the current led-index
    for (int s = 0; s < num_strands; s++)
    {
        frame->strands[s] = strands[s];
    }
}

static IRAM_ATTR size_t frame_encode(frame_t *frame, uint8_t *buffer, size_t buffer_size)
{
    size_t length = 0;

    // Fill pixels
    while (frame->remaining_pixels > 0 && length + pixel_size_bytes <= buffer_size)
    {
        uint32_t colors[frame->num_strands];
        for (int s = 0; s < frame->num_strands; s++)
        {
            // TODO This can become simpler if all pixels are stored as one big buff of 3*uint8_t
            // On the other hand, if there's a buffer underrun, it may be better to ensure we always
            // clock out a whole number of leds.
            // Still, even then easier if we can just cast the leds from struct.
            colors[s] = (uint32_t)frame->strands[s]->blue | ((uint32_t)frame->strands[s]->red << 8) | ((uint32_t)frame->strands[s]->green << 16);
            frame->strands[s]++;
        }
        frame->remaining_pixels--;

        for (int b = 23; b >= 0; b--)
        {
            uint8_t bits = 0;
            for (int s = 0; s < frame->num_strands; s++)
            {
                bits |= (colors[s] & BIT(b)) ? BIT(s) : 0;
            }

            // Set only the changing slices of each bit, rest
            // is already prepared in frame_prepare_buffer.
#if SOC_I2S_TRANS_SIZE_ALIGN_WORD
            buffer[0] = bits;
            buffer[6] = bits;
#else
            buffer[1] = bits;
            buffer[2] = bits;
#endif
            buffer += bit_size_bytes;
            length += bit_size_bytes;
        }
    }

    return length;
}

static inline bool frame_is_done(frame_t *frame)
{
    return frame->remaining_pixels == 0;
}

typedef struct encoder_node_s
{
    dma_descriptor_t desc;
    uint8_t *buffer;
} encoder_node_t;

typedef struct encoder_s
{
    frame_t frame;
    dma_descriptor_t *next_desc;
    size_t space_buffer_size;
    uint8_t *space_buffer;
    size_t space_bytes;
    size_t num_dma_nodes;
    encoder_node_t nodes[];
} encoder_t;

static void encoder_free(encoder_t *encoder);

static esp_err_t encoder_new(encoder_t **ret_encoder, size_t max_pixels, int max_late_buffers)
{
    esp_err_t ret = ESP_OK;
    encoder_t *encoder = NULL;

    // Our ring-buffer DMA needs at least 3 buffers, as the DMA controller does a bit of
    // look-ahead: it has a current and a next buffer, and a buffer that it just completed
    // (that we'll be filling up again).
    // So, if we want to allow the interrupt handler to be a bit late, we need to add more
    // buffers.
    const size_t num_dma_nodes = 3 + max_late_buffers;
    encoder = heap_caps_calloc(1, sizeof(encoder_t) + num_dma_nodes * sizeof(encoder_node_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    ESP_GOTO_ON_FALSE(encoder, ESP_ERR_NO_MEM, err, TAG, "no mem for encoder");
    encoder->num_dma_nodes = num_dma_nodes;

    encoder->space_buffer_size = MIN(DMA_DESCRIPTOR_BUFFER_MAX_SIZE, frame_get_space_size());
    ESP_LOGI(TAG, "allocating %zu bytes for latch buffer (%uus)", encoder->space_buffer_size, frame_get_buffer_duration(encoder->space_buffer_size));
    encoder->space_buffer = heap_caps_calloc(1, encoder->space_buffer_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    ESP_GOTO_ON_FALSE(encoder->space_buffer, ESP_ERR_NO_MEM, err, TAG, "no mem for encoder latch buffer");

    size_t buffer_size = frame_get_buffer_size(DMA_DESCRIPTOR_BUFFER_MAX_SIZE, max_pixels);
    ESP_LOGI(TAG, "allocating %zu DMA buffers, %zu bytes per buffer (%uus per buffer)", num_dma_nodes, buffer_size, frame_get_buffer_duration(buffer_size));

    encoder->num_dma_nodes = num_dma_nodes;
    for (int i = 0; i < encoder->num_dma_nodes; i++)
    {
        encoder_node_t *node = &encoder->nodes[i];

        ESP_LOGD(TAG, "- encoder_node descriptor %d at %p", i, &node->desc);
        node->buffer = heap_caps_calloc(1, buffer_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
        ESP_GOTO_ON_FALSE(node->buffer, ESP_ERR_NO_MEM, err, TAG, "no mem for encoder pixel buffer(s)");
        frame_prepare_buffer(node->buffer, buffer_size);

        node->desc.buffer = node->buffer;
        node->desc.dw0.size = buffer_size;
        node->desc.dw0.length = 0;
        node->desc.dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_CPU;
        node->desc.dw0.suc_eof = 0;
        node->desc.dw0.err_eof = 0;
        node->desc.dw0.reserved29 = 0;
        node->desc.next = &encoder->nodes[i + 1].desc;
    }
    encoder->nodes[encoder->num_dma_nodes - 1].desc.next = &encoder->nodes[0].desc;
    *ret_encoder = encoder;
    return ESP_OK;

err:
    if (encoder)
    {
        encoder_free(encoder);
    }
    return ret;
}

static void encoder_free(encoder_t *encoder)
{
    if (encoder->space_buffer)
    {
        free(encoder->space_buffer);
    }
    for (size_t i = 0; i < encoder->num_dma_nodes; i++)
    {
        if (encoder->nodes[i].buffer)
        {
            free(encoder->nodes[i].buffer);
        }
    }
    free(encoder);
}

static IRAM_ATTR void encoder_load_strands(encoder_t *encoder, ws2812_pixel_t *strands[], int num_strands, int max_num_pixels)
{
    // Reset all DMA descriptors for loading pixels again, resetting
    // any previous EOF and latch buffers.
    for (int i = 0; i < encoder->num_dma_nodes; i++)
    {
        encoder_node_t *node = &encoder->nodes[i];
        node->desc.buffer = node->buffer;
        node->desc.dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_CPU;
        node->desc.dw0.suc_eof = 0;
        node->desc.next = &encoder->nodes[i + 1].desc;
    }
    encoder->nodes[encoder->num_dma_nodes - 1].desc.next = &encoder->nodes[0].desc;
    encoder->next_desc = &encoder->nodes[0].desc;

    frame_load_strands(&encoder->frame, strands, num_strands, max_num_pixels);
    encoder->space_bytes = frame_get_space_size();
}

/**
 * Fill all available (i.e. CPU-owned) DMA buffers with new
 * content.
 *
 * @returns number of buffers filled.
 */
static IRAM_ATTR int encoder_update(encoder_t *encoder)
{
    int buffers_filled = 0;

    dma_descriptor_t *curr = encoder->next_desc;
    if (curr == NULL)
    {
        return buffers_filled;
    }

    while (curr->dw0.owner == DMA_DESCRIPTOR_BUFFER_OWNER_CPU && !frame_is_done(&encoder->frame))
    {
        buffers_filled++;
        size_t length = frame_encode(&encoder->frame, curr->buffer, curr->dw0.size);
        // ESP_LOGD(TAG, "--> encode %zu pixel bytes at %p", length, curr);
        curr->dw0.length = length;
        curr->dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
        curr = curr->next;
    }

    while (frame_is_done(&encoder->frame) && encoder->space_bytes > 0 && curr->dw0.owner == DMA_DESCRIPTOR_BUFFER_OWNER_CPU)
    {
        buffers_filled++;
        size_t length = MIN(encoder->space_bytes, encoder->space_buffer_size);
        curr->dw0.length = length;
        curr->dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
        curr->buffer = encoder->space_buffer;
        // ESP_LOGD(TAG, "--> encode %zu space bytes at %p", length, curr);

        encoder->space_bytes -= length;
        if (encoder->space_bytes == 0)
        {
            curr->dw0.suc_eof = 1;
            curr->next = NULL;
        }

        curr = curr->next;
    }
    encoder->next_desc = curr;
    // if (curr == NULL)
    // {
    //     ESP_LOGD(TAG, "--> encode done");
    // }
    return buffers_filled;
}

static dma_descriptor_t *encoder_get_dma_desc(encoder_t *encoder)
{
    return &encoder->nodes[0].desc;
}

static void strands_free(ws2812_strands_t strands)
{
    free(strands);
}

static ws2812_strands_t strands_alloc(int num_strands, int max_pixels)
{
    size_t strands_size = num_strands * sizeof(ws2812_strands_t);
    ws2812_strands_t strands = heap_caps_calloc(1, strands_size, MALLOC_CAP_INTERNAL);
    if (strands == NULL)
    {
        return NULL;
    }

    // Perform multiple allocs, might succeed more easily then a single contiguous chunk
    for (int s = 0; s < num_strands; s++)
    {
        strands[s] = heap_caps_calloc(max_pixels, sizeof(ws2812_pixel_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (strands[s] == NULL)
        {
            strands_free(strands);
            return NULL;
        }
    }
    return strands;
}

#define WS2812_STRIPS_MAX 8
#define WS2812_SEND_QUEUE_LENGTH 1
#define WS2812_STRANDS_QUEUE_LENGTH 2

typedef struct ws2812_bus_s
{
    int bus_id;                   // Bus ID, index from 0
    i2s_hal_context_t hal;        // Hal object
    intr_handle_t intr;           // LCD peripheral interrupt handle
    esp_pm_lock_handle_t pm_lock; // lock APB frequency when necessary

    QueueHandle_t send_queue;    // Send queue: strands in this queue will be transmitted to DMA
    QueueHandle_t strands_queue; // Available strands (for filling by app, then enqueued)

    volatile int max_late_buffers;
    volatile int dma_underrun_errors;
    /**
     * Highest measured interrupt delta time so far, in us.
     * Note: this is the 'raw' time between interrupts, including
     * the expected time it takes to clock out one DMA descriptor.
     */
    volatile uint32_t max_int_time;

    /**
     * Timestamp of last interrupt exit in us.
     * Used to compute interrupt timing.
     */
    int64_t last_int_ts;

    encoder_handle encoder;
    int num_strands;
    int max_num_pixels;
    ws2812_strands_t strands; // In-progress strands, or NULL if none
} ws2812_bus_t;

static IRAM_ATTR void ws2812_isr_handler(void *args)
{
    ws2812_bus_t *bus = (ws2812_bus_t *)args;
    encoder_t *encoder = bus->encoder;
    BaseType_t high_task_woken = pdFALSE;
    bool need_yield = false;
    uint32_t intr_status = i2s_ll_get_intr_status(bus->hal.dev);

    static int skips = 0;

    if (intr_status & I2S_LL_EVENT_TX_DSCR_ERR)
    {
        bus->dma_underrun_errors++;
        uint32_t int_time = (uint32_t)(esp_timer_get_time() - bus->last_int_ts);
        if (int_time > bus->max_int_time)
        {
            bus->max_int_time = int_time;
        }

        i2s_ll_clear_intr_status(bus->hal.dev, WS2812_INTERRUPT_MASK);

        // Force latch to allow clean restart on next frame
        // This latch generates the usual TX_EOF to close the
        // current frame.
        encoder_load_strands(encoder, bus->strands, bus->num_strands, 0);
        encoder_update(encoder);

        i2s_ll_tx_stop(bus->hal.dev);
        i2s_ll_tx_reset(bus->hal.dev); // reset TX engine first
        i2s_ll_start_out_link(bus->hal.dev);
        esp_rom_delay_us(1);
        i2s_ll_tx_start(bus->hal.dev);
    }
    if (intr_status & I2S_LL_EVENT_TX_EOF)
    {
        esp_intr_disable(bus->intr);

        if (bus->strands)
        {
            if (bus->pm_lock)
            {
                esp_pm_lock_release(bus->pm_lock);
            }
            high_task_woken = pdFALSE;
            if (xQueueSendFromISR(bus->strands_queue, &bus->strands, &high_task_woken) == pdTRUE)
            {
                if (high_task_woken == pdTRUE)
                {
                    need_yield = true;
                }
            }
        }

        bus->strands = NULL;
        high_task_woken = pdFALSE;
        if (xQueueReceiveFromISR(bus->send_queue, &bus->strands, &high_task_woken) == pdTRUE)
        {
            if (high_task_woken == pdTRUE)
            {
                need_yield = true;
            }
        }

        if (bus->strands)
        {
            i2s_ll_clear_intr_status(bus->hal.dev, WS2812_INTERRUPT_MASK);
            encoder_load_strands(encoder, bus->strands, bus->num_strands, bus->max_num_pixels);
            encoder_update(encoder);

            i2s_ll_tx_stop(bus->hal.dev);
            i2s_ll_tx_reset(bus->hal.dev); // reset TX engine first
            i2s_ll_start_out_link(bus->hal.dev);

            skips = 0;
            if (bus->pm_lock)
            {
                esp_pm_lock_acquire(bus->pm_lock);
            }
            esp_intr_enable(bus->intr);

            // Wait for I2S FIFO to be filled by DMA engine.
            // Only really needed if pixel clock is set very high, but let's be sure.
            esp_rom_delay_us(1);
            i2s_ll_tx_start(bus->hal.dev);
        }
    }
    if (intr_status & I2S_LL_EVENT_TX_DONE)
    {
        int64_t int_time = esp_timer_get_time() - bus->last_int_ts;
        if (int_time > bus->max_int_time)
        {
            bus->max_int_time = int_time;
        }
        i2s_ll_clear_intr_status(bus->hal.dev, I2S_LL_EVENT_TX_DONE);
        int buffers_filled = 0;
        if (skips > 0)
        {
            skips--;
            return;
        }
        else
        {
            buffers_filled = encoder_update(encoder);
        }
        if (buffers_filled > 1)
        {
            int late_buffers = buffers_filled - 1;
            if (late_buffers > bus->max_late_buffers)
            {
                bus->max_late_buffers = late_buffers;
            }
        }
    }
    bus->last_int_ts = esp_timer_get_time();

    if (need_yield)
    {
        portYIELD_FROM_ISR();
    }
}

esp_err_t ws2812_new(ws2812_bus_config_t *config, ws2812_bus_handle_t *ret_bus)
{
    esp_err_t ret = ESP_OK;
    ws2812_bus_t *bus = NULL;

    // Allocate bus
    bus = heap_caps_calloc(1, sizeof(ws2812_bus_t), MALLOC_CAP_INTERNAL);
    ESP_GOTO_ON_FALSE(bus, ESP_ERR_NO_MEM, err, TAG, "ws2812_new: cannot allocate bus");
    bus->bus_id = -1; // must make sure to not release bus 0 on any subsequent failure

    // Allocate encoder
    bus->num_strands = config->num_strands;
    int max_pixels = 0;
    for (int s = 0; s < config->num_strands; s++)
    {
        if (config->num_pixels[s] > max_pixels)
        {
            max_pixels = config->num_pixels[s];
        }
    }
    bus->max_num_pixels = max_pixels;
    ESP_GOTO_ON_ERROR(
        encoder_new(&bus->encoder, bus->max_num_pixels, config->max_late_buffers),
        err, TAG, "ws2812_new: cannot allocate encoder");

    // Allocate and initialize I2S peripheral
    int bus_id = -1;
    for (int i = 0; i < SOC_LCD_I80_BUSES; i++)
    {
        if (i2s_platform_acquire_occupation(i, "ws2812") == ESP_OK)
        {
            bus_id = i;
            break;
        }
    }
    ESP_GOTO_ON_FALSE(bus_id != -1, ESP_ERR_NOT_FOUND, err, TAG, "ws2812_new: no I2S LCD peripheral available");
    bus->bus_id = bus_id;
    i2s_hal_init(&bus->hal, bus->bus_id);

    // Allocate PM lock if needed
#if CONFIG_PM_ENABLE
    // create pm lock based on different clock source
    // clock sources like PLL and XTAL will be turned off in light sleep
    ESP_GOTO_ON_ERROR(
        esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "ws2812", &bus->pm_lock),
        err, TAG, "ws2812_new: create pm lock failed");
#endif

    // Set up bit slice clock
    uint32_t src_clk_hz = 0;
    ESP_GOTO_ON_ERROR(
        esp_clk_tree_src_get_freq_hz((soc_module_clk_t)I2S_CLK_SRC_PLL_160M, ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &src_clk_hz),
        err, TAG, "ws2812_new: get clock source frequency failed");

    // TODO: Don't compute final clock based on bck_div, but adjust num/den using set_mclk (non-_raw)
    // #if SOC_I2S_HW_VERSION_2
    //     i2s_ll_tx_enable_clock(hal->dev);
    //     i2s_ll_mclk_bind_to_tx_clk(hal->dev);
    // #endif
    //     i2s_ll_tx_clk_set_src(hal->dev, clk_src);
    //     i2s_ll_tx_set_mclk(hal->dev, clk_info->sclk, clk_info->mclk, clk_info->mclk_div);
    //     i2s_ll_tx_set_bck_div_num(hal->dev, clk_info->bclk_div);

    i2s_ll_tx_clk_set_src(bus->hal.dev, I2S_CLK_SRC_PLL_160M);
    // TODO Find a more appropriate bit clock using a/b
    i2s_ll_set_raw_mclk_div(bus->hal.dev, PERIPH_CLOCK_PRE_SCALE, 1, 0);

    uint32_t resolution_hz = src_clk_hz / PERIPH_CLOCK_PRE_SCALE;
    uint32_t pclk_hz_requested = WS2812_BIT_SLICE_CLOCK_HZ;
    uint32_t pclk_prescale = resolution_hz / pclk_hz_requested / 2;
    ESP_GOTO_ON_FALSE(
        pclk_prescale > 0 && pclk_prescale <= I2S_LL_BCK_MAX_PRESCALE,
        ESP_ERR_NOT_SUPPORTED, err, TAG,
        "ws2812_new: prescaler can't satisfy PCLK clock %" PRIu32 "Hz (prescaler=%lu, should be [1..%u])",
        pclk_hz_requested, pclk_prescale, (unsigned int)I2S_LL_BCK_MAX_PRESCALE);
    i2s_ll_tx_set_bck_div_num(bus->hal.dev, pclk_prescale);

    uint32_t pclk_hz_actual = resolution_hz / pclk_prescale / 2;
    // ESP_LOGI(TAG, "ws2812_new: requested slice_clk=%" PRIu32 "Hz, actual slice_clk=%" PRIu32 "Hz", pclk_hz_requested, pclk_hz_actual);
    ESP_LOGI(TAG, "ws2812_new: requested bit_clk=%" PRIu32 "Hz, actual bit_clk=%" PRIu32 "Hz", pclk_hz_requested / 4, pclk_hz_actual / 4);

    // Allocate interrupt
    // TODO Check whether we want to run at higher interrupt level
    int isr_flags = ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM;
    ret = esp_intr_alloc_intrstatus(lcd_periph_signals.buses[bus->bus_id].irq_id, isr_flags,
                                    (uint32_t)i2s_ll_get_intr_status_reg(bus->hal.dev),
                                    WS2812_INTERRUPT_MASK, ws2812_isr_handler, bus, &bus->intr);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "ws2812_new: install interrupt failed");

    // ret = esp_intr_alloc_intrstatus(lcd_periph_signals.buses[bus->bus_id].irq_id, isr_flags,
    //                                 (uint32_t)i2s_ll_get_intr_status_reg(bus->hal.dev),
    //                                 I2S_LL_EVENT_TX_DONE, handle_isr_tx_done, bus, &bus->intr2);
    // ESP_GOTO_ON_ERROR(ret, err, TAG, "install interrupt failed 2");

    i2s_ll_enable_intr(bus->hal.dev, WS2812_INTERRUPT_MASK, false); // disable interrupt temporarily
    i2s_ll_clear_intr_status(bus->hal.dev, WS2812_INTERRUPT_MASK);  // clear pending interrupt

    // Reset peripheral, DMA channel and FIFO
    i2s_ll_tx_reset(bus->hal.dev);
    i2s_ll_tx_reset_dma(bus->hal.dev);
    i2s_ll_tx_reset_fifo(bus->hal.dev);

    // initialize DMA link
    i2s_ll_dma_enable_eof_on_fifo_empty(bus->hal.dev, true);
    i2s_ll_dma_enable_owner_check(bus->hal.dev, true);
    i2s_ll_dma_enable_auto_write_back(bus->hal.dev, true);
    i2s_ll_set_out_link_addr(bus->hal.dev, (uint32_t)encoder_get_dma_desc(bus->encoder));
    i2s_ll_enable_dma(bus->hal.dev, true);

    // enable I2S LCD master mode to clock out bits in parallel
    i2s_ll_enable_lcd(bus->hal.dev, true);
    i2s_ll_tx_stop_on_fifo_empty(bus->hal.dev, true);
    i2s_ll_tx_bypass_pcm(bus->hal.dev, true);
    i2s_ll_tx_set_slave_mod(bus->hal.dev, false);
#if SOC_I2S_TRANS_SIZE_ALIGN_WORD
    // switch to I2S 16bits mode, two WS cycle <=> one I2S FIFO
    // TODO: See if this is really necessary
    i2s_ll_tx_set_bits_mod(bus->hal.dev, 16);
#else
    i2s_ll_tx_set_bits_mod(bus->hal.dev, WS2812_BUS_WIDTH);
#endif
    i2s_ll_tx_select_std_slot(bus->hal.dev, I2S_STD_SLOT_BOTH, true); // Sets conf_chan.tx_chan_mod
    i2s_ll_tx_enable_right_first(bus->hal.dev, true);                 // TODO Does this matter?
#if SOC_I2S_SUPPORTS_DMA_EQUAL
    i2s_ll_tx_enable_dma_equal(bus->hal.dev, true);
#endif
    // enable the necessary interrupt bits (but keep the interrupt handler still disabled)
    i2s_ll_enable_intr(bus->hal.dev, WS2812_INTERRUPT_MASK, true);

    // Trigger dummy transaction to get the interrupt to be triggered, such that
    // when we later enable the interrupt, it will immediately enter the handler
    // and that can start the actual transfer.
    dma_descriptor_t dummy_desc = {
        .buffer = &bus->encoder->space_bytes, // ugly, but we need a few zero bytes in DMA RAM
        .dw0.size = 4,
        .dw0.length = 4,
        .dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA,
        .dw0.suc_eof = 1,
        .next = NULL,
    };
    i2s_ll_set_out_link_addr(bus->hal.dev, (uint32_t)&dummy_desc);
    i2s_ll_start_out_link(bus->hal.dev);
    // Wait for I2S FIFO to be filled by DMA engine.
    // Only really needed if pixel clock is set very high, but let's be sure.
    esp_rom_delay_us(1);
    i2s_ll_tx_start(bus->hal.dev);
    while (!(i2s_ll_get_intr_status(bus->hal.dev) & I2S_LL_EVENT_TX_EOF))
    {
    }
    // Clear all interrupt flags, except TX_EOF
    i2s_ll_clear_intr_status(bus->hal.dev, WS2812_INTERRUPT_MASK & ~I2S_LL_EVENT_TX_EOF);

    // Install the 'real' DMA chain
    i2s_ll_set_out_link_addr(bus->hal.dev, (uint32_t)encoder_get_dma_desc(bus->encoder));

    // Configure GPIO pins
    for (int i = 0; i < config->num_strands; i++)
    {
        if (config->gpio_pins[i] < 0)
        {
            return ESP_ERR_INVALID_ARG;
        }
    }

    for (int s = 0; s < config->num_strands; s++)
    {
        int pin = config->gpio_pins[s];
        ESP_GOTO_ON_ERROR(
            gpio_set_direction(pin, GPIO_MODE_OUTPUT),
            err, TAG, "ws2812_new: failed to set GPIO direction for data pin %d (GPIO %d)", s, pin);
#if SOC_I2S_TRANS_SIZE_ALIGN_WORD
        esp_rom_gpio_connect_out_signal(pin, lcd_periph_signals.buses[bus_id].data_sigs[s + 8], false, false);
#else
        esp_rom_gpio_connect_out_signal(pin, lcd_periph_signals.buses[bus_id].data_sigs[s + SOC_LCD_I80_BUS_WIDTH - bus_config->bus_width], false, false);
#endif
        gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[pin], PIN_FUNC_GPIO);
    }

    bus->send_queue = xQueueCreate(WS2812_SEND_QUEUE_LENGTH, sizeof(ws2812_strands_t));
    ESP_GOTO_ON_FALSE(bus->send_queue, ESP_ERR_NO_MEM, err, TAG, "ws2812_new: create send queue failed");

    bus->strands_queue = xQueueCreate(WS2812_STRANDS_QUEUE_LENGTH, sizeof(ws2812_strands_t));
    ESP_GOTO_ON_FALSE(bus->strands_queue, ESP_ERR_NO_MEM, err, TAG, "ws2812_new: create strands queue failed");

    for (int f = 0; f < WS2812_STRANDS_QUEUE_LENGTH; f++)
    {
        ws2812_strands_t strands = strands_alloc(bus->num_strands, bus->max_num_pixels);
        ESP_GOTO_ON_FALSE(strands, ESP_ERR_NO_MEM, err, TAG, "ws2812_new: cannot allocate memory for strands frame %d", f);
        xQueueSend(bus->strands_queue, &strands, 0);
    }

    ESP_LOGI(TAG, "ws2812_new: bus allocated at I2S%d", bus->bus_id);
    *ret_bus = bus;

    return ESP_OK;

err:
    if (bus)
    {
        ws2812_free(bus);
    }
    return ret;
}

void ws2812_free(ws2812_bus_handle_t bus)
{
    if (bus->strands_queue)
    {
        // Wait for each of the strands to become available
        // and consume them (which means we'll drain all
        // transmissions).
        for (int f = 0; f < WS2812_STRANDS_QUEUE_LENGTH; f++)
        {
            strands_free(ws2812_get_strands(bus));
        }
        vQueueDelete(bus->strands_queue);
    }
    if (bus->send_queue)
    {
        vQueueDelete(bus->send_queue);
    }
    if (bus->intr)
    {
        esp_intr_free(bus->intr);
    }
    if (bus->bus_id >= 0)
    {
        i2s_platform_release_occupation(bus->bus_id);
    }
    if (bus->pm_lock)
    {
        esp_pm_lock_delete(bus->pm_lock);
    }
    if (bus->encoder)
    {
        encoder_free(bus->encoder);
    }
    free(bus);
}

ws2812_strands_t ws2812_get_strands(ws2812_bus_handle_t bus)
{
    ws2812_strands_t strands;
    assert(xQueueReceive(bus->strands_queue, &strands, portMAX_DELAY) == pdTRUE);
    return strands;
}

void ws2812_unget_strands(ws2812_bus_handle_t bus, ws2812_strands_t strands)
{
    assert(strands);
    assert(strands != bus->strands);
    assert(xQueueSendToFront(bus->strands_queue, &strands, 0) == pdTRUE);
}

void ws2812_enqueue_strands(ws2812_bus_handle_t bus, ws2812_strands_t strands)
{
    // Some very basic sanity checks. Could be made smarter to properly
    // check whether these strands are not already enqueued.
    assert(strands);
    assert(strands != bus->strands);
    assert(xQueueSend(bus->send_queue, &strands, portMAX_DELAY) == pdTRUE);
    esp_intr_enable(bus->intr);
}

void ws2812_get_stats(ws2812_bus_handle_t bus, ws2812_stats_t *ret_stats)
{
    ret_stats->max_int_time = bus->max_int_time;
    ret_stats->max_late_buffers = bus->max_late_buffers;
    ret_stats->dma_underrun_errors = bus->dma_underrun_errors;
    size_t max_buffer_size = encoder_get_dma_desc(bus->encoder)->dw0.size;
    int buffer_duration = frame_get_buffer_duration(max_buffer_size);
    int computed_late_buffers = (bus->max_int_time - buffer_duration + buffer_duration - 1) / buffer_duration;
    ret_stats->needed_late_buffers = MAX(bus->max_late_buffers, computed_late_buffers);
}

void ws2812_reset_stats(ws2812_bus_handle_t bus)
{
    bus->max_int_time = 0;
    bus->max_late_buffers = 0;
    bus->dma_underrun_errors = 0;
}
