/**
 * Glitch-free I2S-based WS2812 driver with parallel outputs.
 *
 * Copyright (C) 2023 Martin Poelstra
 *
 * License: MIT
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"

/**
 * A single RGB pixel.
 */
typedef struct
{
    uint8_t green;
    uint8_t red;
    uint8_t blue;
} ws2812_pixel_t;

/**
 * Array of array of pixels.
 *
 * E.g.:
 * - strands[0][0] is the first pixel of the first led strip,
 * - strands[0][1] is the second pixel of the first led strip,
 * - strands[1][0] is the first pixel of the second led strip,
 * - etc.
 */
typedef ws2812_pixel_t **ws2812_strands_t;
typedef struct ws2812_bus_s *ws2812_bus_handle_t;

/**
 * Maximum number of supported strands (led strips) per bus.
 */
#define WS2812_MAX_STRANDS 8

/**
 * Configuration for ws2812_new().
 */
typedef struct
{
    /**
     * Number of strands (led strips).
     */
    int num_strands;

    /**
     * Number of pixels in each strand.
     * The first `num_strands` items need to be provided.
     *
     * Note: 'dummy' pixels may be transmitted after each strand if not all strands have the same number of pixels.
     */
    int num_pixels[WS2812_MAX_STRANDS];

    /**
     * GPIO pin numbers for data lines.
     * The first `num_strands` items need to be provided.
     */
    int gpio_pins[WS2812_MAX_STRANDS];

    /**
     * Allow this number of buffers to be late (for late interrupts).
     *
     * Can be left 0, and should probably not be more than 2 or 3.
     * Increasing this value prevents DMA buffer underruns, but
     * also uses more memory and increases the latency to start
     * transmitting a frame.
     *
     * See ws2812_stats_t for more info.
     */
    int max_late_buffers;
} ws2812_bus_config_t;

/**
 * Internal DMA buffer statistics to tune max_late_buffers.
 */
typedef struct ws2812_stats_s
{
    /**
     * Highest number of additional DMA buffers that needed
     * to be filled in one interrupt.
     *
     * If this number is higher than one, it means the interrupt
     * handler was delayed. No frame was corrupted, unless dma_underrun_errors
     * is larger than zero.
     *
     * This number will (cannot) be higher than ws2812_bus_config_t::max_late_buffers
     * (it will cause a dma_underrun_error in that case).
     */
    int max_late_buffers;

    /**
     * Number of times buffers were late.
     *
     * If this number is higher than one, it means the interrupt
     * handler was delayed. No frame was corrupted, unless dma_underrun_errors
     * is larger than zero.
     */
    int late_buffer_occurrences;

    /**
     * Number of DMA underrun errors that occurred so far.
     *
     * Every time this happens, a frame will have been partially
     * transmitted, but always in multiples of one led. So it
     * will never lead to flashes/strange colors etc, only to
     * part of the strands not being updated.
     *
     * Increase ws2812_bus_config_t::max_late_buffers to mitigate.
     */
    int dma_underrun_errors;

    /**
     * Guestimated number of additional DMA buffers needed
     * to ensure transmissions without DMA underruns.
     *
     * You can use this number for ws2812_bus_config_t::max_late_buffers.
     *
     * Take a look at the actually used max_late_buffers and dma_underrun_errors
     * to determine what the real-life needed values are.
     *
     * Note that high numbers (more than 2 or 3) probably indicate
     * an issue with other interrupts blocking for too long.
     * Move ws2812_new() to another CPU core.
     */
    int needed_late_buffers;

    /**
     * Time in microseconds between invocations of the internal
     * interrupt handler.
     * This time should normally be close to the time to transmit
     * one DMA buffer (~630us). If it is larger, more buffers will
     * be needed to compensate.
     *
     * This is a low-level statistic; you probably want to look
     * at needed_late_buffers instead.
     */
    uint32_t max_int_delta;

    /**
     * Total amount of time spent inside interrupt handler (in us).
     */
    uint32_t total_interrupt_time;

    /**
     * Number of frames that could be loaded directly
     * after each other, without waiting for DMA descriptors
     * from previous frame to drain.
     * If this number equals the frame rate, it's most
     * efficient.
     */
    uint32_t consecutive_frames;
} ws2812_stats_t;

/**
 * Allocate and initialize a new ws2812 bus with the given configuration.
 *
 * @returns ESP_OK when successful. `ret_bus` will be filled with a handle to the initialized bus.
 * @returns Any other ESP_ERR_* constant in case of an error. `ret_bus` will be unmodified in this case.
 */
esp_err_t ws2812_new(ws2812_bus_config_t *config, ws2812_bus_handle_t *ret_bus);

/**
 * Deallocate previously allocated bus.
 *
 * Waits for any pending transmissions to complete before cleaning up.
 */
void ws2812_free(ws2812_bus_handle_t bus);

/**
 * Get a new strands buffer ('frame') from the queue.
 *
 * The strand's pixels should be filled by the application and
 * then either ws2812_enqueue_strands() should be called to transmit
 * the strands, or ws2812_unget_strands() to return ('cancel') the
 * strands to the bus.
 *
 * The returned strands contains num_strands arrays of pixels, each
 * array of pixels being (at least) as long as the length given
 * during bus creation.
 *
 * A number of buffers will be returned immediately, further buffers
 * will need to wait until earlier buffers are transmitted/ungetted.
 */
ws2812_strands_t ws2812_get_strands(ws2812_bus_handle_t bus);

/**
 * Put strands buffer back to the bus without transmitting.
 *
 * This can be used if a strands buffer was obtained, but should
 * not be transmitted ('cancelled').
 */
void ws2812_unget_strands(ws2812_bus_handle_t bus, ws2812_strands_t strands);

/**
 * Enqueue strands buffer for transmission.
 *
 * The first strands buffer is transmitted as fast as possible, the
 * next will wait until the first transmission is ready.
 * This ensures that frames are transmitted as fast as possible,
 * yet aren't computed faster than what can be transmitted.
 */
void ws2812_enqueue_strands(ws2812_bus_handle_t bus, ws2812_strands_t strands);

/**
 * Get DMA buffer statistics such as late interrupt arrivals.
 * Useful to tune number of DMA buffers to ensure glitch-free delivery.
 *
 * See ws2812_stats_t for more info.
 */
void ws2812_get_stats(ws2812_bus_handle_t bus, ws2812_stats_t *ret_stats);

/**
 * Reset DMA buffer statistics back to zero.
 */
void ws2812_reset_stats(ws2812_bus_handle_t bus);
