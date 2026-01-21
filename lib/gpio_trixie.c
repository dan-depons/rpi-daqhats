/*
*   file gpio.c
*   author Measurement Computing Corp.
*   brief This file contains lightweight GPIO pin control functions.
*
*   date 17 Jan 2018
*/
#include <gpiod.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>  // For memset if needed
#include <poll.h>    // For poll(2)

#include "gpio.h"

//#define DEBUG

const char* app_name = "daqhats";

static bool gpio_initialized = false;
static struct gpiod_chip* chip;
static unsigned int num_lines;

// Per-pin requests (v2: single-line bulk requests)
#define NUM_GPIO 32
static struct gpiod_line_request* pin_requests[NUM_GPIO] = {NULL};

// Variables for GPIO interrupt threads
static int gpio_int_thread_signal[NUM_GPIO] = {0};
static void* gpio_callback_data[NUM_GPIO] = {0};
static pthread_t gpio_int_threads[NUM_GPIO];
static bool gpio_threads_running[NUM_GPIO] = {0};
static void (*gpio_callback_functions[NUM_GPIO])(void*) = {0};

void gpio_init(void)
{
    if (gpio_initialized)
    {
        return;
    }

    // Try gpiochip4 first (Pi 5), fallback to gpiochip0
    chip = gpiod_chip_open("/dev/gpiochip4");
    if (!chip)
    {
        chip = gpiod_chip_open("/dev/gpiochip0");
        if (!chip)
        {
            printf("gpio_init: could not open gpiochip\n");
            return;
        }
#ifdef DEBUG
        printf("gpio_init: found gpiochip0\n");
#endif
    }
    else
    {
#ifdef DEBUG
        printf("gpio_init: found gpiochip4\n");
#endif
    }

#ifdef DEBUG
    printf("gpio_init: chip = %p\n", chip);
#endif

    // Get number of lines
    struct gpiod_chip_info* info = gpiod_chip_get_info(chip);
    if (!info)
    {
        printf("gpio_init: failed to get chip info\n");
        gpiod_chip_close(chip);
        chip = NULL;
        return;
    }
    num_lines = gpiod_chip_info_get_num_lines(info);
    gpiod_chip_info_free(info);

#ifdef DEBUG
    printf("gpio_init: num_lines = %u\n", num_lines);
#endif

    gpio_initialized = true;
}

void gpio_close(void)
{
    if (!gpio_initialized)
    {
        return;
    }

#ifdef DEBUG
    printf("gpio_close\n");
#endif

    // Release any active requests
    for (unsigned int i = 0; i < NUM_GPIO; ++i)
    {
        if (pin_requests[i])
        {
            gpiod_line_request_release(pin_requests[i]);
            pin_requests[i] = NULL;
        }
    }

    // Stop any running threads
    for (unsigned int i = 0; i < NUM_GPIO; ++i)
    {
        if (gpio_threads_running[i])
        {
            gpio_int_thread_signal[i] = 1;
            pthread_join(gpio_int_threads[i], NULL);
            gpio_threads_running[i] = false;
        }
    }

    gpiod_chip_close(chip);
    chip = NULL;

    gpio_initialized = false;
}

static int request_pin_output(unsigned int pin, unsigned int value)
{
    if (pin >= num_lines)
    {
        printf("request_pin_output: pin %d invalid\n", pin);
        return -1;
    }

    // Release existing request
    if (pin_requests[pin])
    {
        gpiod_line_request_release(pin_requests[pin]);
        pin_requests[pin] = NULL;
    }

    // Configs for single-line output request
    struct gpiod_request_config* req_cfg = gpiod_request_config_new();
    if (!req_cfg)
    {
        perror("gpiod_request_config_new failed");
        return -1;
    }
    gpiod_request_config_set_consumer(req_cfg, app_name);

    struct gpiod_line_config* line_cfg = gpiod_line_config_new();
    if (!line_cfg)
    {
        perror("gpiod_line_config_new failed");
        gpiod_request_config_free(req_cfg);
        return -1;
    }

    struct gpiod_line_settings* settings = gpiod_line_settings_new();
    if (!settings)
    {
        perror("gpiod_line_settings_new failed");
        gpiod_line_config_free(line_cfg);
        gpiod_request_config_free(req_cfg);
        return -1;
    }

    // Set to output
    if (0 != gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT))
    {
        perror("gpiod_line_settings_set_direction failed");
        gpiod_line_settings_free(settings);
        gpiod_line_config_free(line_cfg);
        gpiod_request_config_free(req_cfg);
        return -1;
    }

    // Set initial value
    enum gpiod_line_value init_val = value ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE;
    if (0 != gpiod_line_settings_set_output_value(settings, init_val))
    {
        perror("gpiod_line_settings_set_output_value failed");
        gpiod_line_settings_free(settings);
        gpiod_line_config_free(line_cfg);
        gpiod_request_config_free(req_cfg);
        return -1;
    }

    // Add settings for the single offset (array required by API)
    unsigned int offsets[] = {pin};
    if (0 != gpiod_line_config_add_line_settings(line_cfg, offsets, 1, settings))
    {
        perror("gpiod_line_config_add_line_settings failed");
        gpiod_line_settings_free(settings);
        gpiod_line_config_free(line_cfg);
        gpiod_request_config_free(req_cfg);
        return -1;
    }
    gpiod_line_settings_free(settings);

    // Request lines (v2: no separate offsets arg)
    pin_requests[pin] = gpiod_chip_request_lines(chip, req_cfg, line_cfg);
    gpiod_line_config_free(line_cfg);
    gpiod_request_config_free(req_cfg);

    if (!pin_requests[pin])
    {
        perror("gpiod_chip_request_lines failed");
        return -1;
    }

    return 0;
}

static int request_pin_input(unsigned int pin)
{
    if (pin >= num_lines)
    {
        printf("request_pin_input: pin %d invalid\n", pin);
        return -1;
    }

    // Release existing request
    if (pin_requests[pin])
    {
        gpiod_line_request_release(pin_requests[pin]);
        pin_requests[pin] = NULL;
    }

    // Configs for single-line input request
    struct gpiod_request_config* req_cfg = gpiod_request_config_new();
    if (!req_cfg)
    {
        perror("gpiod_request_config_new failed");
        return -1;
    }
    gpiod_request_config_set_consumer(req_cfg, app_name);

    struct gpiod_line_config* line_cfg = gpiod_line_config_new();
    if (!line_cfg)
    {
        perror("gpiod_line_config_new failed");
        gpiod_request_config_free(req_cfg);
        return -1;
    }

    struct gpiod_line_settings* settings = gpiod_line_settings_new();
    if (!settings)
    {
        perror("gpiod_line_settings_new failed");
        gpiod_line_config_free(line_cfg);
        gpiod_request_config_free(req_cfg);
        return -1;
    }

    // Set to input
    if (0 != gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT))
    {
        perror("gpiod_line_settings_set_direction failed");
        gpiod_line_settings_free(settings);
        gpiod_line_config_free(line_cfg);
        gpiod_request_config_free(req_cfg);
        return -1;
    }

    // Add settings for the single offset (array required by API)
    unsigned int offsets[] = {pin};
    if (0 != gpiod_line_config_add_line_settings(line_cfg, offsets, 1, settings))
    {
        perror("gpiod_line_config_add_line_settings failed");
        gpiod_line_settings_free(settings);
        gpiod_line_config_free(line_cfg);
        gpiod_request_config_free(req_cfg);
        return -1;
    }
    gpiod_line_settings_free(settings);

    // Request lines (v2: no separate offsets arg)
    pin_requests[pin] = gpiod_chip_request_lines(chip, req_cfg, line_cfg);
    gpiod_line_config_free(line_cfg);
    gpiod_request_config_free(req_cfg);

    if (!pin_requests[pin])
    {
        perror("gpiod_chip_request_lines failed");
        return -1;
    }

    return 0;
}

static int request_pin_events(unsigned int pin, unsigned int mode)
{
    if (pin >= num_lines)
    {
        printf("request_pin_events: pin %d invalid\n", pin);
        return -1;
    }

    // Release existing request
    if (pin_requests[pin])
    {
        gpiod_line_request_release(pin_requests[pin]);
        pin_requests[pin] = NULL;
    }

    // Configs for single-line input with edges
    struct gpiod_request_config* req_cfg = gpiod_request_config_new();
    if (!req_cfg)
    {
        perror("gpiod_request_config_new failed");
        return -1;
    }
    gpiod_request_config_set_consumer(req_cfg, app_name);

    struct gpiod_line_config* line_cfg = gpiod_line_config_new();
    if (!line_cfg)
    {
        perror("gpiod_line_config_new failed");
        gpiod_request_config_free(req_cfg);
        return -1;
    }

    struct gpiod_line_settings* settings = gpiod_line_settings_new();
    if (!settings)
    {
        perror("gpiod_line_settings_new failed");
        gpiod_line_config_free(line_cfg);
        gpiod_request_config_free(req_cfg);
        return -1;
    }

    // Set to input
    if (0 != gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT))
    {
        perror("gpiod_line_settings_set_direction failed");
        gpiod_line_settings_free(settings);
        gpiod_line_config_free(line_cfg);
        gpiod_request_config_free(req_cfg);
        return -1;
    }

    // Set edge detection
    enum gpiod_line_edge edge;
    switch (mode)
    {
        case 0: edge = GPIOD_LINE_EDGE_FALLING; break;
        case 1: edge = GPIOD_LINE_EDGE_RISING; break;
        case 2: edge = GPIOD_LINE_EDGE_BOTH; break;
        default:
            gpiod_line_settings_free(settings);
            gpiod_line_config_free(line_cfg);
            gpiod_request_config_free(req_cfg);
            return -1;
    }
    if (0 != gpiod_line_settings_set_edge_detection(settings, edge))
    {
        perror("gpiod_line_settings_set_edge_detection failed");
        gpiod_line_settings_free(settings);
        gpiod_line_config_free(line_cfg);
        gpiod_request_config_free(req_cfg);
        return -1;
    }

    // Add settings for the single offset (array required by API)
    unsigned int offsets[] = {pin};
    if (0 != gpiod_line_config_add_line_settings(line_cfg, offsets, 1, settings))
    {
        perror("gpiod_line_config_add_line_settings failed");
        gpiod_line_settings_free(settings);
        gpiod_line_config_free(line_cfg);
        gpiod_request_config_free(req_cfg);
        return -1;
    }
    gpiod_line_settings_free(settings);

    // Request lines (v2: no separate offsets arg)
    pin_requests[pin] = gpiod_chip_request_lines(chip, req_cfg, line_cfg);
    gpiod_line_config_free(line_cfg);
    gpiod_request_config_free(req_cfg);

    if (!pin_requests[pin])
    {
        perror("gpiod_chip_request_lines failed");
        return -1;
    }

    return 0;
}

// Helper to clear pending events (non-blocking poll + read)
static void clear_pending_events(unsigned int pin)
{
    int fd = gpiod_line_request_get_fd(pin_requests[pin]);
    if (fd < 0) return;

    struct pollfd pfd = { .fd = fd, .events = POLLIN };
    while (poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN)) {
        struct gpiod_edge_event_buffer *buf = gpiod_edge_event_buffer_new(1);
        if (buf) {
            gpiod_line_request_read_edge_events(pin_requests[pin], buf, 1);
            gpiod_edge_event_buffer_free(buf);
        }
        pfd.revents = 0;  // Reset for next poll
    }
}

void gpio_set_output(unsigned int pin, unsigned int value)
{
#ifdef DEBUG
    printf("gpio_set_output %d %d\n", pin, value);
#endif
    if (!gpio_initialized)
    {
        gpio_init();
    }
    if (request_pin_output(pin, value) == -1)
    {
        printf("gpio_set_output: request failed\n");
    }
}

void gpio_write(unsigned int pin, unsigned int value)
{
#ifdef DEBUG
    printf("gpio_write %d %d\n", pin, value);
#endif
    if (!gpio_initialized)
    {
        gpio_init();
    }
    if (pin >= num_lines || !pin_requests[pin])
    {
        printf("gpio_write: pin %d not requested as output\n", pin);
        return;
    }

    // Set pin value (relative offset 0 for single-line request)
    if (0 != gpiod_line_request_set_value(pin_requests[pin], 0, value))
    {
        printf("gpio_write: gpiod_line_request_set_value failed\n");
    }
}

void gpio_input(unsigned int pin)
{
#ifdef DEBUG
    printf("gpio_input %d\n", pin);
#endif
    if (!gpio_initialized)
    {
        gpio_init();
    }
    if (request_pin_input(pin) == -1)
    {
        printf("gpio_input: request failed\n");
    }
}

void gpio_release(unsigned int pin)
{
#ifdef DEBUG
    printf("gpio_release %d\n", pin);
#endif
    if (!gpio_initialized)
    {
        gpio_init();
    }
    if (pin >= num_lines || !pin_requests[pin])
    {
        return;
    }

    // Release pin
    gpiod_line_request_release(pin_requests[pin]);
    pin_requests[pin] = NULL;
}

int gpio_read(unsigned int pin)
{
    if (!gpio_initialized)
    {
        gpio_init();
    }
    if (pin >= num_lines || !pin_requests[pin])
    {
        printf("gpio_read: pin %d not requested as input\n", pin);
        return -1;
    }

    // get the value (relative offset 0 for single-line request)
    int value = gpiod_line_request_get_value(pin_requests[pin], 0);
    if (value == -1)
    {
        printf("gpio_read: gpiod_line_request_get_value failed\n");
    }
    return value;
}

static void *gpio_interrupt_thread(void* arg)
{
    unsigned int pin = (unsigned int)(intptr_t)arg;
    struct gpiod_edge_event_buffer *buf = gpiod_edge_event_buffer_new(1);
    if (!buf) {
        printf("gpio_interrupt_thread: failed to create buffer\n");
        return NULL;
    }

    int fd = gpiod_line_request_get_fd(pin_requests[pin]);
    if (fd < 0) {
        gpiod_edge_event_buffer_free(buf);
        return NULL;
    }

    struct pollfd pfd = { .fd = fd, .events = POLLIN };

    while (0 == gpio_int_thread_signal[pin]) {
        int ret = poll(&pfd, 1, 1);  // 1ms timeout
        if (ret > 0 && (pfd.revents & POLLIN)) {
            int num = gpiod_line_request_read_edge_events(pin_requests[pin], buf, 1);
            if (num > 0) {
                // Discard event details, call callback
                if (gpio_callback_functions[pin]) {
                    gpio_callback_functions[pin](gpio_callback_data[pin]);
                }
            }
            pfd.revents = 0;  // Reset
        }
    }

    gpiod_edge_event_buffer_free(buf);
    return NULL;
}

int gpio_interrupt_callback(unsigned int pin, unsigned int mode, void (*function)(void*),
    void* data)
{
    if (!gpio_initialized)
    {
        gpio_init();
    }
    if (pin >= num_lines)
    {
        printf("gpio_interrupt_callback: pin %d invalid\n", pin);
        return -1;
    }

    // If disabling events (mode > 2)
    if (mode > 2)
    {
        if (pin_requests[pin])
        {
            gpiod_line_request_release(pin_requests[pin]);
            pin_requests[pin] = NULL;
        }
        if (gpio_threads_running[pin])
        {
            gpio_int_thread_signal[pin] = 1;
            pthread_join(gpio_int_threads[pin], NULL);
            gpio_threads_running[pin] = false;
        }
        gpio_callback_functions[pin] = NULL;
        return 0;
    }

    // Request with events
    if (request_pin_events(pin, mode) != 0)
    {
        printf("gpio_interrupt_callback: request events failed\n");
        return -1;
    }

    // Stop existing thread if any
    if (gpio_threads_running[pin])
    {
        gpio_int_thread_signal[pin] = 1;
        pthread_join(gpio_int_threads[pin], NULL);
        gpio_threads_running[pin] = false;
    }

    // Clear pending events
    clear_pending_events(pin);

    // Set callback
    gpio_callback_functions[pin] = function;
    gpio_callback_data[pin] = data;
    gpio_int_thread_signal[pin] = 0;

    // Start thread
    if (0 == pthread_create(&gpio_int_threads[pin], NULL, gpio_interrupt_thread, (void*)(intptr_t)pin))
    {
        gpio_threads_running[pin] = true;
    }
    else
    {
        printf("gpio_interrupt_callback: pthread_create failed\n");
        gpiod_line_request_release(pin_requests[pin]);
        pin_requests[pin] = NULL;
        return -1;
    }

    return 0;
}

int gpio_wait_for_low(unsigned int pin, unsigned int timeout_ms)
{
    if (!gpio_initialized)
    {
        gpio_init();
    }
    if (pin >= num_lines)
    {
        printf("gpio_wait_for_low: pin %d invalid\n", pin);
        return -1;
    }

    // Request input if needed
    if (request_pin_input(pin) != 0)
    {
        return -1;
    }

    // Return if already low (and release input request to match original)
    int current = gpiod_line_request_get_value(pin_requests[pin], 0);
    if (current == 0)
    {
        gpio_release(pin);
        return 1;
    }

    // Request falling edge events
    if (request_pin_events(pin, 0) != 0)  // 0 = falling
    {
        printf("gpio_wait_for_low: request falling edge failed\n");
        return -1;
    }

    // Clear pending
    clear_pending_events(pin);

    // Wait with timeout using poll
    int fd = gpiod_line_request_get_fd(pin_requests[pin]);
    if (fd < 0) {
        gpio_release(pin);
        return -1;
    }

    struct pollfd pfd = { .fd = fd, .events = POLLIN };
    int ret = poll(&pfd, 1, timeout_ms);
    int result = 0;
    if (ret > 0 && (pfd.revents & POLLIN)) {
        // Read one event to confirm
        struct gpiod_edge_event_buffer *buf = gpiod_edge_event_buffer_new(1);
        if (buf) {
            int num = gpiod_line_request_read_edge_events(pin_requests[pin], buf, 1);
            if (num > 0) {
                result = 1;
            }
            gpiod_edge_event_buffer_free(buf);
        }
    }

    // Release (matches original: leaves unrequested after wait)
    gpiod_line_request_release(pin_requests[pin]);
    pin_requests[pin] = NULL;

    return result;
}