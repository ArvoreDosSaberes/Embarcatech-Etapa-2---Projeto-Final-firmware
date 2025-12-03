#include "pico.h"
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#ifdef I2C_USE_FREERTOS
#include "I2C_freeRTOS.hpp"
#endif
#include "oled.h"
#include "ssd1306.h"
#include <string.h>
#include "big_font.h"

// Internal buffer and render area
static uint8_t ssd_buffer[ssd1306_buffer_length];
static struct render_area area = {
    .start_column = 0,
    .end_column = ssd1306_width - 1,
    .start_page = 0,
    .end_page = ssd1306_n_pages - 1,
    .buffer_length = 0
};

/* Internal text buffer for facade text lines */
typedef struct {
    char text[max_text_columns];
    oled_text_alignment_t alignment;
} text_line_t;

static text_line_t text_buffer[max_text_lines];

void oled_init(void) {
    // Initialize I2C pins and bus is done by ssd1306_i2c.h includes
    i2c_init(i2c1, ssd1306_i2c_clock * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    #ifdef I2C_USE_FREERTOS
    initI2CSemaphore(1);
    #endif
    
    calculate_render_area_buffer_length(&area);
    ssd1306_init();
    memset(ssd_buffer, 0, ssd1306_buffer_length);
    render_on_display(ssd_buffer, &area);
}

void oled_clear(void) {
    memset(ssd_buffer, 0, ssd1306_buffer_length);
    #ifdef I2C_USE_FREERTOS
    takeI2C(1, portMAX_DELAY);
    #endif
    render_on_display(ssd_buffer, &area);
    #ifdef I2C_USE_FREERTOS
    releaseI2C(1);
    #endif
}

void oled_render(void) {
    #ifdef I2C_USE_FREERTOS
    takeI2C(1, portMAX_DELAY);
    #endif
    render_on_display(ssd_buffer, &area);
    #ifdef I2C_USE_FREERTOS
    releaseI2C(1);
    #endif
}

void oled_set_pixel(int x, int y, bool on) {
    ssd1306_set_pixel(ssd_buffer, x, y, on);
}

void oled_draw_line(int x0, int y0, int x1, int y1, bool on) {
    ssd1306_draw_line(ssd_buffer, x0, y0, x1, y1, on);
}

void oled_draw_char(int x, int y, char c) {
    ssd1306_draw_char(ssd_buffer, x, y, (uint8_t)c);
}

void oled_draw_string(int x, int y, const char *str) {
    ssd1306_draw_string(ssd_buffer, x, y, (char *)str);
}

void oled_draw_big_char(int x, int y, char c) {
    ssd1306_draw_big_char(ssd_buffer, x, y, (uint8_t)c);
}

/**
 * Set a single line of text in the internal text buffer
 */
void oled_set_text_line(uint8_t line, const char *text, oled_text_alignment_t alignment) {
    if (line < max_text_lines) {
        strncpy(text_buffer[line].text, text, max_text_columns - 2);
        text_buffer[line].text[max_text_columns - 1] = '\0';
        text_buffer[line].alignment = alignment;
    }
}

/**
 * Clear a single line of text in the internal text buffer
 */
void oled_clear_text_line(uint8_t line) {
    oled_set_text_line(line, "", OLED_ALIGN_LEFT);
}

/**
 * Render the internal text buffer lines to the display
 */
void oled_render_text(void) {
    oled_clear();
    for (uint8_t line = 0; line < max_text_lines; line++) {
        int x = 0;
        int text_len = strlen(text_buffer[line].text);
        int text_width = text_len * font_width;

        switch (text_buffer[line].alignment) {
            case OLED_ALIGN_LEFT:
                x = 0;
                break;
            case OLED_ALIGN_CENTER:
                x = (ssd1306_width - text_width) / 2;
                break;
            case OLED_ALIGN_RIGHT:
                x = ssd1306_width - text_width;
                break;
            case OLED_ALIGN_JUSTIFY: // Justify is complex, for now, we'll treat it as left-aligned.
                x = 0;
                // A proper implementation would calculate spacing between words.
                break;
        }

        if (x < 0) x = 0; // Ensure text is not drawn off-screen to the left

        oled_draw_string(x, line * ssd1306_line_height, text_buffer[line].text);
    }
    oled_render();
}
