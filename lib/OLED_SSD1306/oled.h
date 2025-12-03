#ifndef OLED_H
#define OLED_H

#include <stdbool.h>
#include <stdint.h>
#include "ssd1306.h"  // Driver header, includes macros and low-level functions

#ifdef __cplusplus
extern "C" {
#endif

/** Initialize the OLED display */
void oled_init(void);

/** Clear the display buffer and refresh */
void oled_clear(void);

/** Render the current buffer to the display */
void oled_render(void);

/** Set a pixel at (x,y) on or off */
void oled_set_pixel(int x, int y, bool on);

/** Draw a line from (x0,y0) to (x1,y1) */
void oled_draw_line(int x0, int y0, int x1, int y1, bool on);

/** Draw a single character at (x,y) */
void oled_draw_char(int x, int y, char c);

/** Draw a string at (x,y) */
void oled_draw_string(int x, int y, const char *str);
void oled_draw_big_char(int x, int y, char c);
void oled_draw_big_string(int x, int y, const char *str);

/** Set a single line of text in the internal text buffer */
/** Enum for text alignment */
typedef enum {
    OLED_ALIGN_LEFT,
    OLED_ALIGN_CENTER,
    OLED_ALIGN_RIGHT,
    OLED_ALIGN_JUSTIFY
} oled_text_alignment_t;

/** Set a single line of text in the internal text buffer */
void oled_set_text_line(uint8_t line, const char *text, oled_text_alignment_t alignment);
void oled_clear_text_line(uint8_t line);

/** Render the internal text buffer lines to the display */
void oled_render_text(void);

#ifndef font_width
#define font_width 8
#endif // font_width

// Parâmetros e macros do I2C.
#ifndef I2C_SDA
#define I2C_SDA  14
#endif // I2C_SDA

#ifndef I2C_SCL
#define I2C_SCL  15
#endif // I2C_SCL

#ifdef __cplusplus
}
#endif

#endif // OLED_H
