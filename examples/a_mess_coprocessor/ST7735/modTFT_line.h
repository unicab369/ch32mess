// stolen and adjusted from: GitHub: https://github.com/limingjie/

#include "modST7735.h"


void tft_draw_pixel(
    uint16_t x, uint16_t y, uint16_t color
) {
    x += ST7735_X_OFFSET;
    y += ST7735_Y_OFFSET;
    START_WRITE();
    tft_set_window(x, y, x, y);
    write_data_16(color);
    END_WRITE();
}

//! private
static void _draw_fast_vLine(
    int16_t x, int16_t y, int16_t h, uint16_t color
) {
    x += ST7735_X_OFFSET;
    y += ST7735_Y_OFFSET;

    uint16_t sz = 0;
    for (int16_t j = 0; j < h; j++) {
        _buffer[sz++] = color >> 8;
        _buffer[sz++] = color;
    }

    START_WRITE();
    tft_set_window(x, y, x, y + h - 1);
    DATA_MODE();
    SPI_send_DMA(_buffer, sz, 1);
    END_WRITE();
}


//! private
static void _draw_fast_hLine(
    int16_t x, int16_t y, int16_t w, uint16_t color
) {
    x += ST7735_X_OFFSET;
    y += ST7735_Y_OFFSET;

    uint16_t sz = 0;
    for (int16_t j = 0; j < w; j++) {
        _buffer[sz++] = color >> 8;
        _buffer[sz++] = color;
    }

    START_WRITE();
    tft_set_window(x, y, x + w - 1, y);
    DATA_MODE();
    SPI_send_DMA(_buffer, sz, 1);
    END_WRITE();
}

// Draw line helpers
#define _diff(a, b) ((a > b) ? (a - b) : (b - a))
#define _swap_int16(a, b)   {                           \
                                int16_t temp = a;       \
                                a            = b;       \
                                b            = temp;    \
                            }

#define MAX_PIXELS 64*2  // Optimal balance between DMA efficiency and memory usage (128 bytes)

typedef struct {
    uint16_t coords[MAX_PIXELS];  // Packed coordinates (X in upper byte, Y in lower)
    uint16_t color[MAX_PIXELS];   // Pre-filled color buffer
    uint16_t count;
    uint16_t x_min, x_max, y_min, y_max;  // Running bounds tracking
} PixelBuffer;

static PixelBuffer pb = {0};

// Inline function for faster bounds checking
static inline void _update_bounds(uint16_t x, uint16_t y) {
    if (x < pb.x_min) pb.x_min = x;
    if (x > pb.x_max) pb.x_max = x;
    if (y < pb.y_min) pb.y_min = y;
    if (y > pb.y_max) pb.y_max = y;
}

// Flush with DMA - now optimized with pre-calculated bounds
static void _flush_dma() {
    if (pb.count == 0) return;
    
    // Set optimized window
    tft_set_window(pb.x_min, pb.y_min, pb.x_max, pb.y_max);
    DATA_MODE();
    
    // DMA transfer (uses pre-filled color buffer)
    SPI_send_DMA((uint8_t*)pb.color, pb.count * 2, 1);
    
    // Reset buffer and bounds
    pb.count = 0;
    pb.x_min = 0xFFFF;
    pb.x_max = 0;
    pb.y_min = 0xFFFF;
    pb.y_max = 0;
}

// Ultra-optimized Bresenham with DMA batching
static void _draw_line_bresenham2(
    int16_t x0, int16_t y0,
    int16_t x1, int16_t y1, 
    uint16_t color
) {
    // Initial bounds setup
    pb.x_min = (x0 < x1) ? x0 : x1;
    pb.x_max = (x0 > x1) ? x0 : x1;
    pb.y_min = (y0 < y1) ? y0 : y1;
    pb.y_max = (y0 > y1) ? y0 : y1;
    
    uint8_t steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep) {
        _swap_int16(x0, y0);
        _swap_int16(x1, y1);
    }

    if (x0 > x1) {
        _swap_int16(x0, x1);
        _swap_int16(y0, y1);
    }

    int16_t dx = x1 - x0;
    int16_t dy = abs(y1 - y0);
    int16_t err = dx >> 1;
    int16_t step = (y0 < y1) ? 1 : -1;

    // Pre-fill color buffer (constant color optimization)
    for (uint16_t i = 0; i < MAX_PIXELS; i++) {
        pb.color[i] = color;
    }

    // Track previous Y in display coordinates (not swapped)
    int16_t prev_disp_y = steep ? x0 : y0;

    for (; x0 <= x1; x0++) {
        uint16_t px = steep ? y0 : x0;
        uint16_t py = steep ? x0 : y0;
        
        // Store pixel and update bounds
        pb.coords[pb.count++] = (px << 8) | py;
        _update_bounds(px, py);

        // Flush conditions:
        // 1. Buffer full
        // 2. Y changed (vertical movement)
        // 3. Approaching end of line (prevents tiny final flush)
        if (pb.count >= MAX_PIXELS || 
            py != prev_disp_y || 
            (x0 >= x1 - 1 && pb.count > 0)) {
            _flush_dma();
            prev_disp_y = py;
        }

        // Bresenham error adjustment
        err -= dy;
        if (err < 0) {
            err += dx;
            y0 += step;
        }
    }
}


//! _draw_line_bresenham
static void _draw_line_bresenham(
    int16_t x0, int16_t y0,
    int16_t x1, int16_t y1, uint16_t color
) {
    uint8_t steep = _diff(y1, y0) > _diff(x1, x0);
    if (steep) {
        _swap_int16(x0, y0);
        _swap_int16(x1, y1);
    }

    if (x0 > x1) {
        _swap_int16(x0, x1);
        _swap_int16(y0, y1);
    }

    int16_t dx   = x1 - x0;
    int16_t dy   = _diff(y1, y0);
    int16_t err  = dx >> 1;
    int16_t step = (y0 < y1) ? 1 : -1;

    for (; x0 <= x1; x0++) {
        if (steep) {
            tft_draw_pixel(y0, x0, color);
        }
        else {
            tft_draw_pixel(x0, y0, color);
        }
        err -= dy;
        if (err < 0) {
            err += dx;
            y0 += step;
        }
    }
}


void tft_draw_rect(
    uint16_t x, uint16_t y,
    uint16_t width, uint16_t height, uint16_t color
) {
    _draw_fast_hLine(x, y, width, color);
    _draw_fast_hLine(x, y + height - 1, width, color);
    _draw_fast_vLine(x, y, height, color);
    _draw_fast_vLine(x + width - 1, y, height, color);
}

void tft_draw_line(
    int16_t x0, int16_t y0,
    int16_t x1, int16_t y1, uint16_t color
) {
    if (x0 == x1) {
        if (y0 > y1) _swap_int16(y0, y1);
        _draw_fast_vLine(x0, y0, y1 - y0 + 1, color);
    }
    else if (y0 == y1) {
        if (x0 > x1) _swap_int16(x0, x1);
        _draw_fast_hLine(x0, y0, x1 - x0 + 1, color);
    }
    else {
        _draw_line_bresenham(x0, y0, x1, y1, color);
    }
}

void tft_line_tests() {
    //! dots test
    // tft_draw_pixel(rand8() % 160, rand8() % 80, colors[rand8() % 19]);

    // //! draw vertical lines
    // static uint8_t x_idx = 0;
    // tft_draw_line(x_idx, 0, x_idx, 80, colors[rand8() % 19]);
    // x_idx += 1;
    // if (x_idx >= 160) x_idx = 0;

    // //! draw horizontal lines
    // static uint8_t y_idx = 0;
    // tft_draw_line(0, y_idx, 180, y_idx, colors[rand8() % 19]);
    // y_idx += 1;
    // if (y_idx >= 80) y_idx = 0;

    //! draw random lines
    tft_draw_line(0, 0, 30, 50, RED);

    // tft_draw_line(rand8() % 160, rand8() % 80, rand8() % 160, rand8() % 80, colors[rand8() % 19]);
}