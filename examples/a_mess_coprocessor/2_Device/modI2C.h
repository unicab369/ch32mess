#include "ch32fun.h"
#include "i2c_hal.h"


#define SSD1306_128X64
#include "ssd1306_i2c.h"
#include "ssd1306.h"
#include "bomb.h"

#define INA219_ADDR 0x40

//! BH1750
#define BH17_CONT_HI1 0x10      // 1 lux resolution 120ms
#define BH17_CONT_HI2 0x11      // .5 lux resolution 120ms
#define BH17_CONT_LOW 0x13      // 4 lux resolution 16ms
#define BH17_ONCE_HI1 0x20      // 1 lux resolution 120ms
#define BH17_ONCE_HI2 0x21      // .5 lux resolution 120ms
#define BH17_ONCE_LOW 0x23      // 4 lux resolution 16ms
#define BH17_RESET 0x07
#define BH17_POWER_ON 0x01
#define BH17_ADDR 0x23

uint16_t hexToDecimal(const uint8_t* hexArray, size_t length) {
	uint16_t result = 0;
	
	for (size_t i = 0; i < length; i++) {
		result = (result << 8) | hexArray[i];
	}
	
	return result;
}

void BH17_Setup() {
	uint8_t command[1] = { BH17_CONT_LOW };
	I2CWrite(BH17_ADDR, command, sizeof(command));
}

uint16_t BH17_Read() {
	uint8_t readData[2];
	I2CRead(BH17_ADDR, readData, 2);
	return hexToDecimal(readData, sizeof(readData));
}

uint16_t INA219_Read() {
	uint8_t readData[2];
	uint16_t value16;

	uint8_t data1[] = {0x02};
	I2CWrite(INA219_ADDR, data1, 2);
	I2CRead(INA219_ADDR, readData, 2);
	
	memcpy(&value16, readData, sizeof(value16));
	uint16_t swapped = (value16>>8) | (value16<<8);
	return (swapped>>3)*4;
}

//! SHT30
uint8_t SHT_RESET_CMD[2]        = { 0x30, 0xA2 };
uint8_t SHT_HIGHREP_HOLD_CMD[2] = { 0x2C, 0x06 };
uint8_t SHT_MEDREP_HOLD_CMD[2]  = { 0x2C, 0x0D };
uint8_t SHT_LOWREP_HOLD_CMD[2]  = { 0x2C, 0x10 };
uint8_t SHT_HIGHREP_FREE_CMD[2] = { 0x24, 0x00 };
uint8_t SHT_MEDREP_FREE_CMD[2]  = { 0x24, 0x0B };
uint8_t SHT_LOWREP_FREE_CMD[2]  = { 0x24, 0x16 };
uint8_t SHT_HEATER_DISABLE[2]   = { 0x30, 0x66 };
uint8_t SHT_HEATER_ENABLE[2]    = { 0x30, 0x6D };
#define SHT3_ADDR 0x44

struct SensorData {
	uint16_t tempF, hum, lux, voltage, mA;
};

void i2c_getReadings(struct SensorData* data) {
   // uint16_t lux = 0;
	// uint16_t tempC = 0, tempF = 0, hum = 0;
	// uint16_t busVoltage = 0, mA = 0;

	//! BH1750 Reading
	int check = I2CTest(BH17_ADDR);

	if (check) { 
		data->lux = BH17_Read(); 
	}

	// char strOut[32];
	// mini_snprintf(strOut, sizeof(strOut), "\nlux=%lu", lux);
	// printf(strOut);
	// ssd1306_drawstr_sz(0, 16, strOut, 1, fontsize_8x8);

	// //! SHT31 Reading
	int check2 = I2CTest(SHT3_ADDR);

	if (check2) {
		I2CWrite(SHT3_ADDR, SHT_LOWREP_FREE_CMD, sizeof(SHT_LOWREP_FREE_CMD));
		Delay_Ms(5);
		uint8_t buff[6];
		I2CRead(SHT3_ADDR, buff, 6);

		uint16_t rawTemp = 0, rawHum = 0;
		rawTemp = (buff[0]<<8) + buff[1];
		rawHum = (buff[3]<<8) + buff[4];

		// tempC = rawTemp*175/65535 - 45;
		data->tempF = rawTemp*63/13107 - 49;
		data->hum = rawHum*100/65535;
	}

	// mini_snprintf(strOut, sizeof(strOut), "\nt=%lu, h=%lu", tempF, hum);
	// printf(strOut);

	// //! INA219 Reading
	int check3 = I2CTest(INA219_ADDR);

	if (check3) {
		data->voltage = INA219_Read();
	}

   // printf("\n\rtemp=%lu, hum=%lu, lux=%lu, v=%lu, mA=%lu", tempC, hum, lux, busVoltage, mA);
   // mini_snprintf(strOut, sizeof(strOut), "\nV=%lu, mA=%lu", busVoltage, mA);
   // printf(strOut);		

   // uint8_t allValues[] = { 0xB1, 0xCA, 0xFF, 0xFF, 0xFF, 0xFF, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x00, 0x0D, 0x0A };
   // uint16_t allValues[] = { 0xCAB1, 0xFFFF, 0xFFFF, tempF, hum, lux, busVoltage, 0x0A0D };
}




#define CHUNK_SIZE 39  // Define your desired chunk size
#define SSD1306_PAGES       (SSD1306_H / 8) // 8 pages for 64 rows
// #define SSD1306_PAGES       1 // 8 pages for 64 rows
uint8_t frame_buffer[SSD1306_PAGES][SSD1306_W] = { 0 };

typedef struct {
	uint8_t page;
	uint8_t bitmask;
} M_Page_Mask;

M_Page_Mask page_masks[SSD1306_H];

void precompute_page_masks() {
	for (uint8_t y = 0; y < SSD1306_H; y++) {
		page_masks[y].page       = y >> 3;             // (y / 8)
		page_masks[y].bitmask    = 1 << (y & 0x07);    // (y % 8)
	}
}


void ssd1306_setwindow(uint8_t start_page, uint8_t end_page, uint8_t start_column, uint8_t end_column) {
	ssd1306_cmd(SSD1306_COLUMNADDR);
	ssd1306_cmd(start_column);   				// Column start address (0 = reset)
	ssd1306_cmd(end_column); 	// Column end address (127 = reset)
	
	ssd1306_cmd(SSD1306_PAGEADDR);
	ssd1306_cmd(start_page); 	// Page start address (0 = reset)
	ssd1306_cmd(end_page); 		// Page end address
}

void ssd1306_setWindow_pages(uint8_t start_page, uint8_t end_page) {
	ssd1306_setwindow(start_page, end_page, 0, SSD1306_W - 1); // Set the window to the current page
}

void ssd1306_print_str_at(
  const char *str, uint8_t page, uint8_t column
) {
	ssd1306_setWindow_pages(page, page); // Set the window to the current page

	for (int i=0; i<32; i++) {
		if (*str) {
			uint8_t char_index = *str - 32; // Adjust for ASCII offset
			ssd1306_data((uint8_t *)FONT_7x5[char_index], 5); // Send font data
			str++;
		}
	}
}

typedef struct {
   uint8_t pos;         // X or y position of the line
   uint8_t start;      // Start position of the line
   uint8_t end;        // End position of the line
} M_Line;


void ssd1306_renderFrame(uint8_t start_page, uint8_t end_page) {
	ssd1306_setwindow(start_page, end_page, 0, SSD1306_W - 1);

    // Clear the display by sending the cleared buffer in chunks
    for (uint8_t page = 0; page < SSD1306_PAGES; page++) {
        // Send page data in chunks
        for (uint16_t chunk = 0; chunk < SSD1306_W; chunk += CHUNK_SIZE) {
            uint16_t chunk_end = (chunk + CHUNK_SIZE > SSD1306_W) ? SSD1306_W : chunk + CHUNK_SIZE;
            ssd1306_data(&frame_buffer[page][chunk], chunk_end - chunk);
        }
    }
}



//! Compute horizontal line
void compute_horLine(
	uint8_t y, uint8_t x1, uint8_t x2, 
	uint8_t thickness, uint8_t mirror
) {
    // Validate coordinates
    if (y >= SSD1306_H) return;

	// Clamp to display bounds
    if (x1 >= SSD1306_W) x1 = SSD1306_W - 1;
    if (x2 >= SSD1306_W) x2 = SSD1306_W - 1;
    
	// Handle mirroring
	if (mirror) {
		x1 = SSD1306_W - 1 - x1;
		x2 = SSD1306_W - 1 - x2;
	}

	// Ensure x1 <= x2 (swap if needed)
	if (x1 > x2) {
		uint8_t temp = x1;
		x1 = x2;
		x2 = temp;
	}

    // Handle thickness
    uint8_t y_end  = y + thickness - 1;
    if (y_end >= SSD1306_H) y_end = SSD1306_H - 1;
	if (y_end < y) return;  // Skip if thickness is 0 or overflowed

    // Draw thick line
    for (uint8_t y_pos = y; y_pos <= y_end ; y_pos++) {
        M_Page_Mask mask = page_masks[y_pos];

        for (uint8_t x_pos = x1; x_pos <= x2; x_pos++) {
            frame_buffer[mask.page][x_pos] |= mask.bitmask;
        }
    }
}


//! Compute vertical line
void compute_verLine(
	uint8_t x, uint8_t y1, uint8_t y2,
	uint8_t thickness, uint8_t mirror
) {
    // Validate coordinates
    if (x >= SSD1306_W) return;

	// Clamp to display bounds
    if (y1 >= SSD1306_H) y1 = SSD1306_H - 1;
    if (y2 >= SSD1306_H) y2 = SSD1306_H - 1;

	// Handle mirroring
	if (mirror) {
		y1 = SSD1306_H - 1 - y1;
		y2 = SSD1306_H - 1 - y2;
	}

	// Ensure y1 <= y2 (swap if needed)
	if (y1 > y2) {
		uint8_t temp = y1;
		y1 = y2;
		y2 = temp;
	}

    // Handle thickness
    uint8_t x_end = x + thickness - 1;
    if (x_end >= SSD1306_W) x_end = SSD1306_W - 1;
	if (x_end < x) return;  // Skip if thickness causes overflow

    // // Draw vertical line with thickness
    // for (uint8_t y_pos = y1; y_pos <= y2; y_pos++) {
    //     M_Page_Mask mask = page_masks[y_pos];
		
    //     for (uint8_t x_pos = x; x_pos <= x_end; x_pos++) {
    //         frame_buffer[mask.page][x_pos] |= mask.bitmask;
    //     }
    // }

	//# Optimized: save 500-700 us
	uint8_t x_len = x_end - x + 1;  // Precompute length

	for (uint8_t y_pos = y1; y_pos <= y2; y_pos++) {
		M_Page_Mask mask = page_masks[y_pos];
		uint8_t* row_start = &frame_buffer[mask.page][x];  	// Get row pointer

		for (uint8_t i = 0; i < x_len; i++) {
			row_start[i] |= mask.bitmask;  					// Sequential access
		}
	}
}

//! Compute rectangle
void compute_rectangle(
	uint8_t x, uint8_t y, uint8_t w, uint8_t h,
	uint8_t fill
) {
	// Validate coordinates
	if (x >= SSD1306_W || y >= SSD1306_H) return;

	// Clamp to display bounds
	uint16_t x_end = x + w;
	uint16_t y_end = y + h;
	if (x_end >= SSD1306_W) w = SSD1306_W - x - 1;
	if (y_end >= SSD1306_H) h = SSD1306_H - y - 1;

	// // Handle mirroring
	// if (mirror) {
	// 	x = SSD1306_W - 1 - x;
	// 	w = SSD1306_W - 1 - w;
	// }

	// Draw rectangle with optional fill
    if (fill) {
        // Filled rectangle - draw vertical lines
        for (uint8_t x_pos = x; x_pos <= x_end; x_pos++) {
            compute_verLine(x_pos, y, y_end, 1, 0);
        }
    } else {
        // Outline only
        compute_horLine(y, x, x_end, 1, 0);         		// Top edge
        compute_horLine(y_end, x, x_end, 1, 0);     		// Bottom edge
        compute_verLine(x, y + 1, y_end - 1, 1, 0); 		// Left edge
        compute_verLine(x_end, y + 1, y_end - 1, 1, 0); 	// Right edge
    }
}

//! Compute pixel
void compute_pixel(uint8_t x, uint8_t y) {
    if (x >= SSD1306_W || y >= SSD1306_H) return; // Skip if out of bounds
    M_Page_Mask mask = page_masks[y];
    frame_buffer[mask.page][x] |= mask.bitmask;
}

//! Compute_fastHorLine
void compute_fastHorLine(uint8_t x0, uint8_t x1, uint8_t y) {
    if (y >= SSD1306_H) return;
    
	// Clamp x-coordinates
	if (x0 >= SSD1306_W) x0 = SSD1306_W - 1;
	if (x1 >= SSD1306_W) x1 = SSD1306_W - 1;
	
    // Ensure x1 <= x2
    if (x0 > x1) {
        uint8_t tmp = x0;
        x0 = x1;
        x1 = tmp;
    }

    M_Page_Mask mask = page_masks[y];
    for (uint8_t x = x0; x <= x1; x++) {
        frame_buffer[mask.page][x] |= mask.bitmask;
    }
}

//! Compute circle (Bresenham's algorithm)
void compute_circle(uint8_t x0, uint8_t y0, uint8_t radius, uint8_t fill) {
	// Validate center coordinates
	if (x0 >= SSD1306_W || y0 >= SSD1306_H) return;

    int16_t x = -radius;
    int16_t y = 0;
    int16_t err = 2 - 2 * radius;

    do {
        // Calculate endpoints with clamping
        uint8_t x_start 	= x0 + x;
        uint8_t x_end   	= x0 - x;
        uint8_t y_top   	= y0 - y;
        uint8_t y_bottom 	= y0 + y;

		if (fill) {
            // Draw filled horizontal lines (top and bottom halves)
            compute_fastHorLine(x_start, x_end, y_top);     // Top half
            compute_fastHorLine(x_start, x_end, y_bottom);  // Bottom half
		} else {
			uint8_t xy_start 	= x0 + y;
			uint8_t xy_end   	= x0 - y;
			uint8_t yx_start 	= y0 + x;
			uint8_t yx_end   	= y0 - x;

			// Draw all 8 symmetric points (using precomputed page_masks)
			compute_pixel(x_end		, y_bottom); 	// Octant 1
			compute_pixel(x_start	, y_bottom); 	// Octant 2
			compute_pixel(x_start	, y_top); 		// Octant 3
			compute_pixel(x_end		, y_top); 	// Octant 4
			compute_pixel(xy_end	, yx_start); 	// Octant 5
			compute_pixel(xy_start	, yx_start); 	// Octant 6
			compute_pixel(xy_start	, yx_end); 	// Octant 7
			compute_pixel(xy_end	, yx_end); 	// Octant 8
		}

        // Update Bresenham error
        int16_t e2 = err;
        if (e2 <= y) {
            err += ++y * 2 + 1;
            if (-x == y && e2 <= x) e2 = 0;
        }
        if (e2 > x) err += ++x * 2 + 1;
    } while (x <= 0);
}

//! Compute line (Bresenham's algorithm)
void compute_line(
    uint8_t x0, uint8_t y0, 
    uint8_t x1, uint8_t y1,
    uint8_t thickness
) {
    // Clamp coordinates to display bounds
    x0 = (x0 < SSD1306_W) ? x0 : SSD1306_W - 1;
    y0 = (y0 < SSD1306_H) ? y0 : SSD1306_H - 1;
    x1 = (x1 < SSD1306_W) ? x1 : SSD1306_W - 1;
    y1 = (y1 < SSD1306_H) ? y1 : SSD1306_H - 1;

    // Bresenham's line algorithm
    int16_t dx = abs(x1 - x0);
    int16_t dy = -abs(y1 - y0);
    int16_t sx = x0 < x1 ? 1 : -1;
    int16_t sy = y0 < y1 ? 1 : -1;
    int16_t err = dx + dy;
    int16_t e2;

	thickness = 4;

	// Precompute these before the loop:
	uint8_t *fb_base = &frame_buffer[0][0];
	uint8_t radius = thickness >> 1; 		// thickness/2 via bit shift

	while (1) {
		// Draw the pixel(s)
		if (thickness == 1) {
			// Fast path for single-pixel
			if (x0 < SSD1306_W && y0 < SSD1306_H) {
                M_Page_Mask mask = page_masks[y0];
                frame_buffer[mask.page][x0] |= mask.bitmask;
			}
		} else {
			uint8_t width_index = SSD1306_W-1;
			uint8_t height_index = SSD1306_H-1;
			uint8_t x_end = x0 + radius;
			uint8_t y_end = y0 + radius;

			// Calculate bounds (branchless min/max)
			uint8_t x_start = x0 > radius ? x0 - radius : 0;
			uint8_t y_start = y0 > radius ? y0 - radius : 0;

			if (x_end > width_index) x_end = width_index;
			if (y_end > height_index) y_end = height_index;
			uint8_t width = x_end - x_start + 1;

			// Optimized row filling
			for (uint8_t y = y_start; y <= y_end; y++) {
				M_Page_Mask mask = page_masks[y];
				uint8_t *row = fb_base + (mask.page * SSD1306_W) + x_start;
				
				// Unroll small widths (1-4 pixels common)
				switch(width) {
					case 4: row[3] |= mask.bitmask; // fallthrough
					case 3: row[2] |= mask.bitmask; // fallthrough
					case 2: row[1] |= mask.bitmask; // fallthrough
					case 1: row[0] |= mask.bitmask; break;
					default: 
						for (uint8_t i = 0; i < width; i++) 
							row[i] |= mask.bitmask;
				}
			}
		}

		// Bresenham Advance
		if (x0 == x1 && y0 == y1) break;
		int16_t e2 = err << 1; // e2 = 2*err via bit shift
		if (e2 >= dy) { err += dy; x0 += sx; }
		if (e2 <= dx) { err += dx; y0 += sy; }
	}
}

// #include "lib_rand.h"

void modI2C_setup() {
	precompute_page_masks();

	// # SSD1306
	if(!ssd1306_i2c_init()) {
		ssd1306_init();
		printf("done.\n\r");
	}
	
	memset(frame_buffer, 0, sizeof(frame_buffer)); // Clear the frame buffer
	ssd1306_renderFrame(0, 7);
}


static M_Line line = {
	.pos = 0,
	.start = 0,
	.end = 30
};

uint8_t myvalues[15] = { 30, 50, 100, 40, 20, 50, 30, 10, 35, 10, 20, 30, 40, 50, 60 };

void modI2C_task() {
	// seed(0x12345678);
	
	int y = 0;
	
	// for(int8_t i = 0; i<20; i++) {
	// 	// uint8_t rand_byte_low = rand() & 0xFF;
	// 	// compute_horLine(y, 0, myvalues[i], 3, 0);
	// 	// compute_verLine(y, 0, myvalues[i], 3, 1);
	// 	y += 4;
	// }

	// compute_rectangle(0, 0, 20, 10, 1);
	// compute_rectangle(20, 25, 20, 10, 0);
	// compute_rectangle(30, 45, 20, 10, 0);
	// compute_rectangle(10, 30, 40, 30, 0);

	// for (int8_t i = 0; i<7; i++) {
	// 	uint8_t should_fill = i > 3 ? 1 : 0;
	// 	// compute_circle(i*20, 30, 5, should_fill);
	// 	// compute_rectangle(i*20, 30, 20, 10, should_fill);
	// 	y += 5;
	// }

	for(uint8_t x=0;x<SSD1306_W;x+=16) {
		compute_line(x, 0, SSD1306_W, y, 2);
		compute_line(SSD1306_W-x, SSD1306_H, 0, SSD1306_H-y, 2);
		y+= SSD1306_H/8;
	}


	ssd1306_renderFrame(0, 7);

	// ssd1306_vertical_line(&line, 2, 0);

	// line.pos = 20;
	// ssd1306_vertical_line(&line, 3, 0);

	// ssd1306_print_str_at("sz 8x8 testing", 0, 0);
	// ssd1306_print_str_at("testing 22222234fdafadfafa", 1, 0);
	// ssd1306_print_str_at("testing 33334fdafadfafa", 2, 0);
	// ssd1306_print_str_at("testing 44444fdafadfafa", 3, 0);
	// ssd1306_print_str_at("testing 55554fdafadfafa", 4, 0);
	// ssd1306_print_str_at("testing 66664fdafadfafa", 5, 0);
	// ssd1306_print_str_at("testing 77774fdafadfafa", 6, 0);
	// ssd1306_print_str_at("testing 88884fdafadfafa", 7, 0);
}

int i2c_test() {
	for(uint8_t mode=0; mode<(SSD1306_H>32?9:8); mode++) {
		// clear buffer for next mode
		ssd1306_setbuf(0);

		// switch(mode) {
		//    case 0:
		//       printf("buffer fill with binary\n\r");
		//       for(int i=0;i<sizeof(ssd1306_buffer);i++)
		//          ssd1306_buffer[i] = i;
		//       break;
			
		//    case 1:
		//       printf("pixel plots\n\r");
				// for(int i=0;i<SSD1306_W;i++)
				// {
				// 	ssd1306_drawPixel(i, i/(SSD1306_W/SSD1306_H), 1);
				// 	ssd1306_drawPixel(i, SSD1306_H-1-(i/(SSD1306_W/SSD1306_H)), 1);
				// }
		//       break;
			
		//    case 2:
		//       {
		//          printf("Line plots\n\r");
				// uint8_t y= 0;
				// for(uint8_t x=0;x<SSD1306_W;x+=16)
				// {
				// 	ssd1306_drawLine(x, 0, SSD1306_W, y, 1);
				// 	ssd1306_drawLine(SSD1306_W-x, SSD1306_H, 0, SSD1306_H-y, 1);
				// 	y+= SSD1306_H/8;
				// }
		//       }
		//       break;
				
		//    case 3:
		//       printf("Circles empty and filled\n\r");
				// for(uint8_t x=0;x<SSD1306_W;x+=16)
				// 	if(x<64)
				// 		ssd1306_drawCircle(x, SSD1306_H/2, 15, 1);
				// 	else
				// 		ssd1306_fillCircle(x, SSD1306_H/2, 15, 1);
		//       break;
		//    case 4:
		//       printf("Image\n\r");
		//       ssd1306_drawImage(0, 0, bomb_i_stripped, 32, 32, 0);
		//       break;
		//    case 5:
		//       printf("Unscaled Text\n\r");
				ssd1306_drawstr(0,0, "This is a test", 1);
				ssd1306_drawstr(0,8, "of the emergency", 1);
				ssd1306_drawstr(0,16,"broadcasting", 1);
				ssd1306_drawstr(0,24,"system.",1);
				if(SSD1306_H>32) {
					ssd1306_drawstr(0,32, "Lorem ipsum", 1);
					ssd1306_drawstr(0,40, "dolor sit amet,", 1);
					ssd1306_drawstr(0,48,"consectetur", 1);
					ssd1306_drawstr(0,56,"adipiscing",1);
				}
				ssd1306_xorrect(SSD1306_W/2, 0, SSD1306_W/2, SSD1306_W);
		//       break;
				
		//    case 6:
		//       printf("Scaled Text 1, 2\n\r");
				// ssd1306_drawstr_sz(0,0, "sz 8x8", 1, fontsize_8x8);
				// ssd1306_drawstr_sz(0,16, "16x16", 1, fontsize_16x16);
		//       break;
			
		//    case 7:
		//       printf("Scaled Text 4\n\r");
		//       ssd1306_drawstr_sz(0,0, "32x32", 1, fontsize_32x32);
		//       break;
			
		//    case 8:
		//       // printf("Scaled Text 8\n\r");
		//       ssd1306_drawstr_sz(0,0, "64", 1, fontsize_64x64);
		//       break;

		//    default:
		//       break;
		// }

		ssd1306_refresh();
	}
}
