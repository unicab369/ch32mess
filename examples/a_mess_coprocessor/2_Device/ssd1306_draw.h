#include "modSSD1306.h"


//! compute horizontal line
void compute_horLine(
	uint8_t y, Limit x_limit, uint8_t thickness, uint8_t mirror
) {
    // Validate coordinates
    if (y >= SSD1306_H) return;

	// Clamp to display bounds
    if (x_limit.l0 >= SSD1306_W) x_limit.l0 = SSD1306_W_LIMIT;
    if (x_limit.l1 >= SSD1306_W) x_limit.l1 = SSD1306_W_LIMIT;
    
	// Handle mirroring
	if (mirror) {
		x_limit.l0 = SSD1306_W_LIMIT - x_limit.l0;
		x_limit.l1 = SSD1306_W_LIMIT - x_limit.l1;
	}

	// Ensure x1 <= x2 (swap if needed)
	if (x_limit.l0 > x_limit.l1) {
		uint8_t temp = x_limit.l0;
		x_limit.l0 = x_limit.l1;
		x_limit.l1 = temp;
	}

    // Handle thickness
    uint8_t y_end  = y + thickness - 1;
    if (y_end >= SSD1306_H) y_end = SSD1306_H_LIMIT;
	if (y_end < y) return;  // Skip if thickness is 0 or overflowed

    // Draw thick line
    for (uint8_t y_pos = y; y_pos <= y_end ; y_pos++) {
        M_Page_Mask mask = page_masks[y_pos];

        for (uint8_t x_pos = x_limit.l0; x_pos <= x_limit.l1; x_pos++) {
            frame_buffer[mask.page][x_pos] |= mask.bitmask;
        }
    }
}


//! compute vertical line
void compute_verLine(
	uint8_t x, Limit y_limit, uint8_t thickness, uint8_t mirror
) {
    // Validate coordinates
    if (x >= SSD1306_W) return;

	// Clamp to display bounds
    if (y_limit.l0 >= SSD1306_H) y_limit.l0 = SSD1306_H_LIMIT;
    if (y_limit.l1 >= SSD1306_H) y_limit.l1 = SSD1306_H_LIMIT;

	// Handle mirroring
	if (mirror) {
		y_limit.l0 = SSD1306_H_LIMIT - y_limit.l0;
		y_limit.l1 = SSD1306_H_LIMIT - y_limit.l1;
	}

	// Ensure y1 <= y2 (swap if needed)
	if (y_limit.l0 > y_limit.l1) {
		uint8_t temp = y_limit.l0;
		y_limit.l0 = y_limit.l1;
		y_limit.l1 = temp;
	}

    // Handle thickness
    uint8_t x_end = x + thickness - 1;
    if (x_end >= SSD1306_W) x_end = SSD1306_W_LIMIT;
	if (x_end < x) return;  // Skip if thickness causes overflow

    // // Draw vertical line with thickness
    // for (uint8_t y_pos = y_limit.l0; y_pos <= y_limit.l1; y_pos++) {
    //     M_Page_Mask mask = page_masks[y_pos];
		
    //     for (uint8_t x_pos = x; x_pos <= x_end; x_pos++) {
    //         frame_buffer[mask.page][x_pos] |= mask.bitmask;
    //     }
    // }

	//# Optimized: save 500-700 us
	uint8_t x_len = x_end - x + 1;  // Precompute length

	for (uint8_t y_pos = y_limit.l0; y_pos <= y_limit.l1; y_pos++) {
		M_Page_Mask mask = page_masks[y_pos];
		uint8_t* row_start = &frame_buffer[mask.page][x];  	// Get row pointer

		for (uint8_t i = 0; i < x_len; i++) {
			row_start[i] |= mask.bitmask;  					// Sequential access
		}
	}
}

//! compute line (Bresenham's algorithm)
void compute_line(Point p0, Point p1, uint8_t thickness) {
    // Clamp coordinates to display bounds
    p0.x = (p0.x < SSD1306_W) ? p0.x : SSD1306_W_LIMIT;
    p0.y = (p0.y < SSD1306_H) ? p0.y : SSD1306_H_LIMIT;
    p1.x = (p1.x < SSD1306_W) ? p1.x : SSD1306_W_LIMIT;
    p1.y = (p1.y < SSD1306_H) ? p1.y : SSD1306_H_LIMIT;

    // Bresenham's line algorithm
    int16_t dx = abs(p1.x - p0.x);
    int16_t dy = -abs(p1.y - p0.y);
    int16_t sx = p0.x < p1.x ? 1 : -1;
    int16_t sy = p0.y < p1.y ? 1 : -1;
    int16_t err = dx + dy;
    int16_t e2;

	// Precompute these before the loop:
	uint8_t *fb_base = &frame_buffer[0][0];
	uint8_t radius = thickness >> 1; 		// thickness/2 via bit shift

	while (1) {
		// Draw the pixel(s)
		if (thickness == 1) {
			// Fast path for single-pixel
			if (p0.x < SSD1306_W && p0.y < SSD1306_H) {
                M_Page_Mask mask = page_masks[p0.y];
                frame_buffer[mask.page][p0.x] |= mask.bitmask;
			}
		} else {
			uint8_t x_end = p0.x + radius;
			uint8_t y_end = p0.y + radius;

			// Calculate bounds (branchless min/max)
			uint8_t x_start = p0.x > radius ? p0.x - radius : 0;
			uint8_t y_start = p0.y > radius ? p0.y - radius : 0;

			if (x_end > SSD1306_W_LIMIT) x_end = SSD1306_W_LIMIT;
			if (y_end > SSD1306_H_LIMIT) y_end = SSD1306_H_LIMIT;
			uint8_t width = x_end - x_start + 1;

			// Optimized row filling
			for (uint8_t y = y_start; y <= y_end; y++) {
				M_Page_Mask mask = page_masks[y];
				uint8_t *row = fb_base + (mask.page * SSD1306_W) + x_start;
				
				// Optimized filling based on width
				if (width <= 4) {
					// Fully unrolled for common cases
					if (width > 0) row[0] |= mask.bitmask;
					if (width > 1) row[1] |= mask.bitmask;
					if (width > 2) row[2] |= mask.bitmask;
					if (width > 3) row[3] |= mask.bitmask;
				} else {
					// For larger widths, use memset-style optimization
					uint8_t pattern = mask.bitmask;
					for (uint8_t i = 0; i < width; i++) {
						row[i] |= pattern;
					}
				}
			}
		}

		// Bresenham Advance
		if (p0.x == p1.x && p0.y == p1.y) break;
		e2 = err << 1; // e2 = 2*err via bit shift
		if (e2 >= dy) { err += dy; p0.x += sx; }
		if (e2 <= dx) { err += dx; p0.y += sy; }
	}
}


//! compute rectangle
void compute_rectangle(
	Point p0, Area area, uint8_t fill, uint8_t mirror
) {
	// Validate coordinates
	if (p0.x >= SSD1306_W || p0.y >= SSD1306_H) return;

	// Clamp to display bounds
	uint16_t x_end = p0.x + area.w;
	uint16_t y_end = p0.y + area.h;
	if (x_end >= SSD1306_W) area.w = SSD1306_W_LIMIT - p0.x;
	if (y_end >= SSD1306_H) area.h = SSD1306_H_LIMIT - p0.y;

	// Handle mirroring
	if (mirror) {
		p0.x = SSD1306_W_LIMIT - p0.x;
		area.w = SSD1306_W_LIMIT - area.w;
	}

	// Draw rectangle with optional fill
	Limit hLimit = { l0: p0.x, l1: x_end };
    if (fill) {
		// Filled rectangle using horizontal lines (faster for row-major displays)
		for (uint8_t y_pos = p0.y; y_pos <= y_end; y_pos++) {
			compute_horLine(y_pos, hLimit, 1, 0);
		}
    } else {
		Limit vLimit = { l0: p0.y + 1, l1: y_end - 1 };

        // Outline only
        compute_horLine(p0.y, hLimit, 1, 0);     	// Top edge
        compute_horLine(y_end, hLimit, 1, 0);    	// Bottom edge
        compute_verLine(p0.x, vLimit, 1, 0); 		// Left edge
        compute_verLine(x_end, vLimit, 1, 0); 		// Right edge
    }
}

//! compute pixel
void compute_pixel(uint8_t x, uint8_t y) {
    if (x >= SSD1306_W || y >= SSD1306_H) return; // Skip if out of bounds
    M_Page_Mask mask = page_masks[y];
    frame_buffer[mask.page][x] |= mask.bitmask;
}

//! compute_fastHorLine
void compute_fastHorLine(uint8_t y, uint8_t x0, uint8_t x1) {
    if (y >= SSD1306_H) return;
    
	// Clamp x-coordinates
	if (x0 >= SSD1306_W) x0 = SSD1306_W_LIMIT;
	if (x1 >= SSD1306_W) x1 = SSD1306_W_LIMIT;

    M_Page_Mask mask = page_masks[y];
    for (uint8_t x = x0; x <= x1; x++) {
        frame_buffer[mask.page][x] |= mask.bitmask;
    }
}

//! compute circle (Bresenham's algorithm)
void compute_circle(
	Point p0, uint8_t radius, uint8_t fill
) {
	// Validate center coordinates
	if (p0.x >= SSD1306_W || p0.y >= SSD1306_H) return;

    int16_t x = -radius;
    int16_t y = 0;
    int16_t err = 2 - 2 * radius;
	int16_t e2;

    do {
        // Calculate endpoints with clamping
        uint8_t x_start 	= p0.x + x;
        uint8_t x_end   	= p0.x - x;
        uint8_t y_top   	= p0.y - y;
        uint8_t y_bottom 	= p0.y + y;

		if (fill) {
            // Draw filled horizontal lines (top and bottom halves)
            compute_fastHorLine(y_top, x_start, x_end);     // Top half
            compute_fastHorLine(y_bottom, x_start, x_end);  // Bottom half
		} else {
			uint8_t xy_start 	= p0.x + y;
			uint8_t xy_end   	= p0.x - y;
			uint8_t yx_start 	= p0.y + x;
			uint8_t yx_end   	= p0.y - x;

			// Draw all 8 symmetric points (using precomputed page_masks)
			compute_pixel(x_end		, y_bottom); 	// Octant 1
			compute_pixel(x_start	, y_bottom); 	// Octant 2
			compute_pixel(x_start	, y_top); 		// Octant 3
			compute_pixel(x_end		, y_top); 		// Octant 4
			compute_pixel(xy_end	, yx_start); 	// Octant 5
			compute_pixel(xy_start	, yx_start); 	// Octant 6
			compute_pixel(xy_start	, yx_end); 		// Octant 7
			compute_pixel(xy_end	, yx_end); 		// Octant 8
		}

        // Update Bresenham error
		e2 = err;
        if (e2 <= y) {
            err += ++y * 2 + 1;
            if (-x == y && e2 <= x) e2 = 0;
        }
        if (e2 > x) err += ++x * 2 + 1;
    } while (x <= 0);
}



// Helper function to get circle point using SIN_LUT
static void get_circle_point(Point center, uint8_t radius, int16_t angle, int16_t *x, int16_t *y) {
    uint8_t quadrant = angle / 90;
    uint8_t reduced_angle = angle % 90;
    uint8_t sin_val = SIN_LUT[reduced_angle];
    uint8_t cos_val = SIN_LUT[90 - reduced_angle];
    
    switch (quadrant) {
        case 0: *x = center.x + (radius * cos_val) / 255;
                *y = center.y - (radius * sin_val) / 255;
                break;
        case 1: *x = center.x - (radius * sin_val) / 255;
                *y = center.y - (radius * cos_val) / 255;
                break;
        case 2: *x = center.x - (radius * cos_val) / 255;
                *y = center.y + (radius * sin_val) / 255;
                break;
        case 3: *x = center.x + (radius * sin_val) / 255;
                *y = center.y + (radius * cos_val) / 255;
                break;
    }
}

//! compute pie
void compute_pie(Point center, uint8_t radius, int16_t start_angle, int16_t end_angle) {    
    // Draw center point
    compute_pixel(center.x, center.y);
    
    // Special case: full circle
    if (start_angle == end_angle) {
        compute_circle(center, radius, 1); // Fill entire circle
        return;
    }
    
    // Angle stepping (adaptive based on radius)
    uint8_t step = (radius > 40) ? 2 : 1;
    
    // Draw radial lines
    int16_t angle = start_angle;
	int16_t x, y;

    while (1) {
        // Calculate edge point
        get_circle_point(center, radius, angle, &x, &y);
        
        // Draw line from center to edge
		Point p0 = { x: center.x, y: center.y };
		Point p1 = { x: x, y: y };
		compute_line(p0, p1, 1);
        
        // Break conditions
        if (angle == end_angle) break;
        
        // Move to next angle
        angle = (angle + step) % 360;
        
        // Handle wrap-around
        if (start_angle > end_angle && angle < start_angle && angle > end_angle) break;
    }
}


//! compute ring (Bresenham's algorithm)
void compute_ring(
	Point p0, uint8_t radius, uint8_t thickness,
	int16_t start_angle, int16_t end_angle
) {
    // Early exit if center is off-screen or radius is 0
    if ((p0.x >= SSD1306_W) | (p0.y >= SSD1306_H) | (radius == 0)) return;
    // Precompute display bounds and thickness
    uint8_t inner_r = (thickness >= radius) ? 1 : (radius - thickness);
    uint8_t* const frame_base = &frame_buffer[0][0];

    // Draw concentric circles (outer to inner)
    for (uint8_t r = radius; r >= inner_r; r--) {
        int16_t x = -r;
        int16_t y = 0;
        int16_t err = 2 - 2 * r;
		
        do {
            // Calculate and clamp coordinates with 1-pixel extension
            int16_t x_start = p0.x + x;
            int16_t x_end   = p0.x - x;
            int16_t y_top    = p0.y - y;
            int16_t y_bottom = p0.y + y;

			// //! Fill out 2 extra pixels besides x to make ensure no missing
			// //! pixel when drawing the circles
			int16_t x_left  = x_start - 1;
            int16_t x_right = x_end + 1;

            // Skip pixels outside valid screen bounds
            if (y_top >= 0 && y_top < SSD1306_H) {
                M_Page_Mask mask_t = page_masks[y_top];
                uint8_t* row_t = frame_base + (mask_t.page * SSD1306_W);

                if (x_left >= 0 && x_left < SSD1306_W) 		row_t[x_left]  |= mask_t.bitmask;
                if (x_start >= 0 && x_start < SSD1306_W) 	row_t[x_start] |= mask_t.bitmask;
                if (x_end >= 0 && x_end < SSD1306_W) 		row_t[x_end]   |= mask_t.bitmask;
                if (x_right >= 0 && x_right < SSD1306_W) 	row_t[x_right] |= mask_t.bitmask;
            }

            if (y_bottom >= 0 && y_bottom < SSD1306_H) {
                M_Page_Mask mask_b = page_masks[y_bottom];
                uint8_t* row_b = frame_base + (mask_b.page * SSD1306_W);

                if (x_left >= 0 && x_left < SSD1306_W) 		row_b[x_left]  |= mask_b.bitmask;
                if (x_start >= 0 && x_start < SSD1306_W) 	row_b[x_start] |= mask_b.bitmask;
                if (x_end >= 0 && x_end < SSD1306_W) 		row_b[x_end]   |= mask_b.bitmask;
                if (x_right >= 0 && x_right < SSD1306_W) 	row_b[x_right] |= mask_b.bitmask;
            }

            // Optimized Bresenham step
            int16_t e2 = err;
            if (e2 <= y) err += ++y * 2 + 1;  // y*2+1
            if (e2 > x)  err += ++x * 2 + 1;  // x*2+1
        } while (x <= 0);
    }
}

//! compute curve
void compute_curve(uint8_t* points, uint8_t num_points, uint8_t thickness, uint8_t mirror) {
    // Validate input
    if (num_points < 2 || thickness == 0) return;

    // Precompute constants
    const uint8_t* fb_base = &frame_buffer[0][0];
    const uint16_t page_stride = SSD1306_W;
    const uint8_t radius = thickness >> 1;

    // Draw curve segments
    for (uint8_t i = 0; i < num_points - 1; i++) {
        uint8_t x0 = points[i*2];
        uint8_t y0 = points[i*2+1];
        uint8_t x1 = points[(i+1)*2];
        uint8_t y1 = points[(i+1)*2+1];

        // Mirror if needed
        if (mirror) {
            x0 = SSD1306_W_LIMIT - x0;
            y0 = SSD1306_H_LIMIT - y0;
            x1 = SSD1306_W_LIMIT - x1;
            y1 = SSD1306_H_LIMIT - y1;
        }

        // Bresenham's line algorithm with thickness
        int16_t dx = abs(x1 - x0);
        int16_t dy = -abs(y1 - y0);
        int8_t sx = x0 < x1 ? 1 : -1;
        int8_t sy = y0 < y1 ? 1 : -1;
        int16_t err = dx + dy;
        int16_t e2;

        while (1) {
            // Draw thick pixel
            uint8_t x_start = x0 > radius ? x0 - radius : 0;
            uint8_t x_end = (x0 + radius) < SSD1306_W_LIMIT ? (x0 + radius) : SSD1306_W_LIMIT;
            uint8_t y_start = y0 > radius ? y0 - radius : 0;
            uint8_t y_end = (y0 + radius) < SSD1306_H_LIMIT ? (y0 + radius) : SSD1306_H_LIMIT;

            for (uint8_t y = y_start; y <= y_end; y++) {
                M_Page_Mask mask = page_masks[y];
                uint8_t* row = fb_base + (mask.page * page_stride) + x_start;
                uint8_t pixel_width = x_end - x_start + 1;

                // Optimized fill patterns
                switch (pixel_width) {
                    case 4: row[3] |= mask.bitmask; // fallthrough
                    case 3: row[2] |= mask.bitmask; // fallthrough
                    case 2: row[1] |= mask.bitmask; // fallthrough
                    case 1: row[0] |= mask.bitmask; break;
                    default:
                        for (uint8_t x = 0; x < pixel_width; x++) {
                            row[x] |= mask.bitmask;
                        }
                }
            }

            if (x0 == x1 && y0 == y1) break;
            e2 = err << 1; // e2 = 2*err via bit shift
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    }
}

// #include "lib_rand.h"

uint8_t myvalues[16] = { 30, 50, 60, 40, 20, 50, 30, 10, 35, 10, 20, 30, 40, 50, 60, 20 };

void modI2C_task() {
	// seed(0x12345678);
	
	int y = 0;
	
	// for(int8_t i = 0; i<sizeof(myvalues); i++) {
	// 	// uint8_t rand_byte_low = rand() & 0xFF;
	// 	Limit limit = { l0: 0, l1: myvalues[i] };
	// 	// compute_horLine(y, limit, 3, 0);
	// 	// compute_horLine(y, 0, myvalues[i], 3, 1);
	// 	compute_verLine(y, limit, 3, 0);
	// 	// compute_verLine(y, 0, myvalues[i], 3, 1);
	// 	y += 4;
	// }

	// Point piePoint = { x: 30, y: 30 };
	// compute_pie(piePoint, 30, 0, 125);

	// for (int8_t i = 0; i<7; i++) {
	// 	uint8_t should_fill = i > 3 ? 1 : 0;
	// 	Point point = { x: i*20, y: 30 };
	// 	// compute_circle(point, 20, should_fill);
	// 	// compute_ring(point, 20, 3, 0, 180);
	// 	compute_pie(point, 20, 0, 100);

	// 	// Point rect_point = { x: i*20, y: y };
	// 	// Area area = { w: 20, h: 10 };
	// 	// compute_rectangle(rect_point, area, should_fill, 0);
	// 	y += 5;
	// }

	for(uint8_t x=0;x<SSD1306_W;x+=16) {
		Point point_a0 = { x: x, y: 0 };
		Point point_a1 = { x: SSD1306_W, y: y };
		Point point_b0 = { x: SSD1306_W-x, y: SSD1306_H };
		Point point_b1 = { x: 0, y: SSD1306_H-y };
		
		compute_line(point_a0, point_a1, 1);
		compute_line(point_b0, point_b1, 1);

		y+= SSD1306_H/8;
	}

	uint8_t curve_points[] = {10,10, 40,30, 70,10};
	// compute_curve(curve_points, 3, 2, 0); // 2px thick, no mirror


	ssd1306_renderFrame();

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
