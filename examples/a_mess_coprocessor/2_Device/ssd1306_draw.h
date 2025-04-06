#include "ssd1306_poligon.h"



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


// #include "lib_rand.h"

uint8_t myvalues[16] = { 30, 50, 60, 40, 20, 50, 30, 10, 35, 10, 20, 30, 40, 50, 60, 20 };

void modI2C_task() {
	// seed(0x12345678);
	
	int y = 0;
	
    //! hor-ver lines
	// for(int8_t i = 0; i<sizeof(myvalues); i++) {
	// 	// uint8_t rand_byte_low = rand() & 0xFF;
	// 	Limit limit = { l0: 0, l1: myvalues[i] };
	// 	// compute_horLine(y, limit, 3, 0);
	// 	// compute_horLine(y, 0, myvalues[i], 3, 1);
	// 	compute_verLine(y, limit, 3, 0);
	// 	// compute_verLine(y, 0, myvalues[i], 3, 1);
	// 	y += 4;
	// }

    //! line
    // for(uint8_t x=0;x<SSD1306_W;x+=16) {
	// 	Point point_a0 = { x: x, y: 0 };
	// 	Point point_a1 = { x: SSD1306_W, y: y };
	// 	Point point_b0 = { x: SSD1306_W-x, y: SSD1306_H };
	// 	Point point_b1 = { x: 0, y: SSD1306_H-y };
		
	// 	compute_line(point_a0, point_a1, 1);
	// 	compute_line(point_b0, point_b1, 1);

	// 	y+= SSD1306_H/8;
	// }

    //! lines
    Point points[] = {
        (Point) { 30, 30 },
        (Point) { 50, 50 },
        (Point) { 100, 20 },
        (Point) { 80, 10 },
        (Point) { 80, 50 }
    };

    // Concave polygon
    Point star[] = {
        {32,5}, {40,25}, {60,25}, 
        {45,40}, {55,60}, {32,50}, {10,60},
        {20,40}, {5,25}, {25,25}
    };

    // Convex polygon
    Point quad[] = {
        {10,10}, {50,10},
        {40,40}, {20,40}
    };

    // Self-intersecting
    Point hourglass[] = {
        {10,10}, {40,40},
        {10,40}, {40,10}
    };

    // compute_poligon(points, 4, 3);
    compute_fill_polygonOpt(points, 5);

    //! pie, circle, ring
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
