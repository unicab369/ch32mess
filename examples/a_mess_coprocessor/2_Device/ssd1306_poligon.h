#include "ssd1306_line.h"

//! compute poligon
void compute_poligon(Point *pts, uint8_t num_pts, uint8_t thickness) {
    if (num_pts < 3) return;  // Need at least 3 points for a polygon
    compute_lines(pts, num_pts, thickness);
    compute_line(pts[num_pts-1], pts[0], thickness);
}

//! compute filled polygon
void compute_fill_polygon(Point *pts, uint8_t num_pts) {
    if (num_pts < 3) return;

    // Find min/max Y bounds (unrolled first iteration)
    uint8_t y_min = pts[0].y, y_max = pts[0].y;
    for (uint8_t i = 1; i < num_pts; i++) {
        uint8_t py = pts[i].y;
        y_min = py < y_min ? py : y_min;
        y_max = py > y_max ? py : y_max;
    }

    // Precompute edge data
    struct Edge {
        uint8_t y0, y1;
        int16_t x, dx;
        int16_t denominator;
    } edges[num_pts];
    
    for (uint8_t i = 0; i < num_pts; i++) {
        uint8_t j = (i + 1) % num_pts;
        edges[i].y0 = pts[i].y;
        edges[i].y1 = pts[j].y;
        edges[i].dx = pts[j].x - pts[i].x;
        edges[i].denominator = pts[j].y - pts[i].y;
        edges[i].x = pts[i].x;
    }

    // Scanline processing
    for (uint8_t y = y_min; y <= y_max; y++) {
        uint8_t x_intersect[6];  // Reduced for typical convex shapes
        uint8_t intersect_count = 0;

        // Find intersections
        for (uint8_t i = 0; i < num_pts; i++) {
            if ((y >= edges[i].y0 && y < edges[i].y1) || 
                (y >= edges[i].y1 && y < edges[i].y0)) {
                // Avoid division by precomputing denominator
                int16_t x = edges[i].x;
                if (edges[i].denominator != 0) {
                    x += ((int32_t)(y - edges[i].y0) * edges[i].dx) / edges[i].denominator;
                }
                x_intersect[intersect_count++] = (uint8_t)x;
            }
        }

        // Optimized sorting for 2-4 intersections (common case)
        if (intersect_count == 2) {
            if (x_intersect[0] > x_intersect[1]) {
                uint8_t tmp = x_intersect[0];
                x_intersect[0] = x_intersect[1];
                x_intersect[1] = tmp;
            }
        } else if (intersect_count > 2) {
            // Small bubble sort (max 3 passes for 6 elements)
            for (uint8_t i = 0; i < intersect_count - 1; i++) {
                for (uint8_t j = i + 1; j < intersect_count; j++) {
                    if (x_intersect[i] > x_intersect[j]) {
                        uint8_t tmp = x_intersect[i];
                        x_intersect[i] = x_intersect[j];
                        x_intersect[j] = tmp;
                    }
                }
            }
        }

        // Fill using fast horizontal lines
        for (uint8_t i = 0; i + 1 < intersect_count; i += 2) {
            compute_fastHorLine(y, x_intersect[i], x_intersect[i+1]);
        }
    }
}

//! optimized: compute filled polygon
void compute_fill_polygonOpt(Point *pts, uint8_t num_pts) {
    // Fast reject degenerate cases
    if (num_pts < 3) return;

    // 1. Find Y bounds
    uint8_t y_min = 255, y_max = 0;
    for (uint8_t i = 0; i < num_pts; i++) {
        if (pts[i].y < y_min) y_min = pts[i].y;
        if (pts[i].y > y_max) y_max = pts[i].y;
    }

    // 2. Precompute active edges
    struct Edge {
        uint8_t y_start, y_end;
        int16_t current_x;
        int16_t dx;
        int16_t dy;
        int16_t step;
    } edges[num_pts];
    
    uint8_t active_edges = 0;
    for (uint8_t i = 0; i < num_pts; i++) {
        uint8_t j = (i + 1) % num_pts;
        uint8_t y0 = pts[i].y;
        uint8_t y1 = pts[j].y;

        // Only consider non-horizontal edges
        if (y0 != y1) {
            edges[active_edges].y_start = y0 < y1 ? y0 : y1;
            edges[active_edges].y_end = y0 < y1 ? y1 : y0;
            edges[active_edges].dy = y1 - y0;
            edges[active_edges].dx = pts[j].x - pts[i].x;
            edges[active_edges].current_x = y0 < y1 ? (pts[i].x << 8) : (pts[j].x << 8);
            edges[active_edges].step = (edges[active_edges].dx << 8) / edges[active_edges].dy;
            active_edges++;
        }
    }

    // 3. Scanline processing
    uint8_t *fb = &frame_buffer[0][0];
    for (uint8_t y = y_min; y <= y_max; y++) {
        uint8_t intersections[10]; // Supports up to 5 edges crossing
        uint8_t count = 0;

        // Find all intersections
        for (uint8_t i = 0; i < active_edges; i++) {
            if (y >= edges[i].y_start && y < edges[i].y_end) {
                intersections[count++] = edges[i].current_x >> 8;
                edges[i].current_x += edges[i].step;
            }
        }

        // Sort intersections (simple bubble sort)
        for (uint8_t i = 0; i < count; i++) {
            for (uint8_t j = i+1; j < count; j++) {
                if (intersections[i] > intersections[j]) {
                    uint8_t tmp = intersections[i];
                    intersections[i] = intersections[j];
                    intersections[j] = tmp;
                }
            }
        }

        // Fill between pairs
        M_Page_Mask mask = page_masks[y];
        uint8_t *row = fb + (mask.page * SSD1306_W);
        for (uint8_t i = 0; i < count; i += 2) {
            if (i+1 >= count) break;
            uint8_t x1 = intersections[i];
            uint8_t x2 = intersections[i+1];
            
            // Clamp to screen bounds
            if (x2 >= SSD1306_W) x2 = SSD1306_W - 1;
            if (x1 >= SSD1306_W) continue;
            
            // Fast horizontal fill
            for (uint8_t x = x1; x <= x2; x++) {
                row[x] |= mask.bitmask;
            }
        }
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
