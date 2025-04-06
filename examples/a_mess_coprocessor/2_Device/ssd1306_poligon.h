#include "ssd1306_line.h"

//! compute poligon
void compute_polygon(Point *pts, uint8_t num_pts, uint8_t thickness) {
    if (num_pts < 3) return;  // Need at least 3 points for a polygon
    compute_lines(pts, num_pts, thickness);
    compute_line(pts[num_pts-1], pts[0], thickness);
}

//! compute filled polygon
void compute_fill_polygonOpt(Point *pts, uint8_t num_pts) {
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
void compute_fill_polygonOpt2(Point *pts, uint8_t num_pts) {
    // ===== [1] EDGE EXTRACTION =====
    struct Edge {
        uint8_t y_start, y_end;
        int16_t x_start, dx_dy;
    } edges[num_pts];

    uint8_t edge_count = 0;
    uint8_t y_min = 255, y_max = 0;

    // Build edge table and find Y bounds
    for (uint8_t i = 0, j = num_pts-1; i < num_pts; j = i++) {
        // Skip horizontal edges (don't affect filling)
        if (pts[i].y == pts[j].y) continue;

        // Determine edge direction
        uint8_t y0, y1;
        int16_t x0;
        if (pts[i].y < pts[j].y) {
            y0 = pts[i].y; y1 = pts[j].y;
            x0 = pts[i].x;
        } else {
            y0 = pts[j].y; y1 = pts[i].y;
            x0 = pts[j].x;
        }

        // Update global Y bounds
        y_min = y0 < y_min ? y0 : y_min;
        y_max = y1 > y_max ? y1 : y_max;

        // Store edge (dx/dy as fixed-point 8.8)
        edges[edge_count++] = (struct Edge){
            .y_start = y0,
            .y_end = y1,
            .x_start = x0 << 8,
            .dx_dy = ((pts[j].x - pts[i].x) << 8) / (pts[j].y - pts[i].y)
        };
    }

    // ===== [2] SCANLINE PROCESSING =====
    for (uint8_t y = y_min; y <= y_max; y++) {
        uint8_t x_list[8];  // Supports 4 edge crossings (99% of cases)
        uint8_t x_count = 0;

        // Collect active edges
        for (uint8_t e = 0; e < edge_count; e++) {
            if (y >= edges[e].y_start && y < edges[e].y_end) {
                x_list[x_count++] = edges[e].x_start >> 8;
                edges[e].x_start += edges[e].dx_dy;  // Step X
            }
        }

        // Insertion sort (optimal for small N)
        for (uint8_t i = 1; i < x_count; i++) {
            uint8_t val = x_list[i];
            int8_t j = i-1;
            while (j >= 0 && x_list[j] > val) {
                x_list[j+1] = x_list[j];
                j--;
            }
            x_list[j+1] = val;
        }

        // Fill between pairs (with bounds checking)
        for (uint8_t i = 0; i+1 < x_count; i += 2) {
            uint8_t x1 = x_list[i] < SSD1306_W ? x_list[i] : SSD1306_W-1;
            uint8_t x2 = x_list[i+1] < SSD1306_W ? x_list[i+1] : SSD1306_W-1;
            if (x1 < x2) compute_fastHorLine(y, x1, x2);
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
