/**
 * ESP32-S3  |  ESP-IDF  |  esp-who
 *
 * Hand position classification (left/right/up/down/close/far) with LCD overlay.
 * Periodic system restart prevents slowdown from memory fragmentation.
 */

#include "frame_cap_pipeline.hpp"
#include "who_detect_app_lcd.hpp"
#include "bsp/esp-bsp.h"
#include "hand_detect.hpp"

#include <cmath>
#include <string>
#include <cstring>
#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_system.h"

using namespace who::frame_cap;
using namespace who::app;
using namespace who::detect;

static const char *TAG = "HAND_POSITION";

// -----------------------------------------------------------------------------
// Bitmap font (6x8, ASCII 32-126) – unchanged from original
// -----------------------------------------------------------------------------

static const uint8_t FONT6x8[][8] = {
    // ... (keep your full font array exactly as in your original code)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // 0x20 ' '
    {0x20,0x20,0x20,0x20,0x20,0x00,0x20,0x00}, // 0x21 '!'
    {0x50,0x50,0x00,0x00,0x00,0x00,0x00,0x00}, // 0x22 '"'
    {0x50,0x50,0xF8,0x50,0xF8,0x50,0x50,0x00}, // 0x23 '#'
    {0x20,0x78,0xA0,0x70,0x28,0xF0,0x20,0x00}, // 0x24 '$'
    {0xC0,0xC8,0x10,0x20,0x40,0x98,0x18,0x00}, // 0x25 '%'
    {0x60,0x90,0xA0,0x40,0xA8,0x90,0x68,0x00}, // 0x26 '&'
    {0x20,0x20,0x00,0x00,0x00,0x00,0x00,0x00}, // 0x27 '\''
    {0x10,0x20,0x40,0x40,0x40,0x20,0x10,0x00}, // 0x28 '('
    {0x40,0x20,0x10,0x10,0x10,0x20,0x40,0x00}, // 0x29 ')'
    {0x00,0x20,0xA8,0x70,0xA8,0x20,0x00,0x00}, // 0x2A '*'
    {0x00,0x20,0x20,0xF8,0x20,0x20,0x00,0x00}, // 0x2B '+'
    {0x00,0x00,0x00,0x00,0x00,0x20,0x40,0x00}, // 0x2C ','
    {0x00,0x00,0x00,0xF8,0x00,0x00,0x00,0x00}, // 0x2D '-'
    {0x00,0x00,0x00,0x00,0x00,0x00,0x20,0x00}, // 0x2E '.'
    {0x08,0x10,0x10,0x20,0x40,0x40,0x80,0x00}, // 0x2F '/'
    {0x70,0x88,0x98,0xA8,0xC8,0x88,0x70,0x00}, // 0x30 '0'
    {0x20,0x60,0x20,0x20,0x20,0x20,0x70,0x00}, // 0x31 '1'
    {0x70,0x88,0x08,0x30,0x40,0x80,0xF8,0x00}, // 0x32 '2'
    {0xF8,0x08,0x10,0x30,0x08,0x88,0x70,0x00}, // 0x33 '3'
    {0x10,0x30,0x50,0x90,0xF8,0x10,0x10,0x00}, // 0x34 '4'
    {0xF8,0x80,0xF0,0x08,0x08,0x88,0x70,0x00}, // 0x35 '5'
    {0x38,0x40,0x80,0xF0,0x88,0x88,0x70,0x00}, // 0x36 '6'
    {0xF8,0x08,0x10,0x20,0x20,0x20,0x20,0x00}, // 0x37 '7'
    {0x70,0x88,0x88,0x70,0x88,0x88,0x70,0x00}, // 0x38 '8'
    {0x70,0x88,0x88,0x78,0x08,0x10,0x60,0x00}, // 0x39 '9'
    {0x00,0x20,0x00,0x00,0x00,0x20,0x00,0x00}, // 0x3A ':'
    {0x00,0x20,0x00,0x00,0x00,0x20,0x40,0x00}, // 0x3B ';'
    {0x08,0x10,0x20,0x40,0x20,0x10,0x08,0x00}, // 0x3C '<'
    {0x00,0x00,0xF8,0x00,0xF8,0x00,0x00,0x00}, // 0x3D '='
    {0x40,0x20,0x10,0x08,0x10,0x20,0x40,0x00}, // 0x3E '>'
    {0x70,0x88,0x08,0x30,0x20,0x00,0x20,0x00}, // 0x3F '?'
    {0x70,0x88,0xA8,0xB8,0xA0,0x80,0x78,0x00}, // 0x40 '@'
    {0x20,0x50,0x88,0x88,0xF8,0x88,0x88,0x00}, // 0x41 'A'
    {0xF0,0x88,0x88,0xF0,0x88,0x88,0xF0,0x00}, // 0x42 'B'
    {0x70,0x88,0x80,0x80,0x80,0x88,0x70,0x00}, // 0x43 'C'
    {0xE0,0x90,0x88,0x88,0x88,0x90,0xE0,0x00}, // 0x44 'D'
    {0xF8,0x80,0x80,0xF0,0x80,0x80,0xF8,0x00}, // 0x45 'E'
    {0xF8,0x80,0x80,0xF0,0x80,0x80,0x80,0x00}, // 0x46 'F'
    {0x70,0x88,0x80,0xB8,0x88,0x88,0x70,0x00}, // 0x47 'G'
    {0x88,0x88,0x88,0xF8,0x88,0x88,0x88,0x00}, // 0x48 'H'
    {0x70,0x20,0x20,0x20,0x20,0x20,0x70,0x00}, // 0x49 'I'
    {0x38,0x10,0x10,0x10,0x10,0x90,0x60,0x00}, // 0x4A 'J'
    {0x88,0x90,0xA0,0xC0,0xA0,0x90,0x88,0x00}, // 0x4B 'K'
    {0x80,0x80,0x80,0x80,0x80,0x80,0xF8,0x00}, // 0x4C 'L'
    {0x88,0xD8,0xA8,0xA8,0x88,0x88,0x88,0x00}, // 0x4D 'M'
    {0x88,0xC8,0xA8,0x98,0x88,0x88,0x88,0x00}, // 0x4E 'N'
    {0x70,0x88,0x88,0x88,0x88,0x88,0x70,0x00}, // 0x4F 'O'
    {0xF0,0x88,0x88,0xF0,0x80,0x80,0x80,0x00}, // 0x50 'P'
    {0x70,0x88,0x88,0x88,0xA8,0x90,0x68,0x00}, // 0x51 'Q'
    {0xF0,0x88,0x88,0xF0,0xA0,0x90,0x88,0x00}, // 0x52 'R'
    {0x70,0x88,0x80,0x70,0x08,0x88,0x70,0x00}, // 0x53 'S'
    {0xF8,0x20,0x20,0x20,0x20,0x20,0x20,0x00}, // 0x54 'T'
    {0x88,0x88,0x88,0x88,0x88,0x88,0x70,0x00}, // 0x55 'U'
    {0x88,0x88,0x88,0x88,0x88,0x50,0x20,0x00}, // 0x56 'V'
    {0x88,0x88,0x88,0xA8,0xA8,0xD8,0x88,0x00}, // 0x57 'W'
    {0x88,0x88,0x50,0x20,0x50,0x88,0x88,0x00}, // 0x58 'X'
    {0x88,0x88,0x50,0x20,0x20,0x20,0x20,0x00}, // 0x59 'Y'
    {0xF8,0x08,0x10,0x20,0x40,0x80,0xF8,0x00}, // 0x5A 'Z'
    {0x70,0x40,0x40,0x40,0x40,0x40,0x70,0x00}, // 0x5B '['
    {0x80,0x40,0x40,0x20,0x10,0x10,0x08,0x00}, // 0x5C '\'
    {0x70,0x10,0x10,0x10,0x10,0x10,0x70,0x00}, // 0x5D ']'
    {0x20,0x50,0x88,0x00,0x00,0x00,0x00,0x00}, // 0x5E '^'
    {0x00,0x00,0x00,0x00,0x00,0x00,0xF8,0x00}, // 0x5F '_'
    {0x40,0x20,0x00,0x00,0x00,0x00,0x00,0x00}, // 0x60 '`'
    {0x00,0x00,0x70,0x08,0x78,0x88,0x78,0x00}, // 0x61 'a'
    {0x80,0x80,0xB0,0xC8,0x88,0xC8,0xB0,0x00}, // 0x62 'b'
    {0x00,0x00,0x70,0x88,0x80,0x88,0x70,0x00}, // 0x63 'c'
    {0x08,0x08,0x68,0x98,0x88,0x98,0x68,0x00}, // 0x64 'd'
    {0x00,0x00,0x70,0x88,0xF8,0x80,0x70,0x00}, // 0x65 'e'
    {0x30,0x48,0x40,0xE0,0x40,0x40,0x40,0x00}, // 0x66 'f'
    {0x00,0x00,0x68,0x98,0x88,0x68,0x08,0x70}, // 0x67 'g'
    {0x80,0x80,0xB0,0xC8,0x88,0x88,0x88,0x00}, // 0x68 'h'
    {0x20,0x00,0x60,0x20,0x20,0x20,0x70,0x00}, // 0x69 'i'
    {0x10,0x00,0x30,0x10,0x10,0x90,0x60,0x00}, // 0x6A 'j'
    {0x80,0x80,0x90,0xA0,0xC0,0xA0,0x90,0x00}, // 0x6B 'k'
    {0x60,0x20,0x20,0x20,0x20,0x20,0x70,0x00}, // 0x6C 'l'
    {0x00,0x00,0xD0,0xA8,0xA8,0x88,0x88,0x00}, // 0x6D 'm'
    {0x00,0x00,0xB0,0xC8,0x88,0x88,0x88,0x00}, // 0x6E 'n'
    {0x00,0x00,0x70,0x88,0x88,0x88,0x70,0x00}, // 0x6F 'o'
    {0x00,0x00,0xB0,0xC8,0x88,0xC8,0xB0,0x80}, // 0x70 'p'
    {0x00,0x00,0x68,0x98,0x88,0x68,0x08,0x00}, // 0x71 'q'
    {0x00,0x00,0xB0,0xC8,0x80,0x80,0x80,0x00}, // 0x72 'r'
    {0x00,0x00,0x78,0x80,0x70,0x08,0xF0,0x00}, // 0x73 's'
    {0x40,0x40,0xF0,0x40,0x40,0x48,0x30,0x00}, // 0x74 't'
    {0x00,0x00,0x88,0x88,0x88,0x98,0x68,0x00}, // 0x75 'u'
    {0x00,0x00,0x88,0x88,0x88,0x50,0x20,0x00}, // 0x76 'v'
    {0x00,0x00,0x88,0x88,0xA8,0xA8,0x50,0x00}, // 0x77 'w'
    {0x00,0x00,0x88,0x50,0x20,0x50,0x88,0x00}, // 0x78 'x'
    {0x00,0x00,0x88,0x88,0x68,0x08,0x70,0x00}, // 0x79 'y'
    {0x00,0x00,0xF8,0x10,0x20,0x40,0xF8,0x00}, // 0x7A 'z'
    {0x18,0x20,0x20,0x40,0x20,0x20,0x18,0x00}, // 0x7B '{'
    {0x20,0x20,0x20,0x00,0x20,0x20,0x20,0x00}, // 0x7C '|'
    {0x60,0x10,0x10,0x08,0x10,0x10,0x60,0x00}, // 0x7D '}'
    {0x40,0xA8,0x10,0x00,0x00,0x00,0x00,0x00}, // 0x7E '~'
};

static constexpr int FONT_W = 6;
static constexpr int FONT_H = 8;

static void draw_glyph(uint16_t *fb, int fw, int fh,
                       int px, int py, char c,
                       uint16_t fg, uint16_t bg, int scale = 3)
{
    if (c < 0x20 || c > 0x7E) c = '?';
    const uint8_t *g = FONT6x8[c - 0x20];
    for (int row = 0; row < FONT_H; row++) {
        uint8_t bits = g[row];
        for (int col = 0; col < FONT_W; col++) {
            uint16_t color = (bits >> (7-col)) & 1 ? fg : bg;
            for (int sy = 0; sy < scale; sy++)
                for (int sx = 0; sx < scale; sx++) {
                    int x = px + col*scale + sx;
                    int y = py + row*scale + sy;
                    if (x >= 0 && x < fw && y >= 0 && y < fh)
                        fb[y*fw + x] = color;
                }
        }
    }
}

static int draw_string(uint16_t *fb, int fw, int fh,
                       int px, int py, const char *str,
                       uint16_t fg, uint16_t bg, int scale = 3)
{
    int x = px;
    while (*str) {
        draw_glyph(fb, fw, fh, x, py, *str, fg, bg, scale);
        x += FONT_W * scale + scale;
        str++;
    }
    return x - px;
}

static void draw_rect(uint16_t *fb, int fw, int fh,
                      int x, int y, int w, int h, uint16_t color)
{
    for (int row = y; row < y+h && row < fh; row++)
        for (int col = x; col < x+w && col < fw; col++)
            if (col >= 0 && row >= 0)
                fb[row*fw + col] = color;
}

static constexpr uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}
static constexpr uint16_t COLOR_GREEN  = rgb565(0,   255, 0);
static constexpr uint16_t COLOR_YELLOW = rgb565(255, 220, 0);
static constexpr uint16_t COLOR_BG     = rgb565(0,   0,   40);

// -----------------------------------------------------------------------------
// Detection parameters (from reference code)
// -----------------------------------------------------------------------------

static constexpr int SMOOTHING_WINDOW = 5;
static constexpr int MIN_VALID_DETECTIONS = 3;
static constexpr int MAX_NO_DETECT_BEFORE_RESET = 10;
static constexpr int MIN_HAND_AREA = 500;

// Screen dimensions (will be set at runtime from the first frame)
static int SCREEN_W = 320;
static int SCREEN_H = 240;
static float AREA_TOO_CLOSE = 0.30f * 320 * 240;
static float AREA_TOO_FAR   = 0.05f * 320 * 240;

// -----------------------------------------------------------------------------
// Smoothing history and classification (exactly as reference)
// -----------------------------------------------------------------------------

struct SmoothedDetection {
    int cx = -1, cy = -1, area = -1;
    bool valid = false;
};

class HandPositionClassifier {
public:
    void add_detection(int cx, int cy, int area) {
        m_history[m_index].cx = cx;
        m_history[m_index].cy = cy;
        m_history[m_index].area = area;
        m_history[m_index].valid = true;
        m_index = (m_index + 1) % SMOOTHING_WINDOW;
        m_no_detect_counter = 0;
    }

    void clear_history() {
        for (int i = 0; i < SMOOTHING_WINDOW; i++)
            m_history[i].valid = false;
        m_index = 0;
        m_no_detect_counter = 0;
    }

    bool get_smoothed(int &cx_out, int &cy_out, int &area_out) {
        int sum_cx = 0, sum_cy = 0, sum_area = 0, count = 0;
        for (int i = 0; i < SMOOTHING_WINDOW; i++) {
            if (m_history[i].valid) {
                sum_cx += m_history[i].cx;
                sum_cy += m_history[i].cy;
                sum_area += m_history[i].area;
                count++;
            }
        }
        if (count >= MIN_VALID_DETECTIONS) {
            cx_out = sum_cx / count;
            cy_out = sum_cy / count;
            area_out = sum_area / count;
            return true;
        }
        return false;
    }

    void increment_no_detect() {
        m_no_detect_counter++;
        if (m_no_detect_counter >= MAX_NO_DETECT_BEFORE_RESET) {
            clear_history();
        } else {
            m_history[m_index].valid = false;
            m_index = (m_index + 1) % SMOOTHING_WINDOW;
        }
    }

private:
    SmoothedDetection m_history[SMOOTHING_WINDOW];
    int m_index = 0;
    int m_no_detect_counter = 0;
};

static const char *classify_horizontal(int x_center) {
    // Reference thresholds for 320x240 screen
    if (x_center >= 30 && x_center <= 85)   return "Left";
    if (x_center >= 200 && x_center <= 275) return "Right";
    if (x_center >= 160 && x_center <= 180) return "Center";
    return "Center";
}

static const char *classify_vertical(int y_center) {
    if (y_center >= 40 && y_center <= 80)   return "Down";
    if (y_center >= 90 && y_center <= 115)  return "Center";
    if (y_center >= 170 && y_center <= 200) return "Up";
    return "Center";
}

static const char *classify_distance(int area) {
    if (area > AREA_TOO_CLOSE) return "Too Close";
    if (area < AREA_TOO_FAR)   return "Too Far";
    return "Normal";
}

// -----------------------------------------------------------------------------
// Main application class with position classification overlay
// -----------------------------------------------------------------------------

class WhoHandPositionAppLCD : public WhoDetectAppLCD {
public:
    WhoHandPositionAppLCD(WhoFrameCap *frame_cap)
        : WhoDetectAppLCD({{0, 255, 0}}, frame_cap) {}

protected:
    void detect_result_cb(const WhoDetect::result_t &result) override {
        static int64_t last_heap_check = 0;
        static int64_t last_frame_time = 0;
        int64_t now = esp_timer_get_time();

        // Watchdog: if a single frame takes > 400 ms, force early restart
        if (last_frame_time != 0) {
            int64_t frame_dt = now - last_frame_time;
            if (frame_dt > 400000) {
                ESP_LOGW(TAG, "Frame processing too slow (%lld ms), restarting early",
                         frame_dt / 1000);
                vTaskDelay(pdMS_TO_TICKS(100));
                esp_restart();
            }
        }
        last_frame_time = now;

        // Periodic heap monitoring (every 10 seconds)
        if (now - last_heap_check > 10000000) {
            ESP_LOGI(TAG, "Free heap: %lu, PSRAM: %lu, largest block: %lu",
                     esp_get_free_heap_size(),
                     heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                     heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
            last_heap_check = now;
        }

        // Let base class draw detection boxes
        WhoDetectAppLCD::detect_result_cb(result);

        // Update screen dimensions and thresholds on first valid frame
        if (SCREEN_W == 320 && SCREEN_H == 240 && result.img.width > 0) {
            SCREEN_W = result.img.width;
            SCREEN_H = result.img.height;
            AREA_TOO_CLOSE = 0.30f * SCREEN_W * SCREEN_H;
            AREA_TOO_FAR   = 0.05f * SCREEN_W * SCREEN_H;
        }

        // Position classification logic
        if (result.det_res.empty()) {
            m_classifier.increment_no_detect();
            set_gesture("No hand");
        } else {
            // Find best detection (largest area or highest score)
            int best_cx = -1, best_cy = -1, best_area = -1;
            float best_score = 0.0f;

            for (const auto &det : result.det_res) {
                if (det.score < 0.3f) continue;
                int x1 = det.box[0], y1 = det.box[1], x2 = det.box[2], y2 = det.box[3];
                int area = (x2 - x1) * (y2 - y1);
                if (area < MIN_HAND_AREA) continue;

                int cx = (x1 + x2) / 2;
                int cy = (y1 + y2) / 2;
                if (det.score > best_score) {
                    best_score = det.score;
                    best_cx = cx;
                    best_cy = cy;
                    best_area = area;
                }
            }

            if (best_cx != -1) {
                m_classifier.add_detection(best_cx, best_cy, best_area);
            } else {
                m_classifier.increment_no_detect();
            }
        }

        // Get smoothed detection and classify
        int smoothed_cx, smoothed_cy, smoothed_area;
        if (m_classifier.get_smoothed(smoothed_cx, smoothed_cy, smoothed_area)) {
            const char *horiz = classify_horizontal(smoothed_cx);
            const char *vert  = classify_vertical(smoothed_cy);
            const char *dist  = classify_distance(smoothed_area);
            char buf[64];
            snprintf(buf, sizeof(buf), "%s %s %s", horiz, vert, dist);
            set_gesture(buf);
            ESP_LOGI(TAG, "Hand: %s (center %d,%d area %d)",
                     buf, smoothed_cx, smoothed_cy, smoothed_area);
        }

        // Redraw overlay if the gesture string changed
        if (m_needs_redraw) {
            draw_gesture_overlay(result);
            m_needs_redraw = false;
        }
    }

private:
    HandPositionClassifier m_classifier;
    std::string            m_gesture = "No hand";
    bool                   m_needs_redraw = true;

    void set_gesture(const std::string &g) {
        if (g != m_gesture) {
            m_gesture = g;
            m_needs_redraw = true;
        }
    }

    void draw_gesture_overlay(const WhoDetect::result_t &result) {
        if (!result.img.data) return;

        uint16_t *fb = reinterpret_cast<uint16_t *>(result.img.data);
        int fw = result.img.width;
        int fh = result.img.height;

        constexpr int SCALE   = 3;
        constexpr int GLYPH_W = FONT_W * SCALE + SCALE;
        constexpr int GLYPH_H = FONT_H * SCALE;
        constexpr int PAD_X   = 10;
        constexpr int PAD_Y   = 6;

        int len    = (int)m_gesture.size();
        int text_w = len * GLYPH_W - SCALE;
        int text_h = GLYPH_H;
        int box_w  = text_w + PAD_X * 2;
        int box_h  = text_h + PAD_Y * 2;
        int bx     = (fw - box_w) / 2;
        int by     = fh - box_h - 8;

        // Draw background
        draw_rect(fb, fw, fh, bx, by, box_w, box_h, COLOR_BG);

        // Green border (2px)
        for (int i = 0; i < 2; i++) {
            for (int col = bx+i; col < bx+box_w-i; col++) {
                if (by+i >= 0 && by+i < fh)
                    fb[(by+i)*fw + col] = COLOR_GREEN;
                if (by+box_h-1-i >= 0)
                    fb[(by+box_h-1-i)*fw + col] = COLOR_GREEN;
            }
            for (int row = by+i; row < by+box_h-i; row++) {
                if (bx+i >= 0 && bx+i < fw)
                    fb[row*fw + bx+i] = COLOR_GREEN;
                if (bx+box_w-1-i < fw)
                    fb[row*fw + bx+box_w-1-i] = COLOR_GREEN;
            }
        }

        draw_string(fb, fw, fh, bx+PAD_X, by+PAD_Y,
                    m_gesture.c_str(), COLOR_YELLOW, COLOR_BG, SCALE);
    }
};

// -----------------------------------------------------------------------------
// Software restart timer callback (unchanged)
// -----------------------------------------------------------------------------

static void restart_timer_callback(void *arg) {
    ESP_LOGW(TAG, "Periodic restart timer expired – restarting system...");
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();
}

// -----------------------------------------------------------------------------
// app_main
// -----------------------------------------------------------------------------

extern "C" void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

#ifdef BSP_BOARD_ESP32_S3_EYE
    ESP_ERROR_CHECK(bsp_leds_init());
    ESP_ERROR_CHECK(bsp_led_set(BSP_LED_GREEN, false));
#endif

    vTaskDelay(pdMS_TO_TICKS(500));

    // Periodic restart timer (5 minutes)
    constexpr int RESTART_INTERVAL_SEC = 300;
    esp_timer_handle_t restart_timer = nullptr;
    const esp_timer_create_args_t timer_args = {
        .callback = restart_timer_callback,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "restart_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &restart_timer));
    ESP_ERROR_CHECK(esp_timer_start_once(restart_timer, RESTART_INTERVAL_SEC * 1000000ULL));

    // Frame capture pipeline
    auto frame_cap = get_lcd_dvp_frame_cap_pipeline();

    // Camera 180° rotation
    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor) {
        if (sensor->set_vflip) {
            sensor->set_vflip(sensor, 1);
            ESP_LOGI(TAG, "Camera vertical flip enabled");
        }
        if (sensor->set_hmirror) {
            sensor->set_hmirror(sensor, 1);
            ESP_LOGI(TAG, "Camera horizontal mirror enabled");
        }
        ESP_LOGI(TAG, "Camera rotated 180° via sensor registers");
    } else {
        ESP_LOGW(TAG, "Cannot get camera sensor – rotation not applied");
    }

    // Create and run the application
    auto app = new WhoHandPositionAppLCD(frame_cap);
    app->set_model(new HandDetect(
        HandDetect::ESPDET_PICO_224_224_HAND,
        false   // quantization OFF – restart clears any leak
    ));

    ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    ESP_LOGI(TAG, "  Hand Position Classification ready");
    ESP_LOGI(TAG, "  System will restart every %d minutes", RESTART_INTERVAL_SEC/60);
    ESP_LOGI(TAG, "  Move hand Left / Right / Up / Down");
    ESP_LOGI(TAG, "  Or move Closer / Further");
    ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

    app->run();  // blocks forever or until esp_restart()

    esp_timer_delete(restart_timer);
    while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
}
