#include "display.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_io_spi.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_types.h"
#include "esp_lcd_st7735.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "display_font_8x8.h"

#define TAG_DISPLAY "Display"
#define CLK_PIN GPIO_NUM_18 // SCK
#define SDA_PIN GPIO_NUM_23 // MOSI
#define RS_PIN GPIO_NUM_17
#define RST_PIN GPIO_NUM_16
#define CS_PIN GPIO_NUM_5 // CS

#define LCD_HOST SPI2_HOST
#define LCD_H_RES 160
#define LCD_V_RES 128
#define LCD_PIXEL_CLK_HZ (10 * 1000 * 1000)
#define LCD_TEXT_COLOR 0xFFFF
#define LCD_BG_COLOR 0x0000

#define FONT_W 8
#define FONT_H 8
#define FONT_SPACING 0 // currently 0 because the font already has spacing built in
#define CHAR_CELL_W (FONT_W + FONT_SPACING)
#define LINE_HEIGHT (FONT_H + 4)
#define FIELD_MAX_CHARS (LCD_H_RES / CHAR_CELL_W) // 20 characters per line for 8x8 font on 160px wide display

typedef struct {
    int x;
    int y;
    int max_chars;
} display_field_t;

static const display_field_t FIELD_TEMPERATURE = { .x = 0, .y = 0 * LINE_HEIGHT, .max_chars = FIELD_MAX_CHARS };
static const display_field_t FIELD_HUMIDITY    = { .x = 0, .y = 1 * LINE_HEIGHT, .max_chars = FIELD_MAX_CHARS };
static const display_field_t FIELD_PRESSURE    = { .x = 0, .y = 2 * LINE_HEIGHT, .max_chars = FIELD_MAX_CHARS };
static const display_field_t FIELD_MESSAGE_1   = { .x = 0, .y = 3 * LINE_HEIGHT, .max_chars = FIELD_MAX_CHARS };
static const display_field_t FIELD_MESSAGE_2   = { .x = 0, .y = 4 * LINE_HEIGHT, .max_chars = FIELD_MAX_CHARS };
static const display_field_t FIELD_MESSAGE_3   = { .x = 0, .y = 5 * LINE_HEIGHT, .max_chars = FIELD_MAX_CHARS };

static esp_lcd_panel_io_handle_t s_io_handle = NULL;
static esp_lcd_panel_handle_t s_panel_handle = NULL;
static SemaphoreHandle_t s_trans_done_sem = NULL;
static SemaphoreHandle_t s_display_mutex = NULL;

static const char *display_get_glyph(unsigned char c)
{
    if (c > 0x7F) {
        c = '?';
    }

    return font8x8_basic[c];
}

// Runs in ISR context once the SPI/DMA hardware has finished reading the color buffer
static bool IRAM_ATTR notify_color_trans_done(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    SemaphoreHandle_t sem = (SemaphoreHandle_t)user_ctx;
    BaseType_t high_task_woken = pdFALSE;
    xSemaphoreGiveFromISR(sem, &high_task_woken);
    return high_task_woken == pdTRUE;
}

// Issues the draw and blocks until the DMA transfer has finished reading color_data
static esp_err_t display_draw_bitmap_sync(int x_start, int y_start, int x_end, int y_end, const void *color_data)
{
    esp_err_t ret = esp_lcd_panel_draw_bitmap(s_panel_handle, x_start, y_start, x_end, y_end, color_data);
    if (ret != ESP_OK) {
        return ret;
    }

    xSemaphoreTake(s_trans_done_sem, portMAX_DELAY);
    return ESP_OK;
}

static esp_err_t display_fill(uint16_t color)
{
    uint16_t row[LCD_H_RES];
    for (int x = 0; x < LCD_H_RES; ++x) {
        row[x] = color;
    }

    xSemaphoreTake(s_display_mutex, portMAX_DELAY);

    esp_err_t ret = ESP_OK;
    for (int y = 0; y < LCD_V_RES; ++y) {
        ret = display_draw_bitmap_sync(0, y, LCD_H_RES, y + 1, row);
        if (ret != ESP_OK) {
            break;
        }
    }

    xSemaphoreGive(s_display_mutex);

    return ret;
}

// Draws the character c at the given character index in the line, using the provided pixel buffer for the entire line
static void display_draw_char_rows_in_line(uint16_t *pixels, int line_width, int ch_idx, unsigned char c)
{
    // individual character bitmap as array of 8 bytes, each byte is a row of 8 pixels (LSB = leftmost pixel)
    const char *glyph = display_get_glyph(c);

    // draws all the rows of the first character into the line, then all the rows of the second character, etc.
    for (int row = 0; row < FONT_H; ++row) {
        for (int col = 0; col < CHAR_CELL_W; ++col) {

            // pixel is on if the pixel is within the glyph width and the corresponding bit in the glyph row is set
            // casted glyph[row] to unsigned char to avoid sign extension when right shifting
            bool on = (col < FONT_W) && (((unsigned char)glyph[row] >> col) & 0x1);

            pixels[row * line_width + ch_idx * CHAR_CELL_W + col] = on ? LCD_TEXT_COLOR : LCD_BG_COLOR;
        }
    }
}

// Renders a single field line of text
static esp_err_t display_set_field(const display_field_t *field, const char *text)
{
    if (s_panel_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // row-major pixel buffer for the entire line
    static uint16_t pixels[FIELD_MAX_CHARS * CHAR_CELL_W * FONT_H];
    size_t text_len = strlen(text);
    int line_width = field->max_chars * CHAR_CELL_W;

    xSemaphoreTake(s_display_mutex, portMAX_DELAY);

    for (int ch_idx = 0; ch_idx < field->max_chars; ++ch_idx) {
        unsigned char c = (ch_idx < (int)text_len) ? (unsigned char)text[ch_idx] : ' ';
        display_draw_char_rows_in_line(pixels, line_width, ch_idx, c);
    }

    esp_err_t ret = display_draw_bitmap_sync(field->x, field->y, field->x + line_width, field->y + FONT_H, pixels);

    xSemaphoreGive(s_display_mutex);

    return ret;
}

bool display_init_spi_master(void)
{
    if (s_panel_handle != NULL) {
        return true;
    }

    ESP_LOGI(TAG_DISPLAY, "Initializing SPI bus...");
    spi_bus_config_t bus_config = {
        .sclk_io_num = CLK_PIN,
        .mosi_io_num = SDA_PIN,
        .miso_io_num = -1, // Not used
        .quadwp_io_num = -1, // Not used
        .quadhd_io_num = -1, // Not used
        .max_transfer_sz = LCD_H_RES * 20 * sizeof(uint16_t),
    };
    esp_err_t ret = spi_bus_initialize(LCD_HOST, &bus_config, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG_DISPLAY, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
        return false;
    }

    s_trans_done_sem = xSemaphoreCreateBinary();
    if (s_trans_done_sem == NULL) {
        ESP_LOGE(TAG_DISPLAY, "Failed to create transfer-done semaphore");
        return false;
    }

    s_display_mutex = xSemaphoreCreateMutex();
    if (s_display_mutex == NULL) {
        ESP_LOGE(TAG_DISPLAY, "Failed to create display mutex");
        return false;
    }

    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = RS_PIN,
        .cs_gpio_num = CS_PIN,
        .pclk_hz = LCD_PIXEL_CLK_HZ,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = notify_color_trans_done,
        .user_ctx = s_trans_done_sem,
    };

    ESP_LOGI(TAG_DISPLAY, "Creating LCD panel I/O...");
    ret = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &s_io_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_DISPLAY, "esp_lcd_new_panel_io_spi failed: %s", esp_err_to_name(ret));
        return false;
    }

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = RST_PIN,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };

    ESP_LOGI(TAG_DISPLAY, "Creating LCD panel...");
    ret = esp_lcd_new_panel_st7735(s_io_handle, &panel_config, &s_panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_DISPLAY, "esp_lcd_new_panel_st7735 failed: %s", esp_err_to_name(ret));
        return false;
    }

    ret = esp_lcd_panel_reset(s_panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_DISPLAY, "esp_lcd_panel_reset failed: %s", esp_err_to_name(ret));
        return false;
    }

    ret = esp_lcd_panel_init(s_panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_DISPLAY, "esp_lcd_panel_init failed: %s", esp_err_to_name(ret));
        return false;
    }

    // display as landscape
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(s_panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(s_panel_handle, false, true));

    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel_handle, true));
    vTaskDelay(pdMS_TO_TICKS(120));

    return true;
}

bool app_display_init(void){
    ESP_LOGI(TAG_DISPLAY, "Initializing display...");

    if (!display_init_spi_master()) {
        return false;
    }

    ESP_ERROR_CHECK(display_fill(LCD_BG_COLOR));
    app_display_set_temperature(0.0);
    app_display_set_humidity(0.0);
    app_display_set_pressure(0.0);
    app_display_set_message_1("");
    app_display_set_message_2("");
    app_display_set_message_3("");

    return true;
}

bool app_display_set_temperature(double val)
{
    char buf[FIELD_MAX_CHARS + 1];
    snprintf(buf, sizeof(buf), "Temp: %.1fC", val);
    return display_set_field(&FIELD_TEMPERATURE, buf) == ESP_OK;
}

bool app_display_set_humidity(double val)
{
    char buf[FIELD_MAX_CHARS + 1];
    snprintf(buf, sizeof(buf), "Humi: %.1f%%", val);
    return display_set_field(&FIELD_HUMIDITY, buf) == ESP_OK;
}

bool app_display_set_pressure(double val)
{
    char buf[FIELD_MAX_CHARS + 1];
    snprintf(buf, sizeof(buf), "Pres: %.1fPa", val);
    return display_set_field(&FIELD_PRESSURE, buf) == ESP_OK;
}

bool app_display_set_message_1(const char *msg)
{
    return display_set_field(&FIELD_MESSAGE_1, msg) == ESP_OK;
}

bool app_display_set_message_2(const char *msg)
{
    return display_set_field(&FIELD_MESSAGE_2, msg) == ESP_OK;
}

bool app_display_set_message_3(const char *msg)
{
    return display_set_field(&FIELD_MESSAGE_3, msg) == ESP_OK;
}
