#include "display.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_io_spi.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_types.h"
#include "esp_lcd_st7735.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "display_font_8x8.h"

#define TAG_DISPLAY "Display"
#define CLK_PIN GPIO_NUM_18 // SCK
#define SDA_PIN GPIO_NUM_23 // MOSI
#define RS_PIN GPIO_NUM_17
#define RST_PIN GPIO_NUM_16
#define CS_PIN GPIO_NUM_5 // CS

#define LCD_HOST SPI2_HOST
#define LCD_H_RES 128
#define LCD_V_RES 160
#define LCD_PIXEL_CLK_HZ (10 * 1000 * 1000)
#define LCD_TEXT_COLOR 0xFFFF
#define LCD_BG_COLOR 0x0000

#define FONT_W 8
#define FONT_H 8
#define FONT_SPACING 1

static esp_lcd_panel_io_handle_t s_io_handle = NULL;
static esp_lcd_panel_handle_t s_panel_handle = NULL;

static const char *display_get_glyph(unsigned char c)
{
    if (c > 0x7F) {
        c = '?';
    }

    return font8x8_basic[c];
}

static esp_err_t display_fill(uint16_t color)
{
    // static for now to avoid memory allocation drawing issues at bottom row
    // because esp_lcd_panel_draw_bitmap is async DMA
    // to be fixed
    static uint16_t row[LCD_H_RES];
    for (int x = 0; x < LCD_H_RES; ++x) {
        row[x] = color;
    }

    for (int y = 0; y < LCD_V_RES; ++y) {
        esp_err_t ret = esp_lcd_panel_draw_bitmap(s_panel_handle, 0, y, LCD_H_RES, y + 1, row);
        if (ret != ESP_OK) {
            return ret;
        }
    }

    return ESP_OK;
}

static esp_err_t display_draw_char(int x, int y, unsigned char c, uint16_t fg, uint16_t bg)
{
    const char *glyph = display_get_glyph(c);

    // static for now to avoid memory allocation drawing issues at bottom row
    // because esp_lcd_panel_draw_bitmap is async DMA
    // to be fixed
    static uint16_t pixels[FONT_W * FONT_H];

    for (int row = 0; row < FONT_H; ++row) {
        for (int col = 0; col < FONT_W; ++col) {
            bool on = ((unsigned char)glyph[row] >> col) & 0x1;
            pixels[row * FONT_W + col] = on ? fg : bg;
        }
    }

    return esp_lcd_panel_draw_bitmap(s_panel_handle, x, y, x + FONT_W, y + FONT_H, pixels);
}

static esp_err_t display_draw_text(int x, int y, const char *text)
{
    int cursor_x = x;
    for (size_t i = 0; text[i] != '\0'; ++i) {
        esp_err_t ret = display_draw_char(cursor_x, y, (unsigned char)text[i], LCD_TEXT_COLOR, LCD_BG_COLOR);
        if (ret != ESP_OK) {
            return ret;
        }
        cursor_x += FONT_W + FONT_SPACING;
    }
    return ESP_OK;
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

    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = RS_PIN,
        .cs_gpio_num = CS_PIN,
        .pclk_hz = LCD_PIXEL_CLK_HZ,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
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

    const char *msg = "A";
    int x = 0;
    int y = 0;
    ESP_ERROR_CHECK(display_draw_text(x, y, msg));

    return true;
}