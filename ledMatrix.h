#include "esp_system.h"
#include "driver/spi_master.h"

#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK 18
#define PIN_NUM_CS 13

#define PARALLEL_LINES 16

// max7219 registers
#define MAX7219_REG_NOOP 0x0
#define MAX7219_REG_DIGIT0 0x1
#define MAX7219_REG_DIGIT1 0x2
#define MAX7219_REG_DIGIT2 0x3
#define MAX7219_REG_DIGIT3 0x4
#define MAX7219_REG_DIGIT4 0x5
#define MAX7219_REG_DIGIT5 0x6
#define MAX7219_REG_DIGIT6 0x7
#define MAX7219_REG_DIGIT7 0x8
#define MAX7219_REG_DECODEMODE 0x9
#define MAX7219_REG_INTENSITY 0xA
#define MAX7219_REG_SCANLIMIT 0xB
#define MAX7219_REG_SHUTDOWN 0xC
#define MAX7219_REG_DISPLAYTEST 0xF

#define MATRIX_NUMBER_OF_DISPLAYS 4

spi_device_handle_t spi;

uint16_t matrix_cols[16];

void matrix_sendByte(const uint8_t device, const uint8_t reg, const uint8_t data)
{
    esp_err_t ret;
    spi_transaction_t t, tDummy;

    uint16_t buf = (data << 8) | (reg);
    uint16_t bufDummy = 0;

    memset(&t, 0, sizeof(t));           // Zero out the transaction
    t.length = 16;                      // Len is in bytes, transaction length is in bits.
    t.tx_buffer = &buf;                 // Data
    t.flags = SPI_TRANS_CS_KEEP_ACTIVE; // Keep CS active after data transfer

    memset(&tDummy, 0, sizeof(tDummy)); // Zero out the transaction
    tDummy.length = 16;
    tDummy.tx_buffer = &bufDummy;
    tDummy.flags = SPI_TRANS_CS_KEEP_ACTIVE; // Keep CS active after data transfer

    // ESP_LOGI(TAG, "device %u: sending address %#04x: %#04x", i, matrix_spiregister[i], matrix_spidata[i]);

    spi_device_acquire_bus(spi, portMAX_DELAY);
    for (uint8_t i = 0; i < MATRIX_NUMBER_OF_DISPLAYS; i++)
    {
        if (i == MATRIX_NUMBER_OF_DISPLAYS - 1)
        {
            t.flags = 0;
            tDummy.flags = 0;
        }
        if (i == device)
        {
            ret = spi_device_polling_transmit(spi, &t); // Transmit!
            assert(ret == ESP_OK);                      // Should have had no issues.
        }
        else
        {
            ret = spi_device_polling_transmit(spi, &tDummy); // Transmit!
            assert(ret == ESP_OK);                           // Should have had no issues.
        }
    }
    spi_device_release_bus(spi);
}
void matrix_sendByte_all(const uint8_t reg, const uint8_t data)
{
    esp_err_t ret;
    spi_transaction_t t;

    uint16_t buf = (data << 8) | (reg);

    memset(&t, 0, sizeof(t));           // Zero out the transaction
    t.length = 16;                      // Len is in bytes, transaction length is in bits.
    t.tx_buffer = &buf;                 // Data
    t.flags = SPI_TRANS_CS_KEEP_ACTIVE; // Keep CS active after data transfer

    // ESP_LOGI(TAG, "device %u: sending address %#04x: %#04x", i, matrix_spiregister[i], matrix_spidata[i]);

    spi_device_acquire_bus(spi, portMAX_DELAY);
    for (uint8_t i = 0; i < MATRIX_NUMBER_OF_DISPLAYS; i++)
    {
        if (i == MATRIX_NUMBER_OF_DISPLAYS - 1)
        {
            t.flags = 0;
        }
        ret = spi_device_polling_transmit(spi, &t); // Transmit!
        assert(ret == ESP_OK);                      // Should have had no issues.
    }
    spi_device_release_bus(spi);
}

void matrix_writeCol(uint8_t device, uint8_t col, uint8_t data)
{
    matrix_sendByte(device, col + 1, data);
}

void matrix_displayTest(uint8_t device, bool active)
{
    matrix_sendByte(device, MAX7219_REG_DISPLAYTEST, active ? 1 : 0);
}

// sends out message to turn off all leds
void matrix_device_clear()
{
    for (uint8_t device = 0; device < MATRIX_NUMBER_OF_DISPLAYS; device++)
    {
        for (uint8_t col = 0; col < 8; col++)
        {
            matrix_sendByte(device, col + 1, 0);
        }
    }
}

// write all zeros to display buffer
void matrix_clear()
{
    for (uint8_t col = 0; col < 16; col++)
    {
        matrix_cols[col] = 0;
    }
}

void matrix_testCircle(float x, float y, float r, float strokeWidth)
{
    for (uint8_t px = 0; px < 16; px++)
    {
        for (uint8_t py = 0; py < 16; py++)
        {
            float dx = (float)px - x;
            float dy = (float)py - y;
            float dist = sqrtf(dx * dx + dy * dy);
            if ((dist > r - strokeWidth) & (dist < r + strokeWidth))
            {
                matrix_cols[py] = matrix_cols[py] | (1 << px);
            }
        }
    }
}

// clears display and then sends matrix_cols
void matrix_render()
{
    // matrix_device_clear();
    uint8_t byteOffset = 0, colOffset = 0;

    for (uint8_t device = 0; device < MATRIX_NUMBER_OF_DISPLAYS; device++)
    {
        for (uint8_t col = 0; col < 16; col++)
        {
            if (device == 1 && col < 8)
            { // top left
                byteOffset = 0;
                colOffset = 0;
                matrix_writeCol(device, col - colOffset, matrix_cols[col] >> byteOffset);
            }
            else if (device == 0 && col < 8)
            { // top right
                byteOffset = 8;
                colOffset = 0;
                matrix_writeCol(device, col - colOffset, matrix_cols[col] >> byteOffset);
            }
            else if (device == 3 && col >= 8)
            { // bottom left
                byteOffset = 0;
                colOffset = 8;
                matrix_writeCol(device, col - colOffset, matrix_cols[col] >> byteOffset);
            }
            else if (device == 2 && col >= 8)
            { // bottom right
                byteOffset = 8;
                colOffset = 8;
                matrix_writeCol(device, col - colOffset, matrix_cols[col] >> byteOffset);
            }
        }
    }
}

void matrix_testMovingDot()
{
    static uint8_t x = 0, y = 0;
    x = (x + 1) % 16;
    if (x == 0)
        y = (y + 1) % 16;
    matrix_cols[y] = matrix_cols[y] | 1 << x;
}

inline void matrix_setPixel(uint8_t x, uint8_t y)
{
    if ((x > 16) || (y > 16))
        return;
    matrix_cols[y] = matrix_cols[y] | 1 << x;
}

void matrix_init()
{
    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .miso_io_num = -1,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = PARALLEL_LINES * 8 * 2 + 8};
    spi_device_interface_config_t devcfg = {
        //.clock_speed_hz=26*1000*1000,           //Clock out at 26 MHz
        //.clock_speed_hz = 10 * 1000 * 1000, // Clock out at 10 MHz
        .clock_speed_hz = 10 * 1000 * 1000, // Clock out at 10 MHz
        .mode = 0,                          // SPI mode 0
        .spics_io_num = PIN_NUM_CS,         // CS pin
        .queue_size = 7                     // We want to be able to queue 7 transactions at a time
        //.pre_cb = lcd_spi_pre_transfer_callback, // Specify pre-transfer callback to handle D/C line
    };

    // Initialize the SPI bus
    ret = spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    // Attach the LCD to the SPI bus
    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
    // Initialize the LCD

    ESP_LOGI(TAG, "Initializing matrix display...");

    ESP_LOGI(TAG, " - clearing");
    matrix_clear();
    ESP_LOGI(TAG, " - MAX7219_REG_SCANLIMIT 7");
    matrix_sendByte_all(MAX7219_REG_SCANLIMIT, 7); // show all 8 digits
    vTaskDelay(20 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, " - MAX7219_REG_DECODEMODE 0");
    matrix_sendByte_all(MAX7219_REG_DECODEMODE, 0); // using an led matrix (not digits)
    vTaskDelay(20 / portTICK_PERIOD_MS);
    // ESP_LOGI(TAG, " - MAX7219_REG_DISPLAYTEST ON");
    // matrix_sendByte_all(MAX7219_REG_DISPLAYTEST, 1); // display test
    // vTaskDelay(50 / portTICK_PERIOD_MS);
    // ESP_LOGI(TAG, " - MAX7219_REG_DISPLAYTEST OFF");
    matrix_sendByte_all(MAX7219_REG_DISPLAYTEST, 0); // display test
    vTaskDelay(20 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, " - MAX7219_REG_INTENSITY 0");
    matrix_sendByte_all(MAX7219_REG_INTENSITY, 0); // character intensity: range: 0 to 15
    vTaskDelay(20 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, " - MAX7219_REG_SHUTDOWN OFF");
    matrix_sendByte_all(MAX7219_REG_SHUTDOWN, 1); // not in shutdown mode (ie. start it up)
    vTaskDelay(20 / portTICK_PERIOD_MS);

    /*
    for (uint8_t device = 0; device < MATRIX_NUMBER_OF_DISPLAYS; device++)
    {
        ESP_LOGI(TAG, "device %u: display ON", device);
        matrix_displayTest(device, true);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "device %u: display OFF", device);
        matrix_displayTest(device, false);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    */

    while (0)
    {
        matrix_device_clear();
        matrix_clear();
        matrix_testMovingDot();
        matrix_render();
        vTaskDelay(30 / portTICK_PERIOD_MS);
    }
}
// ptr is pointer to 16 element int8_t array
//  with values from -127 to 127 to 16 for sample level
void ledMatrix_plot_signal_circle(int8_t *ptr)
{
    int16_t level = 0;
    for (int i = 0; i < 16; i++)
    {
        level += ABS(ptr[i]);
    }
    static float r = 0;
    // r = (r * 31.0 + (level / 32)) / 32;
    r = level / 256;
    // r = CLAMP(r, 0, 8);
    /*
    r += .01;
    if (r > 6)
        r = 2;
    */
    matrix_clear();
    matrix_testCircle(8, 8, r, .5);
    matrix_render();
}
void ledMatrix_plot_signal(int8_t *ptr)
{
    matrix_clear();
    for (uint8_t x = 0; x < 16; x++)
    {
        matrix_setPixel(x, ptr[x] / 16 + 7);
    }
    matrix_render();
}
