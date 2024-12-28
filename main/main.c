
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "IMU_BMI323_driver";

#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM 0
#define I2C_MASTER_FREQ_HZ 400000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_TIMEOUT_MS 1000

#define RESET_PIN 5

#define SENSOR_I2C_BASE_ADDRESS 0x68

esp_err_t register_read(const uint8_t regname, uint16_t *value)
{
    if (value == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    const uint8_t *wbuffer = &regname; // 1 byte regaddress

    uint8_t data[2];
    esp_err_t ret;
    ret = i2c_master_write_read_device(I2C_MASTER_NUM, SENSOR_I2C_BASE_ADDRESS, wbuffer, 1, data, 2, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    if (ret == ESP_OK)
    {
        *value = (uint16_t)((data[0] << 8) | data[1]);
        printf("Raw Data from Register 0x%02X 0x%02X\n", data[0], data[1]);
    }
    else
    {
        printf("Error reading from register: 0x%02X, Error Code: %d\n", regname, ret);
    }

    return ret;
}

static esp_err_t register_write(uint8_t reg_addr, uint16_t data)
{
    int ret;
    uint8_t write_buf[3] = {reg_addr, (uint8_t)(data & 0xFF), (uint8_t)(data >> 8)};
    // uint8_t write_buf[2] = { (uint8_t)(data >> 8), (uint8_t)(data & 0xFF)};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, SENSOR_I2C_BASE_ADDRESS, write_buf, 3, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error writing to register 0x%02X, Error Code: %d", reg_addr, ret);
    }

    return ret;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    esp_err_t err_check;
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    // Configure the reset pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RESET_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    gpio_config(&io_conf);

    err_check = i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);

    if (err_check != ESP_OK)
        err_check = i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);

    if (err_check != ESP_OK)
        return 1;
    else
        return 0;
}

void app_main(void)
{
    uint16_t erro;
    uint16_t chipID;
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    gpio_set_level(RESET_PIN, 1);        // Set the reset pin high
    vTaskDelay(10 / portTICK_PERIOD_MS); // Delay for a short duration
    gpio_set_level(RESET_PIN, 0);        // Set the reset pin low
    vTaskDelay(10 / portTICK_PERIOD_MS); // Delay after reset
    // register_write(0x7E, 0xDEAF); // reset
    register_write(0x40, 0x0001); // enable Engine
    register_write(0x10, 0x0038); // FEATURE_IO0
    register_write(0x12, 0x012C); // FEATURE_IO2
    register_write(0x14, 0x0001); // FEATURE_IO_STATUS
    register_write(0x20, 0x3127); // config Accelerometer
    register_write(0x21, 0x404B); // config Gyroscope

    vTaskDelay(50000 / portTICK_PERIOD_MS);

    register_read(0x00, &chipID);
    printf("chipID: %u\n", chipID);
    register_read(0x01, &erro);
    printf("Error_check X: %u\n", erro);

    uint16_t acc_x_value;
    uint16_t acc_y_value;
    uint16_t acc_z_value;
    uint16_t gyr_x_value;
    uint16_t gyr_y_value;
    uint16_t gyr_z_value;
    while (1)
    {

        if (register_read(0x03, &acc_x_value) == ESP_OK)
        {
            printf("Acceleration X: %u\n", acc_x_value);
        }

        // Read Y-axis acceleration
        if (register_read(0x04, &acc_y_value) == ESP_OK)
        {
            printf("Acceleration Y: %u\n", acc_y_value);
        }

        // Read Z-axis acceleration
        if (register_read(0x05, &acc_z_value) == ESP_OK)
        {
            printf("Acceleration Z: %u\n", acc_z_value);
        }

        // Read x-axis Gyroscope
        if (register_read(0x06, &gyr_x_value) == ESP_OK)
        {
            printf("Gyroscope X: %u\n", gyr_x_value);
        }

        // Read Y-axis Gyroscope
        if (register_read(0x07, &gyr_y_value) == ESP_OK)
        {
            printf("Gyroscope Y: %u\n", gyr_y_value);
        }

        // Read Z-axis Gyroscope
        if (register_read(0x08, &gyr_z_value) == ESP_OK)
        {
            printf("Gyroscope Z: %u\n", gyr_z_value);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
