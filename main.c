/* ---- standard headers ---- */
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_random.h"
#include "esp_log.h"
#include "driver/i2c_master.h" // Keep this for the new driver
// #include "driver/i2c.h" // REMOVE this, as it's for the legacy driver
#include "esp_timer.h"
#include <math.h>


/* ---- Bluetooth headers ---- */
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"

/* ---- NVS header (add this one) ---- */
#include "nvs_flash.h"


/* ---------- config ---------- */
#define TAG               "ORI_GYRO"
#define DEVICE_NAME       "ESP32-ORI"
#define SPP_SERVER_NAME   "ORI_STREAM"
#define TX_TASK_STACK     4096
#define TX_TASK_PRIO      5

/* --------- I2C Config --------- */
// I2C_NUM is not used directly with the new driver for bus handle, but kept for clarity if needed elsewhere
#define I2C_NUM         I2C_NUM_0
#define GPIO_SDA        21
#define GPIO_SCL        22
#define I2C_FREQ_HZ     400000

/*---------- MPU-6050 Registers ----------*/
#define MPU_ADDR        0x68
#define REG_PWR_MGMT_1  0x6B
#define REG_SMPLRT_DIV  0x19
#define REG_CONFIG      0x1A
#define REG_GYRO_CFG    0x1B
#define REG_ACCEL_CFG   0x1C
#define REG_ACCEL_OUT   0x3B

// static float roll_deg, pitch_deg, yaw_deg;
// static int64_t last_us = 0;

static uint32_t  s_spp_handle = 0;
static bool      s_connected  = false;

//static variables (1st 2 lines: Gyro Data and 2nd 2 lines: I2C handles)
static float roll_deg, pitch_deg, yaw_deg;
static int64_t last_us = 0;

static i2c_master_bus_handle_t i2c_bus_handle; //Handle to I2C Master Bus
static i2c_master_dev_handle_t mpu6050_dev_handle; //Handle to I2C Device File

//writing data to the MPU
static esp_err_t mpu_write(uint8_t reg_addr, uint8_t val) {
    uint8_t write_buf[2] = {reg_addr, val};
    //parameters: 
    // - [1] device handle 
    // - [2] write buffer (temp storage area in memory that holds the data to be transmitted)
    // - [3] write size
    // - [4] xfer timeout in ms (how long should the program wait for the write to occur before timing out)
    return i2c_master_transmit(mpu6050_dev_handle, write_buf, sizeof(write_buf), 100 / portTICK_PERIOD_MS);
}

//reading fata from the MPU
static esp_err_t mpu_read(uint8_t reg_addr, uint8_t *data, size_t len) {
    //parametes:
    // - [1] device handle
    // - [2] write buffer (this is given as pointer to MPU register address since no data is being written)
    // - [3] write size
    // - [4] read buffer (temp storage area in memory that holds the incoming read data)
    // - [5] read size
    // - [6] xfer timeout in ms
    return i2c_master_transmit_receive(mpu6050_dev_handle, &reg_addr, 1, data, len, 100 / portTICK_PERIOD_MS);
}


esp_err_t gyro_init(void) {

    //configurations for the I2C master bus
    const i2c_master_bus_config_t config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_SDA,
        .scl_io_num = GPIO_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7, //filters out short unwanted pulses
        .flags.enable_internal_pullup = true, //ensures that pull-ups are enables
    };

    //initilized the I2C master bus: [1] * Configuration Settings, [2] * I2C Bus Handle
    ESP_ERROR_CHECK(i2c_new_master_bus(&config, &i2c_bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address = MPU_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };

    //adds a device to the I2C master bus: [1] I2C Bus Handle, [2] * Device Configurations, [3] * Device Handle
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &dev_config, &mpu6050_dev_handle));

    //Wake up the MPU-6050 by writing to its registers
    ESP_ERROR_CHECK(mpu_write(REG_PWR_MGMT_1, 0x01)); //PLL gyro x
    ESP_ERROR_CHECK(mpu_write(REG_SMPLRT_DIV, 0x07)); //1 kHz / (7 + 1) = 125 Hz
    ESP_ERROR_CHECK(mpu_write(REG_CONFIG,     0x03)); //DLPF = 44 Hz
    ESP_ERROR_CHECK(mpu_write(REG_GYRO_CFG,   0x00)); //±250 deg/s
    ESP_ERROR_CHECK(mpu_write(REG_ACCEL_CFG,  0x00)); //±2 g

    last_us = esp_timer_get_time();
    ESP_LOGI(TAG, "MPU-6050 Intialized Successfuly");
    return ESP_OK;

}

void gyro_get_angles_mdeg(int16_t *roll, int16_t *pitch, int16_t *yaw)
{
    uint8_t raw[14];
    // Removed the unused 'reg' parameter from mpu_read, now only reg_addr
    if (mpu_read(REG_ACCEL_OUT, raw, sizeof(raw)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MPU-6050 data!");
        return;
    }

    int16_t ax = (raw[0] << 8) | raw[1];
    int16_t ay = (raw[2] << 8) | raw[3];
    int16_t az = (raw[4] << 8) | raw[5];
    int16_t gx = (raw[8] << 8) | raw[9];
    int16_t gy = (raw[10] << 8) | raw[11];
    int16_t gz = (raw[12] << 8) | raw[13];

    /* convert to physical units */
    const float accel_scale = 16384.0f;        // LSB/g
    const float gyro_scale  = 131.0f;          // LSB/(°/s)

    float ax_g = ax / accel_scale;
    float ay_g = ay / accel_scale;
    float az_g = az / accel_scale;

    float dt = (esp_timer_get_time() - last_us) / 1e6f;
    last_us = esp_timer_get_time(); // Update last_us here for accurate delta time

    /* accel‑only roll & pitch */
    float roll_acc  = atan2f(ay_g, az_g) * 57.29578f;    // rad→deg
    float pitch_acc = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * 57.29578f;

    /* integrate gyro */
    roll_deg  += (gx / gyro_scale) * dt;
    pitch_deg += (gy / gyro_scale) * dt;
    yaw_deg   += (gz / gyro_scale) * dt;

    /* complementary filter */
    const float alpha = 0.98f;
    roll_deg  = alpha * roll_deg  + (1 - alpha) * roll_acc;
    pitch_deg = alpha * pitch_deg + (1 - alpha) * pitch_acc;

    /* wrap yaw into −180 … +180 */
    if (yaw_deg > 180)  yaw_deg -= 360;
    if (yaw_deg < -180) yaw_deg += 360;

    /* output in milli‑degrees */
    *roll  = (int16_t)(roll_deg  * 1000.0f);
    *pitch = (int16_t)(pitch_deg * 1000.0f);
    *yaw   = (int16_t)(yaw_deg   * 1000.0f);
}

static void gyro_serial_tx_task(void *arg) {
    while (true) {
        int16_t roll, pitch, yaw;
        gyro_get_angles_mdeg(&roll, &pitch, &yaw); //gets gyro data
        ESP_LOGI(TAG, "Gyro: Roll=%d Pitch=%d Yaw=%d (mdeg)", roll, pitch, yaw);
        vTaskDelay(pdMS_TO_TICKS(100)); // Reads and Prints Data every (100 ms => Ticks)
    }
}

// New: Handle for the I2C master bus and MPU6050 device
// static i2c_master_bus_handle_t i2c_bus_handle;
// static i2c_master_dev_handle_t mpu6050_dev_handle;


// Updated mpu_write function for the new driver
// static esp_err_t mpu_write(uint8_t reg_addr, uint8_t val) {
//     uint8_t write_buf[2] = {reg_addr, val};
//     // The new driver uses i2c_master_bus_transceive or i2c_master_bus_transfer
//     // i2c_master_bus_transceive(mpu6050_dev_handle, write_buf, sizeof(write_buf), NULL, 0, 100 / portTICK_PERIOD_MS);
//     return i2c_master_transmit(mpu6050_dev_handle, write_buf, sizeof(write_buf), 100 / portTICK_PERIOD_MS);
// }

// // Updated mpu_read function for the new driver
// static esp_err_t mpu_read(uint8_t reg_addr, uint8_t *data, size_t len) {
//     // To read, first transmit the register address, then receive the data
//     return i2c_master_transmit_receive(mpu6050_dev_handle, &reg_addr, 1, data, len, 100 / portTICK_PERIOD_MS);
// }


// esp_err_t gyro_init(void)
// {
//     /* I²C bus */
//     const i2c_master_bus_config_t cfg = {
//         .i2c_port = I2C_NUM_0,
//         .sda_io_num = GPIO_SDA,
//         .scl_io_num = GPIO_SCL,
//         .clk_source = I2C_CLK_SRC_DEFAULT,
//         .glitch_ignore_cnt = 7,
//         .flags.enable_internal_pullup = true,
//     };
//     // Initialize the I2C master bus
//     ESP_ERROR_CHECK(i2c_new_master_bus(&cfg, &i2c_bus_handle));

//     // Configure the MPU-6050 device on the bus
//     i2c_device_config_t dev_cfg = {
//         .dev_addr_length = I2C_ADDR_BIT_7,
//         .device_address = MPU_ADDR,
//         .scl_speed_hz = I2C_FREQ_HZ,
//     };
//     ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &mpu6050_dev_handle));


//     /* wake up MPU‑6050 */
//     ESP_ERROR_CHECK(mpu_write(REG_PWR_MGMT_1, 0x01));        // PLL gyro X
//     ESP_ERROR_CHECK(mpu_write(REG_SMPLRT_DIV, 0x07));        // 1 kHz / (7+1) = 125 Hz
//     ESP_ERROR_CHECK(mpu_write(REG_CONFIG,      0x03));       // DLPF = 44 Hz
//     ESP_ERROR_CHECK(mpu_write(REG_GYRO_CFG,    0x00));       // ±250 °/s
//     ESP_ERROR_CHECK(mpu_write(REG_ACCEL_CFG,   0x00));       // ±2 g

//     last_us = esp_timer_get_time();
//     return ESP_OK;
// }

// void gyro_get_angles_mdeg(int16_t *roll, int16_t *pitch, int16_t *yaw)
// {
//     uint8_t raw[14];
//     // Removed the unused 'reg' parameter from mpu_read, now only reg_addr
//     if (mpu_read(REG_ACCEL_OUT, raw, sizeof(raw)) != ESP_OK) return;

//     int16_t ax = (raw[0] << 8) | raw[1];
//     int16_t ay = (raw[2] << 8) | raw[3];
//     int16_t az = (raw[4] << 8) | raw[5];
//     int16_t gx = (raw[8] << 8) | raw[9];
//     int16_t gy = (raw[10] << 8) | raw[11];
//     int16_t gz = (raw[12] << 8) | raw[13];

//     /* convert to physical units */
//     const float accel_scale = 16384.0f;        // LSB/g
//     const float gyro_scale  = 131.0f;          // LSB/(°/s)

//     float ax_g = ax / accel_scale;
//     float ay_g = ay / accel_scale;
//     float az_g = az / accel_scale;

//     float dt = (esp_timer_get_time() - last_us) / 1e6f;
//     last_us += (int64_t)(dt * 1e6f);

//     /* accel‑only roll & pitch */
//     float roll_acc  = atan2f(ay_g, az_g) * 57.29578f;    // rad→deg
//     float pitch_acc = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * 57.29578f;

//     /* integrate gyro */
//     roll_deg  += (gx / gyro_scale) * dt;
//     pitch_deg += (gy / gyro_scale) * dt;
//     yaw_deg   += (gz / gyro_scale) * dt;

//     /* complementary filter */
//     const float alpha = 0.98f;
//     roll_deg  = alpha * roll_deg  + (1 - alpha) * roll_acc;
//     pitch_deg = alpha * pitch_deg + (1 - alpha) * pitch_acc;

//     /* wrap yaw into −180 … +180 */
//     if (yaw_deg > 180)  yaw_deg -= 360;
//     if (yaw_deg < -180) yaw_deg += 360;

//     /* output in milli‑degrees */
//     *roll  = (int16_t)(roll_deg  * 1000.0f);
//     *pitch = (int16_t)(pitch_deg * 1000.0f);
//     *yaw   = (int16_t)(yaw_deg   * 1000.0f);
// }

/* ---------- random angle helper ---------- */
static inline int16_t rand_angle_mdeg(void)
{
    return (int16_t)42 ;
}



/* ---------- add near the top ---------- */
static void gap_cb(esp_bt_gap_cb_event_t event,
                   esp_bt_gap_cb_param_t *param)
{
    /* You can leave this empty or log events if you like */
}


/* ---------- 1 Hz transmit task ---------- */
static void spp_tx_task(void *arg)
{
    char line[48];

    while (true) {
        if (s_connected) {
            int16_t roll, pitch, yaw;
            gyro_get_angles_mdeg(&roll, &pitch, &yaw);
            // int16_t var = rand_angle_mdeg();
            int len = snprintf(line, sizeof(line), "%d,%d,%d\n", roll, pitch, yaw);
            // int len = snprintf(line, sizeof(line), "%d\n", var);
            esp_spp_write(s_spp_handle, len, (uint8_t *)line);
            ESP_LOGI(TAG, "TX %s", line);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ---------- SPP event callback ---------- */
static void spp_cb(esp_spp_cb_event_t evt, esp_spp_cb_param_t *param)
{
    switch (evt) {

    case ESP_SPP_INIT_EVT:
        ESP_LOGI(TAG, "SPP init OK, starting server");
        esp_spp_start_srv(ESP_SPP_SEC_NONE,
                          ESP_SPP_ROLE_SLAVE,
                          0, SPP_SERVER_NAME);
        break;

    case ESP_SPP_SRV_OPEN_EVT:           /* client connected */
        s_spp_handle = param->srv_open.handle;
        s_connected  = true;
        ESP_LOGI(TAG, "Host connected (handle 0x%" PRIX32 ")", s_spp_handle);  // FIXED
        break;

    case ESP_SPP_CLOSE_EVT:              /* client disconnected */
        s_connected = false;
        ESP_LOGI(TAG, "Host disconnected");
        break;

    default:
        break;
    }
}

/* ---------- main ---------- */
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    // ESP_ERROR_CHECK(gyro_init());
    ESP_ERROR_CHECK(gyro_init());
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BTDM));

    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_bt_gap_register_callback(gap_cb));

    //esp_spp_cb_param_t spp_params; // Declare spp_params
    ESP_ERROR_CHECK(esp_spp_register_callback(spp_cb));
    esp_spp_cfg_t cfg = {
        .mode              = ESP_SPP_MODE_CB,
        .enable_l2cap_ertm = true,
        .tx_buffer_size    = 0,
    };
    ESP_ERROR_CHECK(esp_spp_enhanced_init(&cfg));

    ESP_ERROR_CHECK(esp_bt_gap_set_device_name(DEVICE_NAME));
    ESP_ERROR_CHECK(esp_bt_gap_set_scan_mode(
                    ESP_BT_CONNECTABLE ,          /* esp_bt_connection_mode_t   */
                    ESP_BT_GENERAL_DISCOVERABLE   /* esp_bt_discovery_mode_t    */
    ));


    xTaskCreatePinnedToCore(spp_tx_task, "spp_tx",
                            TX_TASK_STACK, NULL,
                            TX_TASK_PRIO, NULL, 0);
    xTaskCreatePinnedToCore(gyro_serial_tx_task, "gryo_serial_tx",
        TX_TASK_STACK, NULL, TX_TASK_PRIO, NULL, 0);
}
