#include <stdio.h>
#include <lsm9ds1_reg.h>
#include <driver/i2c_master.h>

#include <esp_err.h>
#include <esp_log.h>
#include <esp_check.h>

#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "main";

#define I2C_MASTER_SCL_IO GPIO_NUM_7 /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO GPIO_NUM_6 /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_FREQ_HZ 100000    /*!< I2C master clock frequency */
#define I2C_MASTER_TIMEOUT_MS 5000

#define IMU_ADDR 0x6a
#define MAG_ADDR 0x1c

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t imu_dev_handle;
static i2c_master_dev_handle_t mag_dev_handle;

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = -1,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    return ESP_OK;
}

static int32_t platform_write(void *handle, uint8_t reg,
                              const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg,
                             uint8_t *bufp, uint16_t len);
static void platform_delay(uint32_t ms);
static void tx_com(uint8_t *tx_buffer, uint16_t len);

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &imu_dev_handle));

    dev_cfg.device_address = MAG_ADDR;

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &mag_dev_handle));

    stmdev_ctx_t imu_dev = {
        .read_reg = platform_read,
        .write_reg = platform_write,
        .mdelay = platform_delay,
        .handle = imu_dev_handle};

    stmdev_ctx_t mag_dev = {
        .read_reg = platform_read,
        .write_reg = platform_write,
        .mdelay = platform_delay,
        .handle = mag_dev_handle};

    platform_delay(20);

    lsm9ds1_id_t id;
    int32_t ret = lsm9ds1_dev_id_get(&mag_dev, &imu_dev, &id);
    ESP_LOGI(TAG, "ret: %" PRIi32 ", WHO_AM_I: %" PRIx8 ", WHO_AM_I_M: %" PRIx8, ret, id.imu, id.mag);
    if (id.imu != LSM9DS1_IMU_ID || id.mag != LSM9DS1_MAG_ID)
    {
        ESP_LOGE(TAG, "Did not find lsm9ds1, stopping here, reset if you want to check again");
        return;
    }

    { /* Restore default configuration */
        lsm9ds1_dev_reset_set(&mag_dev, &imu_dev, PROPERTY_ENABLE);
        uint8_t rst;
        do
        {
            lsm9ds1_dev_reset_get(&mag_dev, &imu_dev, &rst);
            platform_delay(50);
        } while (rst);
        ESP_LOGI(TAG, "reset lsm9ds1 to default config");
    }

    /* Enable Block Data Update */
    lsm9ds1_block_data_update_set(&mag_dev, &imu_dev,
                                  PROPERTY_ENABLE);
    /* Set full scale */
    lsm9ds1_xl_full_scale_set(&imu_dev, LSM9DS1_4g);
    lsm9ds1_gy_full_scale_set(&imu_dev, LSM9DS1_2000dps);
    // lsm9ds1_mag_full_scale_set(&mag_dev, LSM9DS1_16Ga);
    /* Configure filtering chain - See datasheet for filtering chain details */
    /* Accelerometer filtering chain */
    lsm9ds1_xl_filter_aalias_bandwidth_set(&imu_dev, LSM9DS1_AUTO);
    lsm9ds1_xl_filter_lp_bandwidth_set(&imu_dev,
                                       LSM9DS1_LP_ODR_DIV_50);
    lsm9ds1_xl_filter_out_path_set(&imu_dev, LSM9DS1_LP_OUT);
    /* Gyroscope filtering chain */
    lsm9ds1_gy_filter_lp_bandwidth_set(&imu_dev,
                                       LSM9DS1_LP_ULTRA_LIGHT);
    lsm9ds1_gy_filter_hp_bandwidth_set(&imu_dev, LSM9DS1_HP_MEDIUM);
    lsm9ds1_gy_filter_out_path_set(&imu_dev,
                                   LSM9DS1_LPF1_HPF_LPF2_OUT);
    /* Set Output Data Rate / Power mode */
    lsm9ds1_imu_data_rate_set(&imu_dev, LSM9DS1_IMU_14Hz9);
    lsm9ds1_mag_data_rate_set(&mag_dev, LSM9DS1_MAG_UHP_10Hz);

    /* Read samples in polling mode (no int) */
    while (1)
    {
        lsm9ds1_status_t reg;
        /* Read device status register */
        lsm9ds1_dev_status_get(&mag_dev, &imu_dev, &reg);

        uint8_t tx_buffer[1000];

        if (reg.status_imu.xlda && reg.status_imu.gda)
        {
            int16_t data_raw_acceleration[3] = {0};
            int16_t data_raw_angular_rate[3] = {0};
            float acceleration_mg[3];
            float angular_rate_mdps[3];

            /* Read imu data */
            lsm9ds1_acceleration_raw_get(&imu_dev,
                                         data_raw_acceleration);
            lsm9ds1_angular_rate_raw_get(&imu_dev,
                                         data_raw_angular_rate);
            acceleration_mg[0] = lsm9ds1_from_fs4g_to_mg(
                data_raw_acceleration[0]);
            acceleration_mg[1] = lsm9ds1_from_fs4g_to_mg(
                data_raw_acceleration[1]);
            acceleration_mg[2] = lsm9ds1_from_fs4g_to_mg(
                data_raw_acceleration[2]);
            angular_rate_mdps[0] = lsm9ds1_from_fs2000dps_to_mdps(
                data_raw_angular_rate[0]);
            angular_rate_mdps[1] = lsm9ds1_from_fs2000dps_to_mdps(
                data_raw_angular_rate[1]);
            angular_rate_mdps[2] = lsm9ds1_from_fs2000dps_to_mdps(
                data_raw_angular_rate[2]);
            sprintf((char *)tx_buffer,
                    "IMU - [mg]:%4.2f\t%4.2f\t%4.2f\t[mdps]:%4.2f\t%4.2f\t%4.2f",
                    acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],
                    angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
            tx_com(tx_buffer, strlen((char const *)tx_buffer));
        }

        if (reg.status_mag.zyxda)
        {
            int16_t data_raw_magnetic_field[3] = {0};
            float magnetic_field_mgauss[3];
            /* Read magnetometer data */
            memset(data_raw_magnetic_field, 0x00, 3 * sizeof(int16_t));
            lsm9ds1_magnetic_raw_get(&mag_dev, data_raw_magnetic_field);
            magnetic_field_mgauss[0] = lsm9ds1_from_fs16gauss_to_mG(
                data_raw_magnetic_field[0]);
            magnetic_field_mgauss[1] = lsm9ds1_from_fs16gauss_to_mG(
                data_raw_magnetic_field[1]);
            magnetic_field_mgauss[2] = lsm9ds1_from_fs16gauss_to_mG(
                data_raw_magnetic_field[2]);
            sprintf((char *)tx_buffer, "MAG - [mG]:%4.2f\t%4.2f\t%4.2f\r\n",
                    magnetic_field_mgauss[0], magnetic_field_mgauss[1],
                    magnetic_field_mgauss[2]);
            tx_com(tx_buffer, strlen((char const *)tx_buffer));
        }

        platform_delay(10);
    }
}

static int32_t platform_write(void *handle, uint8_t reg,
                              const uint8_t *bufp, uint16_t len)
{
    size_t size = len + 1;
    uint8_t *write_buf = malloc(size);
    if (write_buf == NULL)
    {
        return -1;
    }

    write_buf[0] = reg;
    memcpy(&write_buf[1], bufp, len);

    esp_err_t err = i2c_master_transmit(handle, write_buf, size, I2C_MASTER_TIMEOUT_MS);

    free(write_buf);

    return (int32_t)err;
}

static int32_t platform_read(void *handle, uint8_t reg,
                             uint8_t *bufp, uint16_t len)
{
    return (int32_t)i2c_master_transmit_receive(handle, &reg, 1, bufp, len, I2C_MASTER_TIMEOUT_MS);
}

static void platform_delay(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
    ESP_LOGI("tx_com", "%.*s", len, (char *)tx_buffer);
}
