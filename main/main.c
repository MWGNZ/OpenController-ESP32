#include "main.h"

#define I2C_MASTER_SCL_IO GPIO_NUM_19 /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO GPIO_NUM_18 /*!< GPIO number used for I2C master data  */
icm20948_device_t icm;

i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = I2C_MASTER_SDA_IO,
    .scl_io_num = I2C_MASTER_SCL_IO,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 400000,
    .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL};

icm0948_config_i2c_t icm_config = {
    .i2c_port = I2C_NUM_0,
    .i2c_addr = ICM_20948_I2C_ADDR_AD0};

// vector to hold quaternion
float q[4] = {1.0, 0.0, 0.0, 0.0};

float yaw, pitch, roll; // Euler angle output
float hold_yaw, hold_roll;

#define GPIO_BTN_A GPIO_NUM_23
#define GPIO_BTN_B GPIO_NUM_22
#define GPIO_BTN_X GPIO_NUM_17
#define GPIO_BTN_Y GPIO_NUM_16

#define GPIO_BTN_L GPIO_NUM_14
#define GPIO_BTN_L2 GPIO_NUM_3
#define GPIO_BTN_R GPIO_NUM_15
#define GPIO_BTN_R2 GPIO_NUM_4
#define GPIO_BTN_L3 GPIO_NUM_5
#define GPIO_BTN_R3 GPIO_NUM_12

#define GPIO_BTN_DU GPIO_NUM_25
#define GPIO_BTN_DD GPIO_NUM_13
#define GPIO_BTN_DL GPIO_NUM_27
#define GPIO_BTN_DR GPIO_NUM_26

#define GPIO_BTN_GYRO_ON GPIO_NUM_34
#define GPIO_BTN_START GPIO_NUM_36
#define GPIO_BTN_SELECT GPIO_NUM_39
#define GPIO_BTN_HOME GPIO_NUM_35

#define ADC_STICK_LX ADC1_CHANNEL_4 /*!< ADC1 channel 4 is GPIO32 */
#define ADC_STICK_LY ADC1_CHANNEL_5 /*!< ADC1 channel 5 is GPIO33 */
// #define ADC_STICK_RX     ADC1_CHANNEL_6
// #define ADC_STICK_RY     ADC1_CHANNEL_7

#define GPIO_INPUT_PIN_MASK ((1ULL << GPIO_BTN_A) | (1ULL << GPIO_BTN_B) | (1ULL << GPIO_BTN_X) | (1ULL << GPIO_BTN_Y) | (1ULL << GPIO_BTN_L) | (1ULL << GPIO_BTN_L2) | (1ULL << GPIO_BTN_R) | (1ULL << GPIO_BTN_R2) | (1ULL << GPIO_BTN_L3) | (1ULL << GPIO_BTN_R3) | (1ULL << GPIO_BTN_DU) | (1ULL << GPIO_BTN_DD) | (1ULL << GPIO_BTN_DL) | (1ULL << GPIO_BTN_DR) | (1ULL << GPIO_BTN_GYRO_ON) | (1ULL << GPIO_BTN_START) | (1ULL << GPIO_BTN_SELECT) | (1ULL << GPIO_BTN_HOME))

int DPAD_ANALOG_POS = 1023;

hoja_controller_mode_t CURENT_CONTROLLER_MODE;

bool gyro_is_on;
bool last_dup = false;
bool last_ddown = false;
bool last_dleft = false;
bool last_dright = false;

void init_dmp(icm20948_device_t *icm)
{
    const char *TAG = "init_dmp";
    bool success = true; // Use success to show if the DMP configuration was successful

    // Initialize the DMP with defaults.
    success &= (icm20948_init_dmp_sensor_with_defaults(icm) == ICM_20948_STAT_OK);
    // DMP sensor options are defined in ICM_20948_DMP.h
    //    INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
    //    INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro + 32-bit calibrated gyro)
    //    INV_ICM20948_SENSOR_RAW_ACCELEROMETER           (16-bit accel)
    //    INV_ICM20948_SENSOR_RAW_GYROSCOPE               (16-bit gyro + 32-bit calibrated gyro)
    //    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
    //    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
    //    INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step Detector)
    //    INV_ICM20948_SENSOR_STEP_COUNTER                (Pedometer Step Detector)
    //    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_ROTATION_VECTOR             (32-bit 9-axis quaternion + heading accuracy)
    //    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy)
    //    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
    //    INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)

    // Enable the DMP orientation sensor
    success &= (inv_icm20948_enable_dmp_sensor(icm, INV_ICM20948_SENSOR_ORIENTATION, 1) == ICM_20948_STAT_OK);

    // Enable any additional sensors / features
    // success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_STAT_OK);
    // success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_STAT_OK);
    // success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_STAT_OK);

    // Configuring DMP to output data at multiple ODRs:
    // DMP is capable of outputting multiple sensor data at different rates to FIFO.
    // Setting value can be calculated as follows:
    // Value = (DMP running rate / ODR ) - 1
    // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
    success &= (inv_icm20948_set_dmp_sensor_period(icm, DMP_ODR_Reg_Quat9, 0) == ICM_20948_STAT_OK); // Set to the maximum
    // success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_STAT_OK); // Set to the maximum
    // success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_STAT_OK); // Set to the maximum
    // success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_STAT_OK); // Set to the maximum
    // success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_STAT_OK); // Set to the maximum
    // success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0) == ICM_20948_STAT_OK); // Set to the maximum
    //  Enable the FIFO
    success &= (icm20948_enable_fifo(icm, true) == ICM_20948_STAT_OK);
    // Enable the DMP
    success &= (icm20948_enable_dmp(icm, 1) == ICM_20948_STAT_OK);
    // Reset DMP
    success &= (icm20948_reset_dmp(icm) == ICM_20948_STAT_OK);
    // Reset FIFO
    success &= (icm20948_reset_fifo(icm) == ICM_20948_STAT_OK);

    // Check success
    if (success)
    {
        ESP_LOGI(TAG, "DMP enabled!");
    }
    else
    {
        ESP_LOGE(TAG, "Enable DMP failed!");
        while (1)
            ; // Do nothing more
    }
}

void UpdateReadings()
{
    while (1)
    {
        icm_20948_DMP_data_t data;
        icm20948_status_e status = inv_icm20948_read_dmp_data(&icm, &data);
        /* Was valid data available? */
        if ((status == ICM_20948_STAT_OK) || (status == ICM_20948_STAT_FIFO_MORE_DATA_AVAIL))
        {
            /* We have asked for orientation data so we should receive Quat9 */
            if ((data.header & DMP_header_bitmap_Quat9) > 0)
            {
                // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
                // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
                // The quaternion data is scaled by 2^30.
                // Scale to +/- 1
                q[1] = ((double)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
                q[2] = ((double)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
                q[3] = ((double)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
                q[0] = sqrt(1.0 - ((q[1] * q[1]) + (q[2] * q[2]) + (q[3] * q[3])));
            }
        }
    }
}

void do_math()
{
    roll = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
    pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
    // conventional yaw increases clockwise from North.
    yaw = -atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - (q[2] * q[2] + q[3] * q[3]));
    // to degrees
    yaw *= 180.0 / M_PI;
    if (yaw < 0)
        yaw += 360.0; // compass circle
    pitch *= 180.0 / M_PI;
    roll *= 180.0 / M_PI;
}

uint16_t fudge_reading(uint16_t raw_reading, bool invert)
{
    if (invert)
    {
        raw_reading = 4095 - raw_reading;
    }

    int midpoint = 2048;
    int deadzone = 260;
    if (((midpoint - deadzone) < raw_reading) && (raw_reading < (midpoint + deadzone)))
    {
        return midpoint;
    }
    return raw_reading;
}

uint16_t clamp(float value, float min, float max)
{
    if (value < min)
        return min;
    if (value > max)
        return max;
    return value;
}

// yaw is x axis
uint16_t fix_yaw(float yaaw)
{
    int deadzone = 2;
    if ((-deadzone < yaaw) && (yaaw < deadzone))
    {
        return 1975;
    }

    float rotate = fmod((yaaw + 180), 360.0); // rotate so 180 is midpoint

    return clamp(15 * 1.6 * (rotate - 180) + 1975, 1, 4000);
}

// roll is y axis
uint16_t fix_roll(float rollll)
{
    int deadzone = 3;
    if ((-deadzone < rollll) && (rollll < deadzone))
    {
        return 1975;
    }

    float rotate = fmod((rollll + 180), 360.0);

    return clamp(15 * 1.8 * (rotate - 180) + 1975, 1, 4000);
}

// Separate task to read sticks.
// This is essential to have as a separate component as ADC scans typically take more time and this is only
// scanned once between each polling interval. This varies from core to core.
void local_analog_cb()
{
    if (gyro_is_on)
    {
        do_math();

        hoja_analog_data.rs_x = fix_yaw(yaw - hold_yaw);
        hoja_analog_data.rs_y = fix_roll(roll - hold_roll);
    }
    else
    {
        hoja_analog_data.rs_x = fix_yaw(0);
        hoja_analog_data.rs_y = fix_roll(0);
    }

    if (CURENT_CONTROLLER_MODE == HOJA_CONTROLLER_MODE_NS)
    {
        hoja_analog_data.ls_x = fudge_reading(adc1_get_raw(ADC_STICK_LX), false);
        hoja_analog_data.ls_y = fudge_reading(adc1_get_raw(ADC_STICK_LY), false);
    }
    else if (CURENT_CONTROLLER_MODE == HOJA_CONTROLLER_MODE_XINPUT)
    {
        hoja_analog_data.ls_x = fudge_reading(adc1_get_raw(ADC_STICK_LX), false);
        hoja_analog_data.ls_y = fudge_reading(adc1_get_raw(ADC_STICK_LY), true);
    }
}

// Set up function to update inputs
void local_button_cb()
{
    ets_delay_us(15);

    // Read the GPIO registers and mask the data
    uint32_t regread_low = REG_READ(GPIO_IN_REG) & GPIO_INPUT_PIN_MASK;
    uint32_t regread_high = REG_READ(GPIO_IN1_REG);

    hoja_button_data.button_right = !util_getbit(regread_low, GPIO_BTN_A);
    hoja_button_data.button_down = !util_getbit(regread_low, GPIO_BTN_B);
    hoja_button_data.button_up = !util_getbit(regread_low, GPIO_BTN_X);
    hoja_button_data.button_left = !util_getbit(regread_low, GPIO_BTN_Y);

    hoja_button_data.trigger_l = !util_getbit(regread_low, GPIO_BTN_L);
    hoja_button_data.trigger_zl = !util_getbit(regread_low, GPIO_BTN_L2);
    hoja_button_data.trigger_r = !util_getbit(regread_low, GPIO_BTN_R);
    hoja_button_data.trigger_zr = !util_getbit(regread_low, GPIO_BTN_R2);
    hoja_button_data.button_stick_left = !util_getbit(regread_low, GPIO_BTN_L3);
    hoja_button_data.button_stick_right = !util_getbit(regread_low, GPIO_BTN_R3);
    hoja_analog_data.lt_a = (hoja_button_data.trigger_zl) ? DPAD_ANALOG_POS : 0;
    hoja_analog_data.rt_a = (hoja_button_data.trigger_zr) ? DPAD_ANALOG_POS : 0;

    bool new_dup = !util_getbit(regread_low, GPIO_BTN_DU);
    if (new_dup == last_dup)
        hoja_button_data.dpad_up = new_dup;
    last_dup = new_dup;

    bool new_ddown = !util_getbit(regread_low, GPIO_BTN_DD);
    if (new_ddown == last_ddown)
        hoja_button_data.dpad_down = new_ddown;
    last_ddown = new_ddown;

    bool new_dleft = !util_getbit(regread_low, GPIO_BTN_DL);
    if (new_dleft == last_dleft)
        hoja_button_data.dpad_left = new_dleft;
    last_dleft = new_dleft;

    bool new_dright = !util_getbit(regread_low, GPIO_BTN_DR);
    if (new_dright == last_dright)
        hoja_button_data.dpad_right = new_dright;
    last_dright = new_dright;

    bool new_gyro = !util_getbit(regread_high, GPIO_BTN_GYRO_ON);
    if (new_gyro != gyro_is_on) // has gyro enable state changed?
    {
        gyro_is_on = new_gyro;

        if (gyro_is_on)
        {
            do_math();
            hold_yaw = yaw;
            hold_roll = roll;
        }
        else
        {
            hoja_analog_data.rs_x = fix_yaw(0);
            hoja_analog_data.rs_y = fix_roll(0);
        }
    }

    hoja_button_data.button_start = !util_getbit(regread_high, GPIO_BTN_START);
    hoja_button_data.button_select = !util_getbit(regread_high, GPIO_BTN_SELECT);
    hoja_button_data.button_home = !util_getbit(regread_high, GPIO_BTN_HOME);
}

void local_boot_evt(hoja_boot_event_t evt)
{
    esp_err_t err;
    const char *TAG = "local_boot_evt";
    ESP_LOGI(TAG, "called local_boot_evt");

    switch (evt)
    {
    // With no battery connected
    case HEVT_BOOT_NOBATTERY:
        ESP_LOGI(TAG, "No battery detected.");
    case HEVT_BOOT_PLUGGED:
    {
        ESP_LOGI(TAG, "Plugged in on boot.");

        switch (loaded_settings.controller_mode)
        {
        case HOJA_CONTROLLER_MODE_DINPUT:
        {
            ESP_LOGI(TAG, "HOJA_CONTROLLER_MODE_DINPUT");

            CURENT_CONTROLLER_MODE = HOJA_CONTROLLER_MODE_DINPUT;
            hoja_set_core(HOJA_CORE_BT_DINPUT);

            err = hoja_start_core();
        }
        break;

        default:
        case HOJA_CONTROLLER_MODE_XINPUT:
        {
            ESP_LOGI(TAG, "HOJA_CONTROLLER_MODE_XINPUT");

            CURENT_CONTROLLER_MODE = HOJA_CONTROLLER_MODE_XINPUT;
            hoja_set_core(HOJA_CORE_BT_XINPUT);

            err = hoja_start_core();
        }
        break;

        case HOJA_CONTROLLER_MODE_NS:
        {
            ESP_LOGI(TAG, "Ninnedo switch mode config go.");

            CURENT_CONTROLLER_MODE = HOJA_CONTROLLER_MODE_NS;
            hoja_set_core(HOJA_CORE_NS);
            core_ns_set_subcore(NS_TYPE_PROCON);

            err = hoja_start_core();
        }
        break;
        }
    }
    break;

    // This case is reached if
    // the controller is unplugged but has a battery
    case HEVT_BOOT_UNPLUGGED:
    {
        ESP_LOGI(TAG, "Unplugged.");

        switch (loaded_settings.controller_mode)
        {
        case HOJA_CONTROLLER_MODE_DINPUT:
        {
            ESP_LOGI(TAG, "HOJA_CONTROLLER_MODE_DINPUT");
            err = hoja_set_core(HOJA_CORE_BT_DINPUT);

            err = hoja_start_core();

            if (err == HOJA_OK)
            {
                ESP_LOGI(TAG, "Started BT Dinput OK.");
            }
            else
            {
                ESP_LOGE(TAG, "Failed to start Dinput BT.");
            }
        }
        break;

        case HOJA_CONTROLLER_MODE_NS:
        {
            ESP_LOGI(TAG, "Nintedo switch mode config go.");
            core_ns_set_subcore(NS_TYPE_PROCON);
            err = hoja_set_core(HOJA_CORE_NS);

            err = hoja_start_core();

            if (err == HOJA_OK)
            {
                ESP_LOGI(TAG, "Started BT Switch OK.");
            }
            else
            {
                ESP_LOGE(TAG, "Failed to start Switch BT.");
            }
        }
        break;

        default:
        case HOJA_CONTROLLER_MODE_XINPUT:
        {
            ESP_LOGI(TAG, "HOJA_CONTROLLER_MODE_XINPUT");
            err = hoja_set_core(HOJA_CORE_BT_XINPUT);

            err = hoja_start_core();

            if (err == HOJA_OK)
            {
                ESP_LOGI(TAG, "Started BT XInput OK.");
            }
            else
            {
                ESP_LOGE(TAG, "Failed to start XInput BT.");
            }
        }
        break;
        }
    }
    break;
    }
    ESP_LOGI(TAG, "finished local_boot_evt");
}

// Handle System events
void local_system_evt(hoja_system_event_t evt, uint8_t param)
{
    const char *TAG = "local_system_evt";
    ESP_LOGI(TAG, "called local_system_evt");
    switch (evt)
    {
    // Called after API initialize function
    case HEVT_API_INIT_OK:
    {
        ESP_LOGI(TAG, "case HEVT_API_INIT_OK");

        local_button_cb();

        ESP_LOGI(TAG, "hoja_set_force_wired");
        hoja_set_force_wired(false);

        // Check to see what buttons are being held. Adjust state accordingly.
        if (hoja_button_data.button_left)
        {
            CURENT_CONTROLLER_MODE = HOJA_CONTROLLER_MODE_RETRO;
        }
        else if (hoja_button_data.button_right)
        {
            CURENT_CONTROLLER_MODE = HOJA_CONTROLLER_MODE_NS;
        }
        else if (hoja_button_data.button_up)
        {
            CURENT_CONTROLLER_MODE = HOJA_CONTROLLER_MODE_XINPUT;
        }
        else if (hoja_button_data.button_down)
        {
            CURENT_CONTROLLER_MODE = HOJA_CONTROLLER_MODE_DINPUT;
        }

        if (loaded_settings.controller_mode != CURENT_CONTROLLER_MODE)
        {
            loaded_settings.controller_mode = CURENT_CONTROLLER_MODE;
            hoja_settings_saveall();
        }
        local_boot_evt(HEVT_BOOT_PLUGGED);
    }
    break;

    default:
        ESP_LOGI(TAG, "Unknown event type: %d", evt);
        break;
    }
    ESP_LOGI(TAG, "finished local_system_evt");
}

// Callback to handle HOJA events
void local_event_cb(hoja_event_type_t type, uint8_t evt, uint8_t param)
{
    const char *TAG = "local_event_cb";
    ESP_LOGI(TAG, "called local_event_cb");

    switch (type)
    {
    default:
        ESP_LOGI(TAG, "Unrecognized event occurred: %X", (unsigned int)type);
        break;

    case HOJA_EVT_BOOT:
        ESP_LOGI(TAG, "HOJA_EVT_BOOT");
        local_boot_evt(evt);
        break;

    case HOJA_EVT_SYSTEM:
        ESP_LOGI(TAG, "HOJA_EVT_SYSTEM");
        local_system_evt(evt, param);
        break;

        ESP_LOGI(TAG, "finished local_event_cb");
    }
}

void init_gyro()
{
    const char *TAG = "init_gyro";
    ESP_ERROR_CHECK(i2c_param_config(icm_config.i2c_port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(icm_config.i2c_port, conf.mode, 0, 0, 0));

    /* setup ICM20948 device */
    icm20948_init_i2c(&icm, &icm_config);

    /* check ID */
    while (icm20948_check_id(&icm) != ICM_20948_STAT_OK)
    {
        ESP_LOGW(TAG, "check id failed");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "check id passed");

    /* check whoami */
    icm20948_status_e stat = ICM_20948_STAT_ERR;
    uint8_t whoami = 0x00;
    while ((stat != ICM_20948_STAT_OK) || (whoami != ICM_20948_WHOAMI))
    {
        whoami = 0x00;
        stat = icm20948_get_who_am_i(&icm, &whoami);
        ESP_LOGW(TAG, "whoami does not match (0x %d). Waiting...", whoami);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "whoami matches (0x %d). Waiting...", whoami);

    /* Here we are doing a SW reset to make sure the device starts in a known state */
    icm20948_sw_reset(&icm);
    vTaskDelay(250 / portTICK_PERIOD_MS);

    icm20948_internal_sensor_id_bm sensors = (icm20948_internal_sensor_id_bm)(ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR);

    // Set Gyro and Accelerometer to a particular sample mode
    // optiona: SAMPLE_MODE_CONTINUOUS. SAMPLE_MODE_CYCLED
    icm20948_set_sample_mode(&icm, sensors, SAMPLE_MODE_CONTINUOUS);

    // Set full scale ranges for both acc and gyr
    icm20948_fss_t myfss;
    myfss.a = GPM_2;   // (icm20948_accel_config_fs_sel_e)
    myfss.g = DPS_250; // (icm20948_gyro_config_1_fs_sel_e)
    icm20948_set_full_scale(&icm, sensors, myfss);

    // Set up DLPF configuration
    icm20948_dlpcfg_t myDLPcfg;
    myDLPcfg.a = ACC_D473BW_N499BW;
    myDLPcfg.g = GYR_D361BW4_N376BW5;
    icm20948_set_dlpf_cfg(&icm, sensors, myDLPcfg);

    // Choose whether or not to use DLPF
    icm20948_enable_dlpf(&icm, ICM_20948_INTERNAL_ACC, true);
    icm20948_enable_dlpf(&icm, ICM_20948_INTERNAL_GYR, true);

    // Now wake the sensor up
    icm20948_sleep(&icm, false);
    icm20948_low_power(&icm, false);

    /* now the fun with DMP starts */
    init_dmp(&icm);
}

void init_hoja()
{
    // IO configuration we can reuse
    gpio_config_t io_conf = {};

    // Set up IO pins for scanning button matrix
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_MASK;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);

    hoja_button_remap_enable(false);

    hoja_register_button_callback(local_button_cb);
    hoja_register_analog_callback(local_analog_cb);
    hoja_register_event_callback(local_event_cb);

    ESP_ERROR_CHECK(hoja_init());
}

void app_main()
{
    gyro_is_on = false;
    hold_yaw = 0;
    hold_roll = 0;
    init_gyro();
    init_hoja();

    UpdateReadings();
}
