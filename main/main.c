#include "main.h"

#define I2C_MASTER_SCL_IO GPIO_NUM_19 /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO GPIO_NUM_18 /*!< GPIO number used for I2C master data  */

uint8_t i2cData[14];
#define RAD_TO_DEG 57.29578

int16_t ax, ay, az;
int16_t gx, gy, gz;

// vvvvvvvvvvvvvvvvvv  VERY VERY IMPORTANT vvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// These are the previously determined offsets and scale factors for accelerometer and gyro for
// a particular example of an MPU-6500. They are not correct for other examples.
// The IMU code will NOT work well or at all if these are not correct

float A_cal[6] = {5652.0, 5993.0, 8202.0, 1.000, 1.000, 1.000}; // 0..2 offset xyz, 3..5 scale xyz

float G_off[3] = {108.0, 9.0, 91.0};               // raw offsets, determined for gyro at rest
#define gscale ((250. / 32768.0) * (M_PI / 180.0)) // gyro default 250 LSB per d/s -> rad/s

// ^^^^^^^^^^^^^^^^^^^ VERY IMPORTANT ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

static float deltat = 0;                // loop time in seconds
static unsigned long now = 0, last = 0; // micros() timers

// GLOBALLY DECLARED, required for Mahony filter
// vector to hold quaternion
float q[4] = {1.0, 0.0, 0.0, 0.0};

// Free parameters in the Mahony filter and fusion scheme,
// Kp for proportional feedback, Ki for integral
float Kp = 20.0;
float Ki = 0.0;

// globals for AHRS loop timing
unsigned long now_ms, last_ms = 0; // millis() timers

float yaw, pitch, roll; // Euler angle output

#define MPU6500_SENSOR_ADDR 0x68 /*!< Slave address of the MPU6500 sensor */
#define MPU6500_PWR_MGMT_1_REG 0x6B
#define MPU6500_WHO_AM_I_REG 0x75
#define MPU6500_READING_DATA_REG 0x3B
#define MPU6500_USER_CTL_REG 0x6A

#define SAMPLE_PERIOD_MS 100

#define GPIO_BTN_A GPIO_NUM_23
#define GPIO_BTN_B GPIO_NUM_22
#define GPIO_BTN_X GPIO_NUM_17
#define GPIO_BTN_Y GPIO_NUM_16

#define GPIO_BTN_L  GPIO_NUM_14
#define GPIO_BTN_L2 GPIO_NUM_3
#define GPIO_BTN_R  GPIO_NUM_15
#define GPIO_BTN_R2 GPIO_NUM_4
#define GPIO_BTN_L3 GPIO_NUM_5
#define GPIO_BTN_R3 GPIO_NUM_12

#define GPIO_BTN_DU GPIO_NUM_25
#define GPIO_BTN_DD GPIO_NUM_13
#define GPIO_BTN_DL GPIO_NUM_27
#define GPIO_BTN_DR GPIO_NUM_26

#define GPIO_BTN_GYRO_ON GPIO_NUM_34
#define GPIO_BTN_START   GPIO_NUM_36
#define GPIO_BTN_SELECT  GPIO_NUM_39
#define GPIO_BTN_HOME    GPIO_NUM_35

#define ADC_STICK_LX ADC1_CHANNEL_4 /*!< ADC1 channel 4 is GPIO32 */
#define ADC_STICK_LY ADC1_CHANNEL_5 /*!< ADC1 channel 5 is GPIO33 */
// #define ADC_STICK_RX     ADC1_CHANNEL_6
// #define ADC_STICK_RY     ADC1_CHANNEL_7

#define GPIO_INPUT_PIN_MASK ((1ULL << GPIO_BTN_A) | (1ULL << GPIO_BTN_B) | (1ULL << GPIO_BTN_X) | (1ULL << GPIO_BTN_Y) | (1ULL << GPIO_BTN_L) | (1ULL << GPIO_BTN_L2) | (1ULL << GPIO_BTN_R) | (1ULL << GPIO_BTN_R2) | (1ULL << GPIO_BTN_L3) | (1ULL << GPIO_BTN_R3) | (1ULL << GPIO_BTN_DU) | (1ULL << GPIO_BTN_DD) | (1ULL << GPIO_BTN_DL) | (1ULL << GPIO_BTN_DR) | (1ULL << GPIO_BTN_GYRO_ON) | (1ULL << GPIO_BTN_START) | (1ULL << GPIO_BTN_SELECT) | (1ULL << GPIO_BTN_HOME))

hoja_controller_mode_t CURENT_CONTROLLER_MODE;

bool gyro_is_on;

//--------------------------------------------------------------------------------------------------
// Mahony scheme uses proportional and integral filtering on
// the error between estimated reference vector (gravity) and measured one.
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date      Author      Notes
// 29/09/2011 SOH Madgwick    Initial release
// 02/10/2011 SOH Madgwick  Optimised for reduced CPU load
// last update 07/09/2020 SJR minor edits
//--------------------------------------------------------------------------------------------------
// IMU algorithm update

void Mahony_update(float ax, float ay, float az, float gx, float gy, float gz, float deltat)
{
    float recipNorm;
    float vx, vy, vz;
    float ex, ey, ez; // error terms
    float qa, qb, qc;
    static float ix = 0.0, iy = 0.0, iz = 0.0; // integral feedback terms
    float tmp;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    tmp = ax * ax + ay * ay + az * az;
    if (tmp > 0.0)
    {

        // Normalise accelerometer (assumed to measure the direction of gravity in body frame)
        recipNorm = 1.0 / sqrt(tmp);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity in the body frame (factor of two divided out)
        vx = q[1] * q[3] - q[0] * q[2]; // to normalize these terms, multiply each by 2.0
        vy = q[0] * q[1] + q[2] * q[3];
        vz = q[0] * q[0] - 0.5f + q[3] * q[3];

        // Error is cross product between estimated and measured direction of gravity in body frame
        // (half the actual magnitude)
        ex = (ay * vz - az * vy);
        ey = (az * vx - ax * vz);
        ez = (ax * vy - ay * vx);

        // Compute and apply to gyro term the integral feedback, if enabled
        if (Ki > 0.0f)
        {
            ix += Ki * ex * deltat; // integral error scaled by Ki
            iy += Ki * ey * deltat;
            iz += Ki * ez * deltat;
            gx += ix; // apply integral feedback
            gy += iy;
            gz += iz;
        }

        // Apply proportional feedback to gyro term
        gx += Kp * ex;
        gy += Kp * ey;
        gz += Kp * ez;
    }

    // Integrate rate of change of quaternion, q cross gyro term
    deltat = 0.5 * deltat;
    gx *= deltat; // pre-multiply common factors
    gy *= deltat;
    gz *= deltat;
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);

    // renormalise quaternion
    recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] = q[0] * recipNorm;
    q[1] = q[1] * recipNorm;
    q[2] = q[2] * recipNorm;
    q[3] = q[3] * recipNorm;
}

static esp_err_t MPU6500_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6500_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t MPU6500_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU6500_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}

static esp_err_t i2c_master_init(void)
{
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

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void reset_gyro()
{
    MPU6500_register_write_byte(MPU6500_PWR_MGMT_1_REG, 0x80); // reset config
    vTaskDelay(1 * pdMS_TO_TICKS(SAMPLE_PERIOD_MS));

    i2cData[0] = 7;    // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
    i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
    i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
    i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g

    MPU6500_register_write_byte(0x19, *i2cData);               // set SMPLRT_DIV
    MPU6500_register_write_byte(MPU6500_PWR_MGMT_1_REG, 0x01); // PLL with X axis gyroscope reference and disable sleep mode
}

void UpdateReadings()
{
    MPU6500_register_read(MPU6500_READING_DATA_REG, i2cData, 14);

    ax = ((i2cData[0] << 8) | i2cData[1]);
    ay = ((i2cData[2] << 8) | i2cData[3]);
    az = ((i2cData[4] << 8) | i2cData[5]);
    gx = ((i2cData[8] << 8) | i2cData[9]);
    gy = ((i2cData[10] << 8) | i2cData[11]);
    gz = ((i2cData[12] << 8) | i2cData[13]);
}

void do_math()
{
    float Axyz[3];
    float Gxyz[3];

    Axyz[0] = (float)ax;
    Axyz[1] = (float)ay;
    Axyz[2] = (float)az;

    // apply offsets and scale factors from Magneto
    for (int i = 0; i < 3; i++)
        Axyz[i] = (Axyz[i] - A_cal[i]) * A_cal[i + 3];

    Gxyz[0] = ((float)gx - G_off[0]) * gscale; // 250 LSB(d/s) default to radians/s
    Gxyz[1] = ((float)gy - G_off[1]) * gscale;
    Gxyz[2] = ((float)gz - G_off[2]) * gscale;

    now = esp_timer_get_time();
    deltat = (now - last) * 1.0e-6; // seconds since last update
    last = now;
    Mahony_update(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], deltat);

    // Compute Tait-Bryan angles. Strictly valid only for approximately level movement

    // In this coordinate system, the positive z-axis is up, X north, Y west.
    // Yaw is the angle between Sensor x-axis and Earth magnetic North
    // (or true North if corrected for local declination, looking down on the sensor
    // positive yaw is counterclockwise, which is not conventional for NED navigation.
    // Pitch is angle between sensor x-axis and Earth ground plane, toward the
    // Earth is positive, up toward the sky is negative. Roll is angle between
    // sensor y-axis and Earth ground plane, y-axis up is positive roll. These
    // arise from the definition of the homogeneous rotation matrix constructed
    // from quaternions. Tait-Bryan angles as well as Euler angles are
    // non-commutative; that is, the get the correct orientation the rotations
    // must be applied in the correct order which for this configuration is yaw,
    // pitch, and then roll.
    // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    // which has additional links.

    // WARNING: This angular conversion is for DEMONSTRATION PURPOSES ONLY. It WILL
    // MALFUNCTION for certain combinations of angles! See https://en.wikipedia.org/wiki/Gimbal_lock

    roll = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
    pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
    // conventional yaw increases clockwise from North. Not that the MPU-6500 knows where North is.
    yaw = -atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - (q[2] * q[2] + q[3] * q[3]));
    // to degrees
    yaw *= 180.0 / M_PI;
    if (yaw < 0)
        yaw += 360.0; // compass circle
    // if (yaw >= 360)
    //     yaw -= 360.0; // compass circle
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

uint16_t fix_yaw(float yaaw)
{
    int deadzone = 5;
    if ((-deadzone < yaaw) && (yaaw < deadzone))
    {
        yaaw = 0;
    }

    // if (yaaw > 89)
    // {
    //     yaaw = 89;
    // }
    // if (yaaw < -89)
    // {
    //     yaaw = -89;
    // }

    float rotate = fmod((yaaw + 180), 360.0); // rotate so 180 is midpoint
    // float rotate = yaaw;

    return 10 * (uint16_t)rotate;
}

uint16_t fix_roll(float rollll)
{
    int deadzone = 5;
    if ((-deadzone < rollll) && (rollll < deadzone))
    {
        rollll = 0;
    }

    if (rollll > 89)
    {
        rollll = 89;
    }
    if (rollll < -89)
    {
        rollll = -89;
    }

    float rotate = fmod((rollll + 180), 360.0); // rotate so 180 is midpoint

    return 10 * (uint16_t)rotate;
}

// Separate task to read sticks.
// This is essential to have as a separate component as ADC scans typically take more time and this is only
// scanned once between each polling interval. This varies from core to core.
void local_analog_cb()
{
    if (gyro_is_on)
    {
        UpdateReadings();
        do_math();

        // read stick 1
        hoja_analog_data.rs_x = fix_yaw(yaw);
        hoja_analog_data.rs_y = fix_roll(roll);
    }
    else
    {
        hoja_analog_data.rs_x = fix_yaw(0);
        hoja_analog_data.rs_y = fix_roll(0);
    }

    if (CURENT_CONTROLLER_MODE == HOJA_CONTROLLER_MODE_NS)
    {
        hoja_analog_data.ls_x = fudge_reading(adc1_get_raw(ADC_STICK_LX), true);
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

    hoja_button_data.trigger_l  = !util_getbit(regread_low, GPIO_BTN_L);
    hoja_button_data.trigger_zl = !util_getbit(regread_low, GPIO_BTN_L2);
    hoja_button_data.trigger_r  = !util_getbit(regread_low, GPIO_BTN_R);
    hoja_button_data.trigger_zr = !util_getbit(regread_low, GPIO_BTN_R2);
    hoja_button_data.button_stick_left  = !util_getbit(regread_low, GPIO_BTN_L3);
    hoja_button_data.button_stick_right = !util_getbit(regread_low, GPIO_BTN_R3);
    hoja_analog_data.lt_a = (hoja_button_data.trigger_zl) ? DPAD_ANALOG_POS : 0;
    hoja_analog_data.rt_a = (hoja_button_data.trigger_zr) ? DPAD_ANALOG_POS : 0;

    hoja_button_data.dpad_up = !util_getbit(regread_low, GPIO_BTN_DU);
    hoja_button_data.dpad_down = !util_getbit(regread_low, GPIO_BTN_DD);
    hoja_button_data.dpad_left = !util_getbit(regread_low, GPIO_BTN_DL);
    hoja_button_data.dpad_right = !util_getbit(regread_low, GPIO_BTN_DR);

    bool new_gyro = !util_getbit(regread_high, GPIO_BTN_GYRO_ON);
    if (new_gyro != gyro_is_on)
    {
        last = esp_timer_get_time();
        gyro_is_on = new_gyro;

        if (gyro_is_on)
        {
            UpdateReadings();
            do_math();
        }
        else
        {
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
            if (err == HOJA_OK)
            {
            }
            else
            {
            }
        }
        break;

        default:
        case HOJA_CONTROLLER_MODE_XINPUT:
        {
            ESP_LOGI(TAG, "HOJA_CONTROLLER_MODE_XINPUT");

            CURENT_CONTROLLER_MODE = HOJA_CONTROLLER_MODE_XINPUT;
            hoja_set_core(HOJA_CORE_BT_XINPUT);

            err = hoja_start_core();
            if (err == HOJA_OK)
            {
            }
            else
            {
            }
        }
        break;

        case HOJA_CONTROLLER_MODE_NS:
        {
            ESP_LOGI(TAG, "Ninnedo switch mode config go.");

            CURENT_CONTROLLER_MODE = HOJA_CONTROLLER_MODE_NS;
            hoja_set_core(HOJA_CORE_NS);
            core_ns_set_subcore(NS_TYPE_PROCON);

            err = hoja_start_core();
            if (err == HOJA_OK)
            {
            }
            else
            {
            }
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
    // util_battery_boot_status();
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

void local_bt_evt(hoja_bt_event_t evt)
{
    const char *TAG = "local_bt_evt";

    switch (evt)
    {
    default:
        ESP_LOGI(TAG, "Unknown bt event");
        break;

    case HEVT_BT_STARTED:
        ESP_LOGI(TAG, "BT Started OK.");
        break;

    case HEVT_BT_CONNECTING:
        ESP_LOGI(TAG, "Connecting BT...");
        break;

    case HEVT_BT_PAIRING:
        ESP_LOGI(TAG, "Pairing BT Device.");
        break;

    case HEVT_BT_CONNECTED:
        ESP_LOGI(TAG, "BT Device Connected.");
        break;

    case HEVT_BT_DISCONNECTED:
        ESP_LOGI(TAG, "BT Device Disconnected.");

        break;
    }
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

    case HOJA_EVT_BT:
        ESP_LOGI(TAG, "HOJA_EVT_BT");
        local_bt_evt(evt);
        break;
    }
    ESP_LOGI(TAG, "finished local_event_cb");
}

void init_gyro()
{
    ESP_ERROR_CHECK(i2c_master_init());

    /* Read the MPU6500 WHO_AM_I register, on power up the register should have the value 0x70 */
    ESP_ERROR_CHECK(MPU6500_register_read(MPU6500_WHO_AM_I_REG, i2cData, 1));

    reset_gyro();
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

    hoja_register_button_callback(local_button_cb);
    hoja_register_analog_callback(local_analog_cb);
    hoja_register_event_callback(local_event_cb);

    ESP_ERROR_CHECK(hoja_init());

    hoja_analog_data.rs_x = fix_yaw(0);
    hoja_analog_data.rs_y = fix_roll(0);
}

void app_main()
{
    gyro_is_on = false;
    init_gyro();
    init_hoja();

    for (int i = 0; 1 < 100; i++)
    {
        UpdateReadings();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    do_math();
}
