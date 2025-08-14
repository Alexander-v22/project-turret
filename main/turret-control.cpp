extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "driver/ledc.h"
    #include "esp_err.h"
    #include "esp_adc/adc_oneshot.h" //needed for the joystick
    #include "esp_log.h"

    #include "nvs_flash.h"
    #include "esp_netif.h"
    #include "esp_event.h"
    #include "esp_wifi.h"
    #include "esp_http_server.h"
    #include "esp_timer.h"
    #include "secrets.h"
    #include "esp_mac.h"

}

#include <cstdint>  // For uint32_t, uint8_t
#include <cstdio>

#include <cstring>
#include <cstdlib>


// DEFINING SERVROS PAN AND TILT FOR TURRET
#define SERVO_PAN_GPIO 18
#define SERVO_TILT_GPIO 19
// NEEDED FOR THE ESP32 TO SEND SIGNALS TO TURN 
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500
// STANDARD PWM FREQUENCY
#define SERVO_FREQ_HZ 50
// ESP32 HAS A LECD TIMER NEED FOR PWM OUTPUT CHANNEL
#define SERVO_TIMER LEDC_TIMER_0 
// FOR SMOOTH AND PRECISE CONTROL 
#define SERVO_RES LEDC_TIMER_16_BIT 


// DEFINING MY PINS TO ANALOG TO DIGITIAL CONVERTERS
#define JOY_X_PIN ADC_CHANNEL_4
#define JOY_Y_PIN ADC_CHANNEL_5


//========= WIFI SETTINGS ===========

// ws server settings 
#define WS_PORT 81
#define WS_PATH "/ws"

static const char* TAG = "WIFI";


// Converts servo angle (0–180°) to LEDC duty cycle (Q16 format) based on pulse width (1000–2000 us)
uint32_t angle_to_duty_us (uint8_t angle){
    uint32_t us = SERVO_MIN_US + ((SERVO_MAX_US - SERVO_MIN_US) * angle ) / 180;
    return (us * (1<<16)) / (1000000/ SERVO_FREQ_HZ); // Uses linear interpolation to calculate pulse width for the current angle.
}

// Converting the analog joystick value(0-4095) to a servo anlges (0-180)
uint8_t map_range(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



//=========== ALL OF THESE ARE SETUPS==================

// Creating a servo SET UP to free up space in the main
static void servo_startup(uint8_t deg_pan, uint8_t deg_tilt){
    uint32_t dpan = angle_to_duty_us(deg_pan);
    uint32_t dtilt = angle_to_duty_us(deg_tilt);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, dpan); //loads the value
    ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0); // Applies the value and sends it to the servo
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, dtilt);
    ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_1);
}
// PWM SET up for servo 
static void setup_ledc(void) {
     //PWM timer on the ESP32 using the lEDC peripheral to control the frequency and resolution of the PWM singal
    ledc_timer_config_t timer = {}; 
        timer.duty_resolution = SERVO_RES;
        timer.freq_hz = SERVO_FREQ_HZ;
        timer.speed_mode = LEDC_LOW_SPEED_MODE; // not the speed of servo but the speed of the internal hardware clock 
        timer.timer_num = SERVO_TIMER;
        timer.clk_cfg = LEDC_AUTO_CLK;

    ledc_timer_config(&timer);


    //configuring the servo that controls the pan
    ledc_channel_config_t pan {};
        pan.channel = LEDC_CHANNEL_0;
        pan.duty = 0;
        pan.gpio_num = SERVO_PAN_GPIO; 
        pan.speed_mode = LEDC_LOW_SPEED_MODE;
        pan.hpoint = 0; //
        pan.timer_sel = SERVO_TIMER;

    ledc_channel_config(&pan);


    //configuring the servo that controls the the tilt
    ledc_channel_config_t tilt {};
        tilt.channel = LEDC_CHANNEL_1;
        tilt.duty = 0;
        tilt.gpio_num = SERVO_TILT_GPIO; 
        tilt.speed_mode = LEDC_LOW_SPEED_MODE;
        tilt.hpoint = 0; //
        tilt.timer_sel = SERVO_TIMER;
    ledc_channel_config(&tilt);
}


// ===== ADC oneshot state (replaces legacy driver/adc.h) =====
static adc_oneshot_unit_handle_t s_adc1 = nullptr;

// ADC SET up for joystick
static void setup_adc(void){
    // adc1_config_width(ADC_WIDTH_BIT_12); // gives us  4095 possible values the analog value can be 
    // adc1_config_channel_atten(JOY_X_PIN, ADC_ATTEN_DB_11); // Configures the volates rand the ADC can read from 
    // adc1_config_channel_atten(JOY_Y_PIN, ADC_ATTEN_DB_11);

    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc1));

    adc_oneshot_chan_cfg_t ch_cfg = {
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_DEFAULT, // 12-bit effective on ESP32
    };
    // JOY_X_PIN / JOY_Y_PIN are ADC1_CHANNEL_*; cast to adc_channel_t
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc1, (adc_channel_t)JOY_X_PIN, &ch_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc1, (adc_channel_t)JOY_Y_PIN, &ch_cfg));
}
git 

//=================WIFI SETUP==========================

static void wifi_init_sta(void) {
    ESP_ERROR_CHECK(nvs_flash_init()); //non-volitale storage needed for my wifi credentials to be stores to lvie through a reboot
    ESP_ERROR_CHECK(esp_netif_init()); // prepares the network interface e.g handles ip address, network, and connections 
    ESP_ERROR_CHECK(esp_event_loop_create_default()); // ESP32 can react to things such as wifi conected or got ip address 
    esp_netif_create_default_wifi_sta(); // creates wifi station instead of a wifi AP mode (esp32 chip acts like a router)


    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // THIS IS FILLING THE REQUIREMENTS NEED TO ACCESS THE WIFI 
    wifi_config_t wifi_config = {};
        std::snprintf((char*)wifi_config.sta.ssid, sizeof(wifi_config.sta.ssid), "%s", WIFI_SSID);
        std::snprintf((char*)wifi_config.sta.password, sizeof(wifi_config.sta.password), "%s", WIFI_PASSWORD);
        wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK ;

    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));// connects to the wifi instead of staring one 
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());
    //applyies the config, start the WIFI hardware, and attempts connections

    ESP_LOGI(TAG, "Wi-Fi STA starting… connect to %s", WIFI_SSID);

}

extern "C" void app_main(void) {

    setup_ledc();
    setup_adc();


    //MAP JOYSTICK POSITION TO SERVO ANGLE 
    //Gets the raw analog data form the joystick so that we can map it to our servos
    // int joy_x = adc1_get_raw(JOY_X_PIN);
    // int joy_y = adc1_get_raw(JOY_Y_PIN);
    int joy_x = 0;
    int joy_y = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(s_adc1, (adc_channel_t)JOY_X_PIN, &joy_x));
    ESP_ERROR_CHECK(adc_oneshot_read(s_adc1, (adc_channel_t)JOY_Y_PIN, &joy_y));

    // map_range # onto the servo
    uint8_t pan_angle = map_range(joy_x, 0 , 4095, 0 ,180);
    uint8_t tilt_angle = map_range(joy_y, 0, 4095, 0, 180);

    while(1){
        // int joy_x = adc1_get_raw(JOY_X_PIN);
        // int joy_y = adc1_get_raw(JOY_Y_PIN);
        ESP_ERROR_CHECK(adc_oneshot_read(s_adc1, (adc_channel_t)JOY_X_PIN, &joy_x));
        ESP_ERROR_CHECK(adc_oneshot_read(s_adc1, (adc_channel_t)JOY_Y_PIN, &joy_y));

        //linear mappings from the ADC range to servo angle range
        uint8_t pan_angle = map_range(joy_x, 0 , 4095, 0 ,180);
        uint8_t tilt_angle = map_range(joy_y, 0, 4095, 0, 180);

        //convers angles to PWM duty (Q16 format) and APPLy every loop to continue enidng the PWM singal to the ESP32 
        uint32_t duty_pan = angle_to_duty_us(pan_angle); //converts the angle to a PWM signal 
        uint32_t duty_tilt = angle_to_duty_us(tilt_angle);  

        servo_startup(pan_angle, tilt_angle);

        vTaskDelay(pdMS_TO_TICKS(30));
    
    }
}
