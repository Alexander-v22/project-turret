extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "driver/ledc.h"
    #include "esp_err.h"
    #include "driver/adc.h" //needed for the joystick

   
   
}

#include <cstdint>  // For uint32_t, uint8_t
#include <cstdio>

// DEFINING SERVROS PAN AND TILT FOR TURRET
#define SERVO_PAN_GPIO 18
#define SERVO_TILT_GPIO 19
#define JOY_X_PIN ADC1_CHANNEL_4
#define JOY_Y_PIN ADC1_CHANNEL_5


// NEEDED FOR THE ESP32 TO SEND SIGNALS TO TURN 
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500 

// STANDARD PWM FREQUENCY
#define SERVO_FREQ_HZ 50

// ESP32 HAS A LECD TIMER NEED FOR PWM OUTPUT CHANNEL
#define SERVO_TIMER LEDC_TIMER_0 

// FOR SMOOTH AND PRECISE CONTROL 
#define SERVO_RES LEDC_TIMER_16_BIT 

// Converts servo angle (0–180°) to LEDC duty cycle (Q16 format) based on pulse width (1000–2000 us)
uint32_t angle_to_duty_us (uint8_t angle){
    uint32_t us = SERVO_MIN_US + ((SERVO_MAX_US - SERVO_MIN_US) * angle ) / 180;
    return (us * (1<<16)) / (1000000/ SERVO_FREQ_HZ); // Uses linear interpolation to calculate pulse width for the current angle.
}

// Converting the analog joystick value(0-4095) to a servo anlges (0-180)
uint8_t map_range(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

extern "C" void app_main(void) {

adc1_config_width(ADC_WIDTH_BIT_12); // gives us  4095 possible values the analog value can be 
adc1_config_channel_atten(JOY_X_PIN, ADC_ATTEN_DB_11); // Configures the volates rand the ADC can read from 
adc1_config_channel_atten(JOY_Y_PIN, ADC_ATTEN_DB_11);

//MAP JOYSTICK POSITION TO SERVO ANGLE 

//Gets the raw analog data form the joystick so that we can map it to our servos
int joy_x = adc1_get_raw(JOY_X_PIN);
int joy_y = adc1_get_raw(JOY_Y_PIN);

// map_rangeping # onto the servo
uint8_t pan_angle = map_range(joy_x, 0 , 4095, 0 ,180);
uint8_t tilt_angle = map_range(joy_y, 0, 4095, 0, 180);


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



    while(1){

        int joy_x = adc1_get_raw(JOY_X_PIN);
        int joy_y = adc1_get_raw(JOY_Y_PIN);

        //linear mappings from the ADC range to servo angle range
        uint8_t pan_angle = map_range(joy_x, 0 , 4095, 0 ,180);
        uint8_t tilt_angle = map_range(joy_y, 0, 4095, 0, 180);

        //convers angles to PWM duty (Q16 format) and APPLy every loop to continue enidng the PWM singal to the ESP32 
        uint32_t duty_pan = angle_to_duty_us(pan_angle); //converts the angle to a PWM signal 
        uint32_t duty_tilt = angle_to_duty_us(tilt_angle);  

        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty_pan); //loads the value
        ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0); // Applies the value and sends it to the servo

        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty_tilt);
        ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_1);

        vTaskDelay(pdMS_TO_TICKS(30));
    
    }
}
    
    