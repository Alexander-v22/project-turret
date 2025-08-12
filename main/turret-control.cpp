extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "driver/ledc.h"
    #include "esp_err.h"

}

#include <cstdint>  // For uint32_t, uint8_t
#include <cstdio>


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

// Converts servo angle (0–180°) to LEDC duty cycle (Q16 format) based on pulse width (1000–2000 us)
uint32_t angle_to_duty_us (uint8_t angle){
    uint32_t us = SERVO_MIN_US + ((SERVO_MAX_US - SERVO_MIN_US) * angle ) / 180;
    return (us * (1<<16)) / (1000000/ SERVO_FREQ_HZ); // Uses linear interpolation to calculate pulse width for the current angle.
}

extern "C" void app_main(void) {

//PWM timer on the ESP32 using the lEDC peripheral to control the frequency and resolution of the PWM singal
ledc_timer_config_t timer = {}; 
    timer.duty_resolution = SERVO_RES;
    timer.freq_hz = SERVO_FREQ_HZ;
    timer.speed_mode = LEDC_LOW_SPEED_MODE; // not the speed of servo but the speed of the internal hardware clock 
    timer.timer_num = SERVO_TIMER;
    timer.clk_cfg = LEDC_AUTO_CLK;

ledc_timer_config(&timer);


ledc_channel_config_t pan {};
    pan.channel = LEDC_CHANNEL_0;
    pan.duty = 0;
    pan.gpio_num = SERVO_PAN_GPIO; 
    pan.speed_mode = LEDC_LOW_SPEED_MODE;
    pan.hpoint = 0; //
    pan.timer_sel = SERVO_TIMER;

ledc_channel_config(&pan);



ledc_channel_config_t tilt {};
    tilt.channel = LEDC_CHANNEL_1;
    tilt.duty = 0;
    tilt.gpio_num = SERVO_TILT_GPIO; 
    tilt.speed_mode = LEDC_LOW_SPEED_MODE;
    tilt.hpoint = 0; //
    tilt.timer_sel = SERVO_TIMER;
ledc_channel_config(&tilt);



    while(1){
        for (int angle = 0; angle <= 180; angle +=45 ){
            uint32_t duty_pan = angle_to_duty_us(angle); //converts the angle to a PWM signal 
            uint32_t duty_tilt = angle_to_duty_us(angle-180);  

            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty_pan); //loads the value
            ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0); // Applies the value and sends it to the servo

            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty_tilt);
            ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_1);

            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}
    
    