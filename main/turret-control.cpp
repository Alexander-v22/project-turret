

extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "driver/ledc.h"
    #include "driver/gpio.h"
    #include "esp_err.h"
    #include "esp_adc/adc_oneshot.h" //needed for the joystick
    #include "esp_log.h"

    #include "nvs_flash.h"
    #include "esp_netif.h"
    #include "esp_event.h"
    #include "esp_wifi.h"
    #include "esp_http_server.h"
    #include "esp_timer.h"
    #include "esp_mac.h"

    #include "sdkconfig.h"


    #include "secrets.h"


}

#include <cstdint>  // For uint32_t, uint8_t
#include <cstdio>

#include <cstring>
#include <cstdlib>

#include <vector>
#include <algorithm>
#include <cmath>



// DEFINING SERVROS PAN AND TILT FOR TURRET
#define SERVO_PAN_GPIO 18
#define SERVO_TILT_GPIO 19
// NEEDED FOR THE ESP32 TO SEND SIGNALS TO TURN 
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500 // New servos went from MG90S to MG955
//Parameters that could have workd are 1000-2000

// STANDARD PWM FREQUENCY
#define SERVO_FREQ_HZ 50
// ESP32 HAS A LECD TIMER NEED FOR PWM OUTPUT CHANNEL
#define SERVO_TIMER LEDC_TIMER_0 
// FOR SMOOTH AND PRECISE CONTROL (resolution) 
#define SERVO_RES LEDC_TIMER_16_BIT 


// DEFINING MY PINS TO ANALOG TO DIGITIAL CONVERTERS
#define JOY_X_PIN ADC_CHANNEL_4
#define JOY_Y_PIN ADC_CHANNEL_5


//DEFINING PUSH BUTTONS GPIO PINS
#define BLUE_BTN GPIO_NUM_35
#define RED_BTN GPIO_NUM_34

//DEFINIG LED SIGNIFIERS FOR BTNS
#define RED_LED_GPIO GPIO_NUM_23
#define BLUE_LED_GPIO GPIO_NUM_22



//========= WIFI/WS and HTTP server ===========

// ws server settings 
#define WS_PORT 81
#define WS_PATH "/ws"
static const char* TAG_WIFI = "WIFI_SETUP";
static const char *TAG_WS = "ws";
static const char *TAG_MODE = "MODE: ";


//========== YAW AND PITCH ==========

#define MAX_YAW 90.0
#define MAX_PITCH 45.0


typedef enum control_mode_t { CONTROL_MODE_JOYSTICK, CONTROL_MODE_WS};
// Want joystick mode to be our defualt mode  
static volatile control_mode_t g_mode = CONTROL_MODE_JOYSTICK;

//Need a fall back if WS control fails
static constexpr uint32_t WS_TIMEOUT_MS = 3000;

//90 degrees form the center
static volatile uint8_t g_pan_angle = 90;
static volatile uint8_t  g_tilt_angle = 90;

//
static volatile uint32_t  g_last_ws_ms = 0;


// Map angle -> pulse_us (500–2500 us at 50 Hz -> 20ms), then convert to LEDC duty counts:

// duty = (pulse_us / 20,000 us) * 2^16  // servos respond to pulse width, not raw duty %
// but esp32's "lang" is in duty cycles 
uint32_t angle_to_ledc_counts (uint8_t angle){
    uint32_t us = SERVO_MIN_US + ((SERVO_MAX_US - SERVO_MIN_US) * angle ) / 180;
    return (us * (1<<16)) / (1000000/ SERVO_FREQ_HZ); // Uses linear interpolation to calculate pulse width for the current angle.
}

// Converting the analog joystick value(0-4095) to a servo anlges (0-180)
uint8_t map_range(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Map for floating point numbers recieved from the front end 
double map_d_range(double x, double in_min, double in_max, double out_min, double out_max) {
    if (x < in_min) x = in_min;
    if (x > in_max) x = in_max;

    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


// Need a ws sever refresher so that we can create a timeout if the WS connection fails 
static inline uint32_t millis (){
    return (uint32_t)(esp_timer_get_time()/1000ULL);
}
 
//=========== ALL OF THESE ARE SETUPS FOR SERVO USE==================

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

static void servo_update(uint8_t deg_pan, uint8_t deg_tilt){
    uint32_t dpan = angle_to_ledc_counts(deg_pan); // these are the "Angles" of the servo = deg_tilt = Angle  then do math
    uint32_t dtilt = angle_to_ledc_counts(deg_tilt);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, dpan); //loads the value
    ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0); // Applies the value and sends it to the servo
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, dtilt);
    ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_1);
}


// ===== ADC oneshot state (replaces legacy driver/adc.h) =====
static adc_oneshot_unit_handle_t s_adc1 = nullptr;

static void setup_adc(void){


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

//=========MODIFIED WEBSOCKET SERVER HELPER

static bool json_get_number (const char* s, const char* key, double* out ){
    const char* p  = std :: strstr(s, key); // prevents re-scanning by keeping a pointer onto the original buffer (Bookmark)
    if (!p) return false; // Fail safe: early return ensuring the key is present to continue to *out
    p = std::strchr(p, ':'); // won't match colon before key's location 
    if(!p) return false; // Fail safe: early return again to make sure after the key text there is a colon 

    char* endp = nullptr; // needed for std::strtod, this way it puts a place "where parsing stopped"
    double v = std::strtod(p + 1, &endp); // (p + 1) skips colon and v is declared to ensure we dont touch *out unless parsing good 
    if (endp == p + 1 )return false; // Fail safe: for every strtod controct if no conversion happend then endp is set to the same pointer 
    *out = v;// finally setting out* to the value 
    return true ; 
}

//=====================WIFI SETUP==========================
static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
        printf("WiFi connecting ... \n");
        break;
    case WIFI_EVENT_STA_CONNECTED:
        printf("WiFi connected ... \n");
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
        printf("WiFi lost connection ... \n");
        break;
    case IP_EVENT_STA_GOT_IP:
        printf("Retrieved IP ... \n\n");
        break;
    default:
        break;
    }
}

static void wifi_init_sta(void) { 
    // 1. Wi-Fi/LwIP initiation phase 

    ESP_ERROR_CHECK(nvs_flash_init()); //non-volitale storage needed for my wifi credentials to be stores to live through a reboot
    ESP_ERROR_CHECK(esp_netif_init());  //prepares the network interface e.g handles ip address, network, and connections (A.K.A TCP/IP stack)
    ESP_ERROR_CHECK(esp_event_loop_create_default()); //  delivers WIFI_EVENT / IP_EVENT callbaks: event loop

    /* Creates the default Wi-Fi station (STA) network interface and bind it
   to the Wi-Fi driver. */
    esp_netif_create_default_wifi_sta(); 

    // initailizes the wifi driver with default configuration 
    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_initiation));

    // 2. Wi-Fi Configuartion phase

    /* Fill in STA credentials and policy. Copy SSID/password into the config struct. Sets a minimum authmode (rejects open/WEP APs) */
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);
    wifi_config_t wifi_config = {};
        std::snprintf((char*)wifi_config.sta.ssid, sizeof(wifi_config.sta.ssid), "%s", WIFI_SSID);
        std::snprintf((char*)wifi_config.sta.password, sizeof(wifi_config.sta.password), "%s", WIFI_PASSWORD);
        wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK ;
    
    /* Puts the driver in STA(client) mode and applies the config we coded */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));// connects to the wifi instead of staring one 
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    //3. start phase 
    ESP_ERROR_CHECK(esp_wifi_start()); 

    //4. wifi connect phase
    ESP_ERROR_CHECK(esp_wifi_connect()); 

    ESP_LOGI(TAG_WIFI, "Wi-Fi STA starting… connect to %s", WIFI_SSID);
}


//==========SETTING UP AN HTTP SERVER=============
static httpd_handle_t g_server = nullptr; // keeps a global handle to the HTTP server instance which is needed so we can later stop the server or register more endpoints
static esp_err_t ws_handler(httpd_req_t* req); //handler funciton that will process WS events (messages)

static void start_ws_server(){

    httpd_config_t httpd_server_config = HTTPD_DEFAULT_CONFIG(); // loads a default HTTP server config 
    httpd_server_config.server_port = 81; // Runs the HTTP server on port 81

    ESP_ERROR_CHECK(httpd_start(&g_server, &httpd_server_config)); 
    //starts a tiny web server on the ESP32 
    // WebSockets run on top of HTTP. You need the HTTP server alive before any WS connection can exist 


    // This is simply a URI Path like a handler 
    static httpd_uri_t ws_uri = {};
        ws_uri.uri = "/ws" ;            // assigns the URI path to the handler
        ws_uri.method = HTTP_GET;       // WebSocket upgrade begins with a GET request
        ws_uri.handler = ws_handler;    // Function to handle WebSocket events (ws-handler)
        ws_uri.user_ctx = nullptr;      // Sets the user context data to nullptr
        ws_uri.is_websocket = true;     // Very important, this indicates that this URI handler is a for a WebScoket connection

        // register the WS URI handler with the server
        ESP_ERROR_CHECK(httpd_register_uri_handler(g_server,&ws_uri));
}


//===========CREATED A WEBSOCKET JSON===========
// ws_handler invoked when ESP32 chips attempts to connect to the WebSocket URI 
static esp_err_t ws_handler(httpd_req_t* req){

    //  Responsible for signifying the intial HTTP request for a connection upgrade 
    if (req -> method == HTTP_GET){
        ESP_LOGI(TAG_WS, "WS Handshake");
        return ESP_OK; // handshake done by server 
    }

    httpd_ws_frame_t frame = {} ; // A httpd_ws_frame_t structure is initialized to hold information about the incoming WebSocket frame.
    frame.type = HTTPD_WS_TYPE_TEXT; // Payload will be interpreted as UTF-8 text (text data), since were using JSON. "OPCODE"

    /* 1. Probes the length of the incoming frame and only fill the frame.len with the actaul length
    of the WebSokcet Frame without recieving the actual payload data yet */ 

    ESP_ERROR_CHECK(httpd_ws_recv_frame(req, &frame, 0)); //httpd_ws_recv_frame "Probes" the incoming frame's 
    if (frame.len == 0 ) return ESP_OK; // if frame.len == 0 then no data in the frame
    

    /* 2. reads the payload safely into a fixed buffer */
    // break this down
    std::vector<uint8_t> buf(frame.len + 1);
    frame.payload = buf.data();
    ESP_ERROR_CHECK(httpd_ws_recv_frame(req, &frame,frame.len));
    buf[frame.len] = 0;
    g_last_ws_ms = millis();

    if (g_mode != CONTROL_MODE_WS){
        return ESP_OK;
    }

    const char * msg = reinterpret_cast<const char*>(buf.data());
    ESP_LOGI(TAG_WS, "msg=%s", msg);


    double yaw_degrees = 0.0;
    double pitch_degrees = 0.0; 

    const bool has_yaw =json_get_number(msg,  "yaw", &yaw_degrees);
    const bool has_pitch =json_get_number(msg,  "pitch", &pitch_degrees);


    if (has_yaw && has_pitch){
        const double pan_yaw = map_d_range(yaw_degrees, -MAX_YAW, +MAX_YAW, 0.0, 180);
        const double tilt_pitch = map_d_range(pitch_degrees, -MAX_PITCH, +MAX_PITCH, 180, 0);

        int pan = static_cast<int>(pan_yaw + 0.5);
        int tilt = static_cast<int>(tilt_pitch + 0.5);

        pan = std::clamp(pan, 0, 160); 
        tilt = std::clamp(tilt, 0 ,160);

        g_pan_angle = static_cast<uint8_t>(pan);
        g_tilt_angle = static_cast<uint8_t>(tilt);
        g_mode = CONTROL_MODE_WS; 
        g_last_ws_ms = millis();
    
        ESP_LOGI(TAG_WS, "has_yaw=%d, has_pitch=%d, yaw=%.2f, pitch=%.2f", has_yaw, has_pitch, yaw_degrees, pitch_degrees);
    }

    return ESP_OK;
}

static int g_joy_center_x = 2048;
static int g_joy_center_y = 2048;

static float ema(float x, float& s, float a){
    s = a*x + (1 - a)*s;
    return s;
}

static float apply_deadzone_norm(float v, float dz){
    float abs_v = fabsf(v);
    if (abs_v < dz ) return 0.0f; // output 0 (deadzone)
    float sign  = (v > 0) - (v < 0); // yields (-1, 0, 1)
    float out = (abs_v - dz)/ (1.0f -dz); // creating a continuous mapping at the deadzone
    out = powf(out, 1.5f); // tames the first chunk of travel so we get micro-aim neer center (response curve)
    return sign*out; 
}

extern "C" void app_main(void) {

    setup_ledc();
    setup_adc();

    wifi_init_sta();
    start_ws_server();

    gpio_set_direction(RED_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(RED_BTN, GPIO_MODE_INPUT);

    gpio_set_direction(BLUE_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(BLUE_BTN, GPIO_MODE_INPUT);

  
    int joy_x = 0;
    int joy_y = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(s_adc1, (adc_channel_t)JOY_X_PIN, &joy_x));
    ESP_ERROR_CHECK(adc_oneshot_read(s_adc1, (adc_channel_t)JOY_Y_PIN, &joy_y));



    static float x_filter = 0;
    static float y_filter = 0;
    const float EMA_A = 0.25f;            // 0..1, lower = smoother another smoothing factor 
    const float DZ = 0.05f;               // 8% deadzone
        
    while(1){
  
        // read raw ADC outputs
        ESP_ERROR_CHECK(adc_oneshot_read(s_adc1, (adc_channel_t)JOY_X_PIN, &joy_x));
        ESP_ERROR_CHECK(adc_oneshot_read(s_adc1, (adc_channel_t)JOY_Y_PIN, &joy_y));

        // This firt clacs. a range from [-2, 2] but then we clamp it between [-1.2, 1.2] in the next line 
        float x_normalized = (joy_x - g_joy_center_x) / 2048.0f;
        float y_normalized = (joy_y - g_joy_center_y) / 2048.0f;
        // Then clamp around [-1.2, 1.2] changed from 1 -> 1.2 for a small errors
        x_normalized = std::clamp(x_normalized, -1.2f, 1.2f);
        y_normalized = std::clamp(y_normalized, -1.2f, 1.2f);

        //low pass EMA to reduce jitter
        x_normalized = ema(x_normalized, x_filter, EMA_A);
        y_normalized = ema(y_normalized, y_filter, EMA_A);

        //Zero out smaoll motions and apply a gentle curve to the rest 
        x_normalized = apply_deadzone_norm(x_normalized, DZ);        
        y_normalized = apply_deadzone_norm(y_normalized, DZ);

        //linear mappings from the ADC range to servo angle range
        uint8_t pan_angle_js  = (uint8_t)std::lround( ( -x_normalized * 0.5f + 0.5f ) * 175.0f );
        uint8_t tilt_angle_js = (uint8_t)std::lround( (  y_normalized * 0.5f + 0.5f ) * 175.0f );

        uint32_t now = millis();

         if(gpio_get_level(RED_BTN) == 1){
            gpio_set_level(RED_LED_GPIO, 1);
            gpio_set_level(BLUE_LED_GPIO, 0);
            
            g_mode = CONTROL_MODE_JOYSTICK ;

            ESP_LOGW( TAG_MODE, "JOYSTICK");
            

        } else if (gpio_get_level(BLUE_BTN) == 1 ){
            gpio_set_level(BLUE_LED_GPIO, 1);
            gpio_set_level(RED_LED_GPIO, 0);

            g_mode = CONTROL_MODE_WS;
            ESP_LOGW( TAG_MODE, "HEAD CONTROL");
        } 

        if(g_mode == CONTROL_MODE_WS && (now - g_last_ws_ms)> WS_TIMEOUT_MS) {
            g_mode = CONTROL_MODE_JOYSTICK;
            ESP_LOGW(TAG_WS,"WS Timeout switching into JOYSTICK MODE");
            gpio_set_level(RED_LED_GPIO, 1);
            gpio_set_level(BLUE_LED_GPIO, 0);
        }


        uint8_t pan_angle  = (g_mode == CONTROL_MODE_WS) ? g_pan_angle  : pan_angle_js;
        uint8_t tilt_angle = (g_mode == CONTROL_MODE_WS) ? g_tilt_angle : tilt_angle_js;

        servo_update(pan_angle, tilt_angle);

        vTaskDelay(pdMS_TO_TICKS(30));// servos accept up to 50Hz PWM but we dont need to recompute angles that fast so we use 33hz to save CPU 
        // increasing this will allows to reduce lag without extra chatter"
    
    }
}