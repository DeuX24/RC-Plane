#include "joystick_calib.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

joystick_calib_t joy_calib = {0}; 

void init_joystick_calibration(adc_oneshot_unit_handle_t adc_handle, bool force_calibration, void (*led_callback)(uint32_t, uint32_t, uint32_t)) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE("CALIB", "Error opening NVS");
        return;
    }

    size_t required_size = sizeof(joystick_calib_t);
    err = nvs_get_blob(my_handle, "calib_data", &joy_calib, &required_size);

    if (err == ESP_ERR_NVS_NOT_FOUND || force_calibration || !joy_calib.is_calibrated) {
        ESP_LOGI("CALIB", "Starting Calibration Sequence in 3 seconds...");
        if(led_callback) led_callback(0, 0, 50); // BLUE
        vTaskDelay(pdMS_TO_TICKS(3000));
        
        ESP_LOGI("CALIB", "Do not touch the sticks. Finding true center...");

        int raw_p, raw_y, raw_r, raw_t;
        
        long sum_p = 0, sum_y = 0, sum_r = 0, sum_t = 0;
        int num_samples = 50;

        for (int i = 0; i < num_samples; i++) {
            adc_oneshot_read(adc_handle, JOY_PITCH_CHAN, &raw_p); 
            adc_oneshot_read(adc_handle, JOY_YAW_CHAN, &raw_y); 
            adc_oneshot_read(adc_handle, JOY_ROLL_CHAN, &raw_r); 
            adc_oneshot_read(adc_handle, JOY_THROTTLE_CHAN, &raw_t); 
            
            sum_p += raw_p;
            sum_y += raw_y;
            sum_r += raw_r;
            sum_t += raw_t;
            
            vTaskDelay(pdMS_TO_TICKS(10)); // 10ms delay between reads (0.5 seconds total)
        }
        
        // Calculate the true average center
        int center_p = sum_p / num_samples;
        int center_y = sum_y / num_samples;
        int center_r = sum_r / num_samples;
        int center_t = sum_t / num_samples;

        joy_calib.pitch_center = center_p; joy_calib.pitch_min = center_p; joy_calib.pitch_max = center_p;
        joy_calib.yaw_center   = center_y; joy_calib.yaw_min   = center_y; joy_calib.yaw_max   = center_y;
        joy_calib.roll_center  = center_r; joy_calib.roll_min  = center_r; joy_calib.roll_max  = center_r;
        
        // Throttle has a center too, but we start tracking min/max from its resting point
        joy_calib.throttle_center = center_t; joy_calib.throttle_min = center_t; joy_calib.throttle_max = center_t;
        
        ESP_LOGI("CALIB", "SWIRL STICKS IN FULL CIRCLES NOW!");
        
        for (int i = 0; i < 500; i++) { // 10 seconds total
            adc_oneshot_read(adc_handle, JOY_PITCH_CHAN, &raw_p);
            adc_oneshot_read(adc_handle, JOY_YAW_CHAN, &raw_y);
            adc_oneshot_read(adc_handle, JOY_ROLL_CHAN, &raw_r);
            adc_oneshot_read(adc_handle, JOY_THROTTLE_CHAN, &raw_t);
            
            if(raw_p < joy_calib.pitch_min) joy_calib.pitch_min = raw_p;
            if(raw_p > joy_calib.pitch_max) joy_calib.pitch_max = raw_p;
            
            if(raw_y < joy_calib.yaw_min) joy_calib.yaw_min = raw_y;
            if(raw_y > joy_calib.yaw_max) joy_calib.yaw_max = raw_y;
            
            if(raw_r < joy_calib.roll_min) joy_calib.roll_min = raw_r;
            if(raw_r > joy_calib.roll_max) joy_calib.roll_max = raw_r;
            
            if(raw_t < joy_calib.throttle_min) joy_calib.throttle_min = raw_t;
            if(raw_t > joy_calib.throttle_max) joy_calib.throttle_max = raw_t;
            
            // Blink LED rapidly
            if(led_callback) led_callback((i % 10 < 5) ? 50 : 0, 0, (i % 10 < 5) ? 50 : 0);
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        int padding = 30; // Safety padding
        joy_calib.pitch_min -= padding; joy_calib.pitch_max += padding;
        joy_calib.yaw_min -= padding;   joy_calib.yaw_max += padding;
        joy_calib.roll_min -= padding;  joy_calib.roll_max += padding;
        joy_calib.throttle_min -= padding; joy_calib.throttle_max += padding;
        
        joy_calib.is_calibrated = true;

        nvs_set_blob(my_handle, "calib_data", &joy_calib, required_size);
        nvs_commit(my_handle);
        ESP_LOGI("CALIB", "Calibration Saved!");
    } else {
        ESP_LOGI("CALIB", "Loaded existing calibration from flash.");
    }
    
    nvs_close(my_handle);
}