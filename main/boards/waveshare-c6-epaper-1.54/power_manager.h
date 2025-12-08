#pragma once
#include <vector>
#include <functional>
#include <esp_log.h>
#include <esp_timer.h>
#include <driver/gpio.h>
#include "esp_io_expander_tca9554.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include <math.h>


class PowerManager {
private:
    i2c_master_bus_handle_t& i2cbus_;
    int bat_adc_pin_ = GPIO_NUM_NC;
    int vbat_keypin_ = GPIO_NUM_NC;
    const int epd_power_pin_;
    const int audio_power_pin_;
    const int vbat_power_pin_;
    adc_cali_handle_t cali_handle_;
    adc_oneshot_unit_handle_t adc_handle_;
    adc_channel_t adc_channel_;
    bool do_calibration = false;
    bool is_state = false;
    esp_io_expander_handle_t io_expander = NULL;
    
    float GetAdcLevel() {
        if (adc_handle_ != nullptr) {
            int adc_raw = 0;
            ESP_ERROR_CHECK(adc_oneshot_read(adc_handle_, adc_channel_, &adc_raw));
            if (do_calibration) {
                int adc_int = 0;
                ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali_handle_, adc_raw, &adc_int));
                float adc_float = (adc_int / 1000.0f) * 2.0;
                return adc_float;
            }
        }
        return 0;
    }

    static void PowerManagerLoopState(void *arg) {
        PowerManager *manager = (PowerManager *)arg;
        float lastvol;
        float Thisvol;
        for(;;) {
            lastvol = manager->GetAdcLevel(); 
            vTaskDelay(pdMS_TO_TICKS(30000));
            Thisvol = manager->GetAdcLevel();
            ESP_LOGE("VOL","%.2f,%.2f",lastvol,Thisvol);
            if(fabs(Thisvol - lastvol) > 0.05) {
                if(lastvol > Thisvol) {
                    manager->is_state = false;    
                } else {
                    manager->is_state = true;
                }
            }
        }
    }

public:
    PowerManager(i2c_master_bus_handle_t& i2cbus,int bat_adc_pin,int vbat_keypin = GPIO_NUM_NC,int epd_power_pin = IO_EXPANDER_PIN_NUM_0, int audio_power_pin = IO_EXPANDER_PIN_NUM_1, int vbat_power_pin = IO_EXPANDER_PIN_NUM_5):
    i2cbus_(i2cbus),
    bat_adc_pin_(bat_adc_pin),  
    vbat_keypin_(vbat_keypin),
    epd_power_pin_(epd_power_pin),
    audio_power_pin_(audio_power_pin),
    vbat_power_pin_(vbat_power_pin) {
        ESP_ERROR_CHECK(esp_io_expander_new_i2c_tca9554(i2cbus_, ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_000, &io_expander));
        ESP_ERROR_CHECK(esp_io_expander_set_dir(io_expander, epd_power_pin_ | audio_power_pin_ | vbat_power_pin_, IO_EXPANDER_OUTPUT));
        ESP_ERROR_CHECK(esp_io_expander_set_level(io_expander, epd_power_pin_ | audio_power_pin_ | vbat_power_pin_, 1));

        /****初始化adc****/
        if (bat_adc_pin_ != GPIO_NUM_NC) {
            if (bat_adc_pin_ >= GPIO_NUM_0 && bat_adc_pin_ <= GPIO_NUM_6)
                adc_channel_ = (adc_channel_t)((int)bat_adc_pin_);
            else
                return;

            adc_cali_curve_fitting_config_t cali_config = {};
    	    cali_config.unit_id = ADC_UNIT_1;
    	    cali_config.atten = ADC_ATTEN_DB_12;
    	    cali_config.bitwidth = ADC_BITWIDTH_12;
            ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle_));

	        adc_oneshot_unit_init_cfg_t init_config1 = {};
    	    init_config1.unit_id = ADC_UNIT_1;
  	        ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc_handle_));
  	        adc_oneshot_chan_cfg_t config = {};
    	    config.bitwidth = ADC_BITWIDTH_12;            
    	    config.atten = ADC_ATTEN_DB_12;
  	        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle_, (adc_channel_t)adc_channel_, &config));
            do_calibration = true;
        }
        
        if(vbat_keypin_ != GPIO_NUM_NC) {
		    gpio_config_t gpio_conf = {};
  		    gpio_conf.intr_type = GPIO_INTR_DISABLE;
  		    gpio_conf.mode = GPIO_MODE_INPUT;
  		    gpio_conf.pin_bit_mask = (0x1ULL<<vbat_keypin_);
  		    gpio_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  		    gpio_conf.pull_up_en = GPIO_PULLUP_ENABLE;
  		    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&gpio_conf));
        }

        xTaskCreatePinnedToCore(PowerManagerLoopState, "PowerManagerLoopState", 3 * 1024, (void *)this, 2, NULL,0);
	}
    
    ~PowerManager() {
        if (adc_handle_) {
            ESP_ERROR_CHECK(adc_oneshot_del_unit(adc_handle_));
        }
    }

    /******adc部分******/
    int GetBatteryLevel(void) {
        const float voltage_float_threshold = 0.1f;
        float voltage_float = 0.0f;
        static float last_voltage_float = 0.0f;
        static int last_battery_level = 0;

        voltage_float = GetAdcLevel();
        if (voltage_float) {
            if (fabs(voltage_float - last_voltage_float) >= voltage_float_threshold) {
                last_voltage_float = voltage_float;
                ESP_LOGE("123133","ADAD");
                if (voltage_float < 3.52) {
                    last_battery_level = 1;
                } else if (voltage_float < 3.64) {
                    last_battery_level = 20;
                } else if (voltage_float < 3.76) {
                    last_battery_level = 40;
                } else if (voltage_float < 3.88) {
                    last_battery_level = 60;
                } else if (voltage_float < 4.0) {
                    last_battery_level = 80;
                } else {
                    last_battery_level = 100;
                }
            }
            return last_battery_level;
        }
        return 100;
    }

    bool IsCharging(void) {
        ESP_LOGE("IsCharging","%d",is_state);
        return is_state;
    }
    
    bool IsDischarging(void) {
        ESP_LOGE("IsDischarging","%d",(is_state == false) ? true : false);
        return (is_state == false) ? true : false;
    }

    bool IsChargingDone(void) {
        if (GetBatteryLevel() == 100) {
            return true;
        }
        return false;
    }

    /******power部分******/
    esp_io_expander_handle_t Get_IoExpanderHandle() {
        return io_expander;
    }

    void PowerEpdOn() {
        ESP_ERROR_CHECK(esp_io_expander_set_level(io_expander, epd_power_pin_, 1));
    }

    void PowerEpdOff() {
        ESP_ERROR_CHECK(esp_io_expander_set_level(io_expander, epd_power_pin_, 0));
    }

    void PowerAudioOn() {
        ESP_ERROR_CHECK(esp_io_expander_set_level(io_expander, audio_power_pin_, 1));
    }

    void PowerAudioOff() {
        ESP_ERROR_CHECK(esp_io_expander_set_level(io_expander, audio_power_pin_, 0));
    }

    void PowerVbatOn() {
        ESP_ERROR_CHECK(esp_io_expander_set_level(io_expander, vbat_power_pin_, 1));
    }

    void PowerVbatOff() {
        ESP_ERROR_CHECK(esp_io_expander_set_level(io_expander, vbat_power_pin_, 0));
    }

    void PowerLoopState() {
        while(!gpio_get_level((gpio_num_t)vbat_keypin_)) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
};
