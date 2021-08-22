
#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "si5351.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"


#define I2C_MASTER_NUM	1
#define I2C_MASTER_SDA_IO 17
#define I2C_MASTER_SCL_IO 16

static const char *TAG = "esp32-transmitter";

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   128          //Multisampling

static esp_adc_cal_characteristics_t *adc_chars;
#if CONFIG_IDF_TARGET_ESP32
static const adc1_channel_t channel = ADC1_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
#elif CONFIG_IDF_TARGET_ESP32S2
static const adc_channel_t channel = ADC_CHANNEL_6;     // GPIO7 if ADC1, GPIO17 if ADC2
static const adc_bits_width_t width = ADC_WIDTH_BIT_13;
#endif
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;


static void check_efuse(void)
{
#if CONFIG_IDF_TARGET_ESP32
    //Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
#elif CONFIG_IDF_TARGET_ESP32S2
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("Cannot retrieve eFuse Two Point calibration values. Default calibration values will be used.\n");
    }
#else
#error "This example is configured for ESP32/ESP32S2."
#endif
}


static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}



extern "C" {
	void app_main();
	void phase_filter( void* );
	void change_filter_balance( uint32_t balance );
}

Si5351 			synth;

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

int lastMult = -1;

void changeFrequency( int currentFrequency )
{
	  int mult = 0;

	  if ( currentFrequency < 8000000 )
		  mult = 100;
	  else if ( currentFrequency < 11000000 )
		  mult = 80;
	  else if ( currentFrequency < 15000000 )
		  mult = 50;
	  else if ( currentFrequency < 22000000 )
		  mult = 40;
	  else if ( currentFrequency < 30000000 )
		  mult = 30;

	  uint64_t freq = currentFrequency * 100ULL;
	  uint64_t pllFreq = freq * mult;

	  synth.set_freq_manual(freq, pllFreq, SI5351_CLK0);
	  synth.set_freq_manual(freq, pllFreq, SI5351_CLK2);

	  if ( mult != lastMult )
	  {
		  synth.drive_strength( SI5351_CLK0, SI5351_DRIVE_8MA );
		  synth.drive_strength( SI5351_CLK2, SI5351_DRIVE_8MA );
		  synth.set_phase(SI5351_CLK0, 0);
		  synth.set_phase(SI5351_CLK2, mult);
		  synth.pll_reset(SI5351_PLLA);
		  lastMult = mult;
	  }
}


void app_main()
{

    ESP_LOGI(TAG, "Initializing I2C");
	i2c_master_init();

	ESP_LOGI(TAG, "Initializing si5351");
	synth.init( I2C_MASTER_NUM, SI5351_CRYSTAL_LOAD_8PF, 25000000, 0 );

	int freq = 14100000;
	changeFrequency(freq);

    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(width);
        adc1_config_channel_atten(channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    //Characterize ADC
    adc_chars = (esp_adc_cal_characteristics_t*) calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);



	ESP_LOGI(TAG, "Entering Audio Processing loop");
    xTaskCreate( &phase_filter, "phase_filter_task_0", 4096 * 2, (void* ) 0, 10, NULL);

	ESP_LOGI(TAG, "Created Audio Processing loop");

    uint32_t last_adc_reading = 0;

    while(1)
    {
        uint32_t adc_reading = 0;

        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)channel);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel, width, &raw);
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES;
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);

        adc_reading = adc_reading * 100 / 4096;

        if ( last_adc_reading != adc_reading ) {
        	last_adc_reading = adc_reading;
        	change_filter_balance( adc_reading );
            printf("Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
        }

        vTaskDelay(pdMS_TO_TICKS(200));

    }
}
