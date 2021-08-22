/* Audio passthru

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "audio_pipeline.h"
#include "i2s_stream.h"
#include "board.h"
#include "passthru_encoder.h"
#include "fir_filter.h"
#include "coeffs.h"

static const char *TAG = "PASSTHRU";

audio_element_handle_t fir_filter = NULL;

void change_filter_balance( uint32_t balance )
{
	if ( fir_filter != NULL )
		set_balance( fir_filter, .8 + (float)balance / 250.0 );
}

void phase_filter( void* param )
{
    audio_pipeline_handle_t pipeline;
    audio_element_handle_t i2s_stream_writer, i2s_stream_reader, passthru_encoder;

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    ESP_LOGI(TAG, "[ 1 ] Start codec chip");
    audio_board_handle_t board_handle = audio_board_init();

    // Set codec mode to "BOTH" to enabled ADC and DAC. Coded Mode line in simply
    // adds the Line In2 to the DAC bypassing the ADC

    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_BOTH, AUDIO_HAL_CTRL_START);

    ESP_LOGI(TAG, "[ 2 ] Create audio pipeline for playback");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipeline = audio_pipeline_init(&pipeline_cfg);

    ESP_LOGI(TAG, "[3.1] Create i2s stream to write data to codec chip");
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type = AUDIO_STREAM_WRITER;
    i2s_stream_writer = i2s_stream_init(&i2s_cfg);
/*
    ESP_LOGI(TAG, "[3.2] Create Passthru Encoder");
    passthru_encoder_cfg_t passthru_cfg = DEFAULT_passthru_encoder_CONFIG();
    passthru_encoder = passthru_encoder_init(&passthru_cfg);
*/

    ESP_LOGI(TAG, "[3.2] Create FIR Filter");
    fir_filter_cfg_t fir_filter_cfg = DEFAULT_fir_filter_CONFIG();

    fir_filter_cfg.type = FF_RIGHT_ONLY;
    fir_filter_cfg.balance_adjust = 1;

/*
    fir_filter_cfg.firLen = FIR_LEN;
    fir_filter_cfg.coeffsLeft = coeffs_minus45;
    fir_filter_cfg.coeffsRight = coeffs_plus45;
*/
    fir_filter_cfg.firLen = 300;

    // Lower Sideband
    fir_filter_cfg.coeffsLeft = coeffs_300plus45;
    fir_filter_cfg.coeffsRight = coeffs_300minus45;

    // Upper Sideband
    fir_filter_cfg.coeffsLeft = coeffs_300minus45;
    fir_filter_cfg.coeffsRight = coeffs_300plus45;
/*


    fir_filter_cfg.firLen = 60;
    fir_filter_cfg.coeffsLeft = coeffs_60minus45;
    fir_filter_cfg.coeffsRight = coeffs_60plus45;

    fir_filter_cfg.firLen = 30;
    fir_filter_cfg.coeffsLeft = coeffs_30minus45;
    fir_filter_cfg.coeffsRight = coeffs_30plus45;
*/

    fir_filter = fir_filter_init(&fir_filter_cfg);


    ESP_LOGI(TAG, "[3.3] Create i2s stream to read data from codec chip");
    i2s_stream_cfg_t i2s_cfg_read = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg_read.type = AUDIO_STREAM_READER;
    i2s_stream_reader = i2s_stream_init(&i2s_cfg_read);


    ESP_LOGI(TAG, "[3.3] Register all elements to audio pipeline");
    audio_pipeline_register(pipeline, i2s_stream_reader, "i2s_read");
    audio_pipeline_register(pipeline, fir_filter, "fir");
    audio_pipeline_register(pipeline, i2s_stream_writer, "i2s_write");

    ESP_LOGI(TAG, "[3.4] Link it together [codec_chip]-->i2s_stream_reader-->i2s_stream_writer-->[codec_chip]");

    const char *link_tag[3] = {"i2s_read", "fir", "i2s_write"};
    audio_pipeline_link(pipeline, &link_tag[0], 3);

    //const char *link_tag[2] = {"i2s_read", "i2s_write"};
    //audio_pipeline_link(pipeline, &link_tag[0], 2);


    ESP_LOGI(TAG, "[ 4 ] Set up  event listener");
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    audio_event_iface_handle_t evt = audio_event_iface_init(&evt_cfg);

    ESP_LOGI(TAG, "[4.1] Listening event from all elements of pipeline");
    audio_pipeline_set_listener(pipeline, evt);

    ESP_LOGI(TAG, "[ 5 ] Start audio_pipeline");
    audio_pipeline_run(pipeline);

    ESP_LOGI(TAG, "[ 6 ] Listen for all pipeline events");

    while (1) {
        audio_event_iface_msg_t msg;
        esp_err_t ret = audio_event_iface_listen(evt, &msg, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[ * ] Event interface error : %d", ret);
            continue;
        }

        /* Stop when the last pipeline element (i2s_stream_writer in this case) receives stop event */
        if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == (void *) i2s_stream_writer
            && msg.cmd == AEL_MSG_CMD_REPORT_STATUS
            && (((int)msg.data == AEL_STATUS_STATE_STOPPED) || ((int)msg.data == AEL_STATUS_STATE_FINISHED))) {
            ESP_LOGW(TAG, "[ * ] Stop event received");
            break;
        }
    }

    ESP_LOGI(TAG, "[ 7 ] Stop audio_pipeline");
    audio_pipeline_stop(pipeline);
    audio_pipeline_wait_for_stop(pipeline);
    audio_pipeline_terminate(pipeline);

    audio_pipeline_unregister(pipeline, i2s_stream_reader);
    audio_pipeline_unregister(pipeline, i2s_stream_writer);

    /* Terminate the pipeline before removing the listener */
    audio_pipeline_remove_listener(pipeline);

    /* Make sure audio_pipeline_remove_listener & audio_event_iface_remove_listener are called before destroying event_iface */
    audio_event_iface_destroy(evt);

    /* Release all resources */
    audio_pipeline_deinit(pipeline);
    audio_element_deinit(i2s_stream_reader);
    audio_element_deinit(i2s_stream_writer);
}
