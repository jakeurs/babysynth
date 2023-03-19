#include "dac_audio_example_main.h"
#include "instrument.h"
#include "samples.h"
#include "ledMatrix.h"

#include "mpr121.h"
#define CONFIG_I2C_ADDRESS 0x5A
#define CONFIG_SCL_GPIO 22
#define CONFIG_SDA_GPIO 21
#define CONFIG_IRQ_GPIO 19
MPR121_t dev;
uint16_t touchThreshold = 40;
uint16_t releaseThreshold = 20;

#define GPIO_OUTPUT_IO_0 4
#define GPIO_OUTPUT_PIN_SEL (1ULL << GPIO_OUTPUT_IO_0)
#define HAPTIC_MOTOR_TIMER_MAX 1
uint16_t haptic_motor_counter = 0;
// #define ENABLE_HAPTIC_MOTOR

#define NUM_ADC_CHANNELS 5
#define ADC_MIN 120
#define ADC_MAX 3800
#define SLIDER_MIN 0
#define SLIDER_MAX 4095
#define ANALOG_0 36
#define ANALOG_1 39
#define ANALOG_2 34
#define ANALOG_3 35
#define ANALOG_4 32
#define ANALOG_5 33

uint16_t sliderValues[5] = {0}; // 0 to 4095 (12 bit)
static int adc_raw[2][11];

int16_t wave_table_sine[WAVETABLE_SIZE];
int16_t wave_table_square[WAVETABLE_SIZE];
int16_t wave_table_saw[WAVETABLE_SIZE];
int16_t wave_table_triangle[WAVETABLE_SIZE];
int16_t wave_table_noise[WAVETABLE_SIZE];
uint8_t samplebuffer[SAMPLEBUFFER_SIZE];
uint16_t samplebuffer_readptr, samplebuffer_writeptr;

uint32_t cnt = 0;
int16_t val = 0;

#define NUM_OSC 1
#define NUM_INSTRUMENTS 5
#define NUM_SAMPLERS 7

MasterSequencer mseq;
InstrumentSequencer iseqs[SEQUENCER_NUM_INSTRUMENTS];
Instrument instruments[NUM_INSTRUMENTS];
Sampler samplers[NUM_SAMPLERS];
LPFilter lpFilter;

int32_t displayData[16];

#define SIGNAL_DISPLAY_TIMER_MAX 8;
uint8_t signalDisplayTimer = SIGNAL_DISPLAY_TIMER_MAX;

adc_oneshot_unit_handle_t adc1_handle;
adc_oneshot_unit_init_cfg_t init_config1 = {
    .unit_id = ADC_UNIT_1,
    .ulp_mode = ADC_ULP_MODE_DISABLE,
};

void haptic_init(void)
{
    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    // disable pull-down mode
    io_conf.pull_down_en = 0;
    // disable pull-up mode
    io_conf.pull_up_en = 0;
    // configure GPIO with the given settings
    gpio_config(&io_conf);
}
void haptic_start(void)
{
    gpio_set_level(GPIO_OUTPUT_IO_0, 1);
    haptic_motor_counter = HAPTIC_MOTOR_TIMER_MAX;
}
void haptic_stop(void)
{
    gpio_set_level(GPIO_OUTPUT_IO_0, 0);
    haptic_motor_counter = 0;
}
void haptic_tick(void)
{
    if (haptic_motor_counter > 0)
    {
        haptic_motor_counter--;
        if (haptic_motor_counter <= 0)
        {
            haptic_stop();
        }
    }
}

void init_MPR121(void)
{
    bool ret = MPR121_begin(&dev, CONFIG_I2C_ADDRESS, touchThreshold, releaseThreshold, CONFIG_IRQ_GPIO, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO);
    ESP_LOGI(TAG, "MPR121_begin=%d", ret);
    if (ret == false)
    {
        switch (MPR121_getError(&dev))
        {
        case NO_ERROR:
            ESP_LOGE(TAG, "no error");
            break;
        case ADDRESS_UNKNOWN:
            ESP_LOGE(TAG, "incorrect address");
            break;
        case READBACK_FAIL:
            ESP_LOGE(TAG, "readback failure");
            break;
        case OVERCURRENT_FLAG:
            ESP_LOGE(TAG, "overcurrent on REXT pin");
            break;
        case OUT_OF_RANGE:
            ESP_LOGE(TAG, "electrode out of range");
            break;
        case NOT_INITED:
            ESP_LOGE(TAG, "not initialised");
            break;
        default:
            ESP_LOGE(TAG, "unknown error");
            break;
        }
        while (1)
        {
            vTaskDelay(1);
        }
    }
    MPR121_setFFI(&dev, FFI_10);                 // AFE Configuration 1
    MPR121_setSFI(&dev, SFI_10);                 // AFE Configuration 2
    MPR121_setGlobalCDT(&dev, CDT_4US);          // reasonable for larger capacitances
    MPR121_autoSetElectrodesDefault(&dev, true); // autoset all electrode settings
}

static esp_err_t
mpu6050_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief Write a byte to a MPU9250 sensor register
 */
static esp_err_t mpu6050_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}

/**
 * @brief i2c master initialization
 */
/*
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
*/

static bool IRAM_ATTR dac_on_convert_done_callback(dac_continuous_handle_t handle, const dac_event_data_t *event, void *user_data)
{
    QueueHandle_t que = (QueueHandle_t)user_data;
    BaseType_t need_awoke;
    /* When the queue is full, drop the oldest item */
    if (xQueueIsQueueFullFromISR(que))
    {
        dac_event_data_t dummy;
        xQueueReceiveFromISR(que, &dummy, &need_awoke);
    }
    /* Send the event from callback */
    xQueueSendFromISR(que, event, &need_awoke);
    return need_awoke;
}

void getADCData(void)
{
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &adc_raw[0][0]));
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_3, &adc_raw[0][1]));
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_6, &adc_raw[0][2]));
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_7, &adc_raw[0][3]));
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_4, &adc_raw[0][4]));
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_5, &adc_raw[0][5]));
    // ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC_CHANNEL_4, adc_raw[0][0]);
    // ESP_LOGI(TAG, "%u", adc_raw[0][5]);

    for (int i = 0; i < NUM_ADC_CHANNELS; i++)
    {
        /* remap slider range to 12 bit unsigned: 0 - 4095*/
        adc_raw[0][i] = CLAMP(adc_raw[0][i], ADC_MIN, ADC_MAX);
        adc_raw[0][i] = ((int32_t)(adc_raw[0][i] - ADC_MIN) * (SLIDER_MAX - SLIDER_MIN)) / (ADC_MAX - ADC_MIN) + SLIDER_MIN;

        /* weighted average low pass filter */
        sliderValues[i] = (sliderValues[i] * 15 + adc_raw[0][i]) / 16;

        /* clamp slider to acceptable range */
        sliderValues[i] = CLAMP(sliderValues[i], SLIDER_MIN, SLIDER_MAX);
    }
    // ESP_LOGI(TAG, "ADC: 1[%d | %u] 2[%d | %u] 3[%d | %u] 4[%d | %u] 5[%d | %u]", adc_raw[0][0], sliderValues[0], adc_raw[0][1], sliderValues[1], adc_raw[0][2], sliderValues[2], adc_raw[0][3], sliderValues[3], adc_raw[0][4], sliderValues[4]);
    //  ESP_LOGI(TAG, "Sliders: 1[%d] 2[%d] 3[%d] 4[%d] 5[%d]", sliderValues[0], sliderValues[1], sliderValues[2], sliderValues[3], sliderValues[4]);
}

// 0 = no touch
// 1 = new touch
// 2 = new release
uint8_t getTouchData(void)
{
    uint8_t status = 0;
    MPR121_updateAll(&dev);
    for (int i = 0; i < 12; i++)
    {
        if (MPR121_isNewTouch(&dev, i))
        {
            status = 1;
            // ESP_LOGI(TAG, "setting freq to %lu", scaleNoteToFreqMilliHz(i));
            if (i % 2 == 0)
                Instrument_SetFreq(&instruments[1], scaleNoteToFreqMilliHz(i));
            if (i % 2 == 1)
                Instrument_SetFreq(&instruments[0], scaleNoteToFreqMilliHz(i));
            if (i < NUM_SAMPLERS)
                Sampler_Play(&samplers[i]);
                // OscillatorBankSetFrequency(&oscBank, scaleNoteToFreqMilliHz(i));
#ifdef ENABLE_HAPTIC_MOTOR
            haptic_start();
#endif
        }
        else if (MPR121_isNewRelease(&dev, i))
        {
            // ESP_LOGI(TAG, "electrode %d was just released", i);
            status = 2;
        }
    }
    return status;
}

static void dac_write_data_asynchronously(dac_continuous_handle_t handle, QueueHandle_t que, uint8_t *data, size_t data_size)
{

    while (1)
    {
        // printf("Play count: %" PRIu32 "\n", cnt++);
        dac_event_data_t evt_data;
        int16_t gainCB = -1000; //-248 is nice
        uint16_t volume_ch_0 = 0;
        uint16_t volume_ch_1 = 0;
        uint16_t volume_ch_2 = 0;
        uint16_t volume_ch_3 = 0;
        uint16_t volume_ch_4 = 0;

        /* Receive the event from callback and load the data into the DMA buffer until the whole audio loaded */
        // while (byte_written < data_size)
        while (1)
        {

            // oscPeriodMS = 6.810504;
            if (cnt++ > 5)
            {
#ifdef ENABLE_HAPTIC_MOTOR
                haptic_tick();
#endif

                uint8_t touchStatus = getTouchData();

                if (touchStatus == 1)
                {
                    // ADSRNoteEvent(&adsr, true, false);
                    // ADSRNoteEvent(&adsrFilter, true, false);
                    // Instrument_NoteOn(&instruments[1]);
                }
                else if (touchStatus == 2)
                {
                    // Instrument_NoteOff(&instruments[1]);
                    //  ADSRNoteEvent(&adsr, false, true);
                    //  ADSRNoteEvent(&adsrFilter, false, true);
                }

                getADCData();
                gainCB = -(4095 - sliderValues[0]) / 4;
                /*
                uint32_t attackSteps = sliderValues[2] * 50;
                uint32_t decaySteps = sliderValues[3] * 50;
                uint32_t releaseSteps = sliderValues[4] * 250;

                if (ABS((int32_t)adsrTarget->attack.durationSamples - (int32_t)attackSteps) > 10000)
                {
                    // ESP_LOGI(TAG, "(int32_t)adsr.attack.durationSamples = %ld (int32_t)attackSteps = %ld res = %ld ABS(res) = %ld", (int32_t)adsr.attack.durationSamples, (int32_t)attackSteps, (int32_t)adsr.attack.durationSamples - (int32_t)attackSteps, ABS((int32_t)adsr.attack.durationSamples - (int32_t)attackSteps));
                    // ESP_LOGI(TAG, "envelope = %u attack = %lu", adsr.envelope, attackSteps);
                    RampSetTime(&adsrTarget->attack, attackSteps);
                }
                if (ABS((int32_t)adsrTarget->decay.durationSamples - (int32_t)decaySteps) > 10000)
                {
                    // ESP_LOGI(TAG, "envelope = %u decay %lu", adsr.envelope, decaySteps);
                    RampSetTime(&adsrTarget->decay, decaySteps);
                }
                if (ABS((int32_t)adsr.release.durationSamples - (int32_t)releaseSteps) > 10000)
                {
                    // ESP_LOGI(TAG, "envelope = %u release = %lu", adsr.envelope, releaseSteps);
                    RampSetTime(&adsrTarget->release, releaseSteps);
                }
                */
                // OscillatorBankSetDetune(&oscBank, detuneMilliHz);

                // ESP_LOGI(TAG, "gainCB = %d detuneMilliHz = %u", gainCB, detuneMilliHz);
                //  ESP_LOGI(TAG, "envelope = %u attack = %u decay %u release = %u", envelope, attackSteps, decaySteps, releaseSteps);
                // plotRamps(&adsr);
                // plotRamps(&adsrFilter);
                // sliderValues[0] = 400;
                // sliderValues[1] = 400;
                // sliderValues[2] = 0;
                // sliderValues[3] = 4096;

                if (ABS(lpFilter.Q * 512 - (int32_t)sliderValues[3]) > 200)
                {
                    lpFilter.Q = sliderValues[3] / 512.0;
                    LPFilterCalculate(&lpFilter);
                }
                if (ABS(lpFilter.frequencyMilliHz - (int32_t)sliderValues[4] * 1000) > 2000)
                {
                    lpFilter.frequencyMilliHz = sliderValues[4] * 1000;
                    LPFilterCalculate(&lpFilter);
                }
                if (ABS(volume_ch_0 - (int32_t)sliderValues[0]) > 20)
                {
                    volume_ch_0 = sliderValues[0];
                    ESP_LOGI(TAG, "volume ch 0 = %u", volume_ch_0);
                }
                if (ABS(volume_ch_1 - (int32_t)sliderValues[1]) > 20)
                {
                    volume_ch_1 = sliderValues[1];
                    ESP_LOGI(TAG, "volume ch 1 = %u", volume_ch_1);
                }
                if (ABS(volume_ch_2 - (int32_t)sliderValues[2]) > 20)
                {
                    volume_ch_2 = sliderValues[2];
                    volume_ch_3 = sliderValues[2];
                    volume_ch_4 = sliderValues[2];
                    ESP_LOGI(TAG, "volume ch 2 = %u", volume_ch_2);
                }

                // volume_ch_0 = 3600;
                // volume_ch_1 = 3600;

                cnt = 0;
            }

            xQueueReceive(que, &evt_data, portMAX_DELAY);
            size_t loaded_bytes = 0;

            //**fix for display
            // float oscPeriodMS = 2.55;
            static float waveformPeriodTracker = 0;
            float oscPeriodMS = 1.0 / instruments[0].oscBank0.frequencyMilliHz * 1000000;
            float displayDataBinSizeFloat = oscPeriodMS / 16;
            static int c = 0;
            // for (int i = 0; i < evt_data.buf_size / 2; i++)
            for (int i = 0; i < 256; i++)
            {

                MasterSequencer_Step(&mseq, SAMPLE_PERIOD);
                Instrument_Tick(&instruments[0], i);
                Instrument_Tick(&instruments[1], i);
                // Instrument_Tick(&instruments[2], i);
                // Instrument_Tick(&instruments[3], i);
                // Instrument_Tick(&instruments[4], i);
                int32_t signal_samples = 0;
                for (int j = 0; j < NUM_SAMPLERS; j++)
                {
                    Sampler_Tick(&samplers[j]);
                    if (j > 2) // mix drums separately
                        signal_samples += ((int32_t)samplers[j].signal - 127) * 32;
                }

                int32_t signal0 = instruments[0].signal;
                int32_t signal1 = instruments[1].signal;
                int32_t signal2 = ((int32_t)samplers[0].signal - 127) * 32;
                int32_t signal3 = ((int32_t)samplers[1].signal - 127) * 32;
                int32_t signal4 = ((int32_t)samplers[2].signal - 127) * 32;

                int32_t mix =
                    (signal0 * 65536) / (volume_ch_0 + 1) +
                    (signal1 * 65536) / (volume_ch_1 + 1) +
                    (signal2 * 65536) / (volume_ch_2 + 1) +
                    (signal3 * 65536) / (volume_ch_3 + 1) +
                    (signal4 * 65536) / (volume_ch_4 + 1) +
                    (signal_samples * 65536) / 16;
                // mix = adjustSignalVolume(mix, gainCB);
                mix = LPFilterTick(&lpFilter, mix);
                mix = mix / 65536;

                mix = mix > 127 ? 127 : mix;
                mix = mix < -127 ? -127 : mix;
                uint8_t out = (uint8_t)(mix + 127);

                data[samplebuffer_writeptr] = out;
                // data[samplebuffer_writeptr] = 0;

                samplebuffer_writeptr++;
                samplebuffer_writeptr %= SAMPLEBUFFER_SIZE - 512;

                waveformPeriodTracker += SAMPLE_PERIOD;
                while (waveformPeriodTracker > oscPeriodMS)
                    waveformPeriodTracker -= oscPeriodMS;

                uint8_t bin = (uint8_t)floorf(waveformPeriodTracker / displayDataBinSizeFloat);
                if (bin < 16) // rare error when switching notes where bin size is larger than 16
                {
                    // display output
                    displayData[bin] = (displayData[bin] + (int16_t)out - 127) / 2;
                    // displayData[bin] = (displayData[bin] + (int16_t)instruments[1].signal / (sliderValues[0] + 1)) / 2;
                }
                else
                {
                    ESP_LOGI(TAG, "display bin error: bin = %u which is larger than the 16 available bins", bin);
                }
            }

            ESP_ERROR_CHECK(dac_continuous_write_asynchronously(handle, evt_data.buf, evt_data.buf_size,
                                                                &data[samplebuffer_readptr], evt_data.buf_size, &loaded_bytes));

            samplebuffer_readptr += loaded_bytes;
            samplebuffer_readptr %= SAMPLEBUFFER_SIZE - 512;

            if (signalDisplayTimer == 0)
            {
                signalDisplayTimer = SIGNAL_DISPLAY_TIMER_MAX;
                int8_t outputDisplayData[16];
                for (int i = 0; i < 16; i++)
                {
                    outputDisplayData[i] = CLAMP(displayData[i], -127, 127);
                    if (ABS(outputDisplayData[i]) == 127)
                        outputDisplayData[i] = 0;
                }
                ledMatrix_plot_signal(&outputDisplayData[0]);
            }
            else
            {
                signalDisplayTimer--;
            }
        }
        /* Clear the legacy data in DMA, clear times equal to the 'dac_continuous_config_t::desc_num' */
        for (int i = 0; i < 4; i++)
        {
            xQueueReceive(que, &evt_data, portMAX_DELAY);
            memset(evt_data.buf, 0, evt_data.buf_size);
        }
        // vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
void initWaveTable_sine(void)
{
    for (int i = 0; i < WAVETABLE_SIZE; i++)
    {
        wave_table_sine[i] = (int16_t)(sin((float)i / WAVETABLE_SIZE * 6.28318) * (INT16_MAX));
    }
}
void initWaveTable_saw(void)
{
    int32_t step = (INT16_MAX * 2) / WAVETABLE_SIZE;
    int32_t start = step * -WAVETABLE_SIZE / 2;
    for (int i = 0; i < WAVETABLE_SIZE; i++)
    {
        wave_table_saw[i] = start + step * i;
    }
}
void initWaveTable_square(void)
{
    for (int i = 0; i < WAVETABLE_SIZE; i++)
    {
        wave_table_square[i] = i < WAVETABLE_SIZE / 2 ? INT16_MIN : INT16_MAX;
    }
}
void initWaveTable_triangle(void)
{
    for (int i = 0; i < WAVETABLE_SIZE; i++)
    {
        wave_table_triangle[i] =
            (2.0 *
                 ABS(2.0 *
                     (((i + WAVETABLE_SIZE / 4) % WAVETABLE_SIZE) / (float)WAVETABLE_SIZE -
                      floor(
                          ((i + WAVETABLE_SIZE / 4) % WAVETABLE_SIZE) / (float)WAVETABLE_SIZE + 1.0 / 2.0))) -
             1.0) *
            INT16_MAX;
    }
}
void initWaveTable_noise(void)
{
    static uint16_t a = 123;
    for (int i = 0; i < WAVETABLE_SIZE; i++)
    {
        a = (a ^ 61) ^ (a >> 16);
        a = a + (a << 3);
        a = a ^ (a >> 4);
        a = a * 0x27d4eb2d;
        a = a ^ (a >> 15);
        wave_table_noise[i] = a;
    }
}

void initSequencer()
{
    MasterSequencer_Init(&mseq, 120);
    for (int i = 0; i < SEQUENCER_NUM_INSTRUMENTS; i++)
    {
        InstrumentSequencer_Init(&iseqs[i], i);
        MasterSequencer_AddInstrument(&mseq, &iseqs[i]);
    }
}
void initSequencer_addInstrument(Instrument *pInst, uint16_t id)
{
    assert(id < SEQUENCER_NUM_INSTRUMENTS);
    InstrumentSequencer_RegisterInstrument(&iseqs[id], pInst);
}
void initSequencer_addSampler(Sampler *pSampler, uint16_t id)
{
    assert(id < SEQUENCER_NUM_INSTRUMENTS);
    InstrumentSequencer_RegisterSampler(&iseqs[id], pSampler);
}
void initSequencer_arp(uint16_t id)
{
    assert(id < SEQUENCER_NUM_INSTRUMENTS);
    int note = 0;
    int noteInc = 1;
    for (int bar = 0; bar < SEQUENCER_NUM_BARS; bar++)
    {
        for (int beat = 0; beat < SEQUENCER_BEATS_PER_BAR; beat++)
        {

            note = (note + noteInc) % SCALECOUNT;
            InstrumentSequencer_NoteOn(&iseqs[id], bar, beat, 0, scaleNoteToMidiNote[note]);
            InstrumentSequencer_NoteOff(&iseqs[id], bar, beat, 1);
            note = (note + noteInc) % SCALECOUNT;
            InstrumentSequencer_NoteOn(&iseqs[id], bar, beat, 2, scaleNoteToMidiNote[note]);
            InstrumentSequencer_NoteOff(&iseqs[id], bar, beat, 3);
            noteInc += beat;
        }
    }
    // Debug_InstrumentSequencer_list(&iseqs[id]);
}
void initSequencer_bass(uint16_t id)
{
    assert(id < SEQUENCER_NUM_INSTRUMENTS);
    int bar = 0;
    InstrumentSequencer_NoteOn(&iseqs[id], bar, 0, 0, scaleNoteToMidiNote[1] - 12);
    InstrumentSequencer_NoteOff(&iseqs[id], bar, 3, 0);
    bar++;
    InstrumentSequencer_NoteOn(&iseqs[id], bar, 0, 0, scaleNoteToMidiNote[5] - 12);
    InstrumentSequencer_NoteOff(&iseqs[id], bar, 3, 0);
    bar++;
    InstrumentSequencer_NoteOn(&iseqs[id], bar, 0, 0, scaleNoteToMidiNote[4] - 12);
    InstrumentSequencer_NoteOff(&iseqs[id], bar, 3, 0);
    bar++;
    InstrumentSequencer_NoteOn(&iseqs[id], bar, 0, 0, scaleNoteToMidiNote[4] - 12);
    InstrumentSequencer_NoteOff(&iseqs[id], bar, 3, 0);

    // Debug_InstrumentSequencer_list(&iseqs[id]);
}
void initSequencer_drums(uint16_t id_kick, uint16_t id_snare, uint16_t id_hh)
{
    assert(id_kick < SEQUENCER_NUM_INSTRUMENTS && id_snare < SEQUENCER_NUM_INSTRUMENTS && id_hh < SEQUENCER_NUM_INSTRUMENTS);
    for (int bar = 0; bar < SEQUENCER_NUM_BARS; bar++)
    {
        for (int beat = 0; beat < SEQUENCER_BEATS_PER_BAR; beat++)
        {
            if (beat == 0 || beat == 2)
            {
                InstrumentSequencer_NoteOn(&iseqs[id_kick], bar, beat, 0, 20);
                InstrumentSequencer_NoteOff(&iseqs[id_kick], bar, beat, 1);
            }
            else if (beat == 1 || beat == 3)
            {
                InstrumentSequencer_NoteOn(&iseqs[id_snare], bar, beat, 0, 35);
                InstrumentSequencer_NoteOff(&iseqs[id_snare], bar, beat, 1);
            }
            InstrumentSequencer_NoteOn(&iseqs[id_hh], bar, beat, 0, 80);
            InstrumentSequencer_NoteOff(&iseqs[id_hh], bar, beat, 1);
            InstrumentSequencer_NoteOn(&iseqs[id_hh], bar, beat, 2, 80);
            InstrumentSequencer_NoteOff(&iseqs[id_hh], bar, beat, 3);
        }
    }
    // Debug_InstrumentSequencer_list(&iseqs[id]);
}

void initInstruments(void)
{
    // instrument 0 - lead
    for (int i = 0; i < NUM_OSC; i++)
    {
        OscillatorSetWaveTable(&instruments[0].oscBank0.oscs[i], &wave_table_saw[0]);
        instruments[0].oscBank0.numOscillators++;
        OscillatorSetWaveTable(&instruments[0].oscBank1.oscs[i], &wave_table_triangle[0]);
        instruments[0].oscBank1.numOscillators++;
    }

    instruments[0].oscBank0FreqOffsetMilliHz = 0;
    instruments[0].oscBank1FreqOffsetMilliHz = 0;
    instruments[0].oscBank0FreqRatio10000 = 10000;
    instruments[0].oscBank1FreqRatio10000 = 10000;
    OscillatorBankSetDetune(&instruments[0].oscBank0, 0);
    OscillatorBankSetDetune(&instruments[0].oscBank1, 0);
    Instrument_SetFreq(&instruments[0], 220000);

    ADSRInitialize(&instruments[0].adsr0_amp, 4095);
    ADSRInitialize(&instruments[0].adsr1_amp, 4095);
    ADSRInitialize(&instruments[0].adsr_filter, 4095);

    instruments[0].lpFilter.z1 = 0;
    instruments[0].lpFilter.z2 = 0;
    instruments[0].lpFilter.Q = 1;
    instruments[0].lpFilter.frequencyMilliHz = 1000000;
    LPFilterCalculate(&instruments[0].lpFilter);

    instruments[0].levelOsc04096 = 4096;
    instruments[0].levelOsc14096 = 4096;
    instruments[0].gainCB = -248;

    // instrument 1 - bass
    for (int i = 0; i < NUM_OSC; i++)
    {
        OscillatorSetWaveTable(&instruments[1].oscBank0.oscs[i], &wave_table_sine[0]);
        instruments[1].oscBank0.numOscillators++;
        OscillatorSetWaveTable(&instruments[1].oscBank1.oscs[i], &wave_table_square[0]);
        instruments[1].oscBank1.numOscillators++;
    }

    instruments[1].oscBank0FreqOffsetMilliHz = 0;
    instruments[1].oscBank1FreqOffsetMilliHz = 0;
    instruments[1].oscBank0FreqRatio10000 = 10000;
    instruments[1].oscBank1FreqRatio10000 = 10000;
    OscillatorBankSetDetune(&instruments[1].oscBank0, 0);
    OscillatorBankSetDetune(&instruments[1].oscBank1, 0);
    Instrument_SetFreq(&instruments[1], 220000);

    ADSRInitialize(&instruments[1].adsr0_amp, 4095);
    ADSRInitialize(&instruments[1].adsr1_amp, 4095);
    ADSRInitialize(&instruments[1].adsr_filter, 4095);

    RampSetTime(&instruments[1].adsr0_amp.attack, 5000);
    RampSetTime(&instruments[1].adsr0_amp.decay, 500);
    RampSetTime(&instruments[1].adsr0_amp.release, 20000);

    RampSetTime(&instruments[1].adsr1_amp.attack, 1000);
    RampSetTime(&instruments[1].adsr1_amp.decay, 1000);
    RampSetTime(&instruments[1].adsr1_amp.release, 1000);

    RampSetTime(&instruments[1].adsr_filter.attack, 1000);
    RampSetTime(&instruments[1].adsr_filter.decay, 1000);
    RampSetTime(&instruments[1].adsr_filter.release, 1000);

    instruments[1].lpFilter.z1 = 0;
    instruments[1].lpFilter.z2 = 0;
    instruments[1].lpFilter.Q = 1;
    instruments[1].lpFilter.frequencyMilliHz = 1000000;
    LPFilterCalculate(&instruments[1].lpFilter);
    instruments[1].levelOsc04096 = 4096;
    instruments[1].levelOsc14096 = 4096;
    instruments[1].gainCB = -248;

    // instrument 2 - kick
    for (int i = 0; i < NUM_OSC; i++)
    {
        OscillatorSetWaveTable(&instruments[2].oscBank0.oscs[i], &wave_table_sine[0]);
        instruments[2].oscBank0.numOscillators++;
        OscillatorSetWaveTable(&instruments[2].oscBank1.oscs[i], &wave_table_noise[0]);
        instruments[2].oscBank1.numOscillators++;
    }

    instruments[2].oscBank0FreqOffsetMilliHz = 0;
    instruments[2].oscBank1FreqOffsetMilliHz = 0;
    instruments[2].oscBank0FreqRatio10000 = 10000;
    instruments[2].oscBank1FreqRatio10000 = 5000;
    OscillatorBankSetDetune(&instruments[2].oscBank0, 0);
    OscillatorBankSetDetune(&instruments[2].oscBank1, 0);
    Instrument_SetFreq(&instruments[2], 220000);

    ADSRInitialize(&instruments[2].adsr0_amp, 1024);
    ADSRInitialize(&instruments[2].adsr1_amp, 1024);
    ADSRInitialize(&instruments[2].adsr_filter, 1024);

    RampSetTime(&instruments[2].adsr0_amp.attack, 5);
    RampSetTime(&instruments[2].adsr0_amp.decay, 50);
    RampSetTime(&instruments[2].adsr0_amp.release, 100);

    RampSetTime(&instruments[2].adsr1_amp.attack, 2);
    RampSetTime(&instruments[2].adsr1_amp.decay, 40);
    RampSetTime(&instruments[2].adsr1_amp.release, 50);

    RampSetTime(&instruments[2].adsr_filter.attack, 1000);
    RampSetTime(&instruments[2].adsr_filter.decay, 1000);
    RampSetTime(&instruments[2].adsr_filter.release, 1000);

    instruments[2].lpFilter.z1 = 0;
    instruments[2].lpFilter.z2 = 0;
    instruments[2].lpFilter.Q = 6;
    instruments[2].lpFilter.frequencyMilliHz = 100000;
    LPFilterCalculate(&instruments[2].lpFilter);
    instruments[2].levelOsc04096 = 4096;
    instruments[2].levelOsc14096 = 4096;
    instruments[2].gainCB = -248;

    // instrument 3 - snare
    for (int i = 0; i < NUM_OSC; i++)
    {
        OscillatorSetWaveTable(&instruments[3].oscBank0.oscs[i], &wave_table_sine[0]);
        instruments[3].oscBank0.numOscillators++;
        OscillatorSetWaveTable(&instruments[3].oscBank1.oscs[i], &wave_table_noise[0]);
        instruments[3].oscBank1.numOscillators++;
    }

    instruments[3].oscBank0FreqOffsetMilliHz = 0;
    instruments[3].oscBank1FreqOffsetMilliHz = 0;
    instruments[3].oscBank0FreqRatio10000 = 10000;
    instruments[3].oscBank1FreqRatio10000 = 20000;
    OscillatorBankSetDetune(&instruments[3].oscBank0, 0);
    OscillatorBankSetDetune(&instruments[3].oscBank1, 0);
    Instrument_SetFreq(&instruments[3], 220000);

    ADSRInitialize(&instruments[3].adsr0_amp, 1024);
    ADSRInitialize(&instruments[3].adsr1_amp, 1024);
    ADSRInitialize(&instruments[3].adsr_filter, 1024);

    RampSetTime(&instruments[3].adsr0_amp.attack, 5);
    RampSetTime(&instruments[3].adsr0_amp.decay, 50);
    RampSetTime(&instruments[3].adsr0_amp.release, 100);

    RampSetTime(&instruments[3].adsr1_amp.attack, 2);
    RampSetTime(&instruments[3].adsr1_amp.decay, 40);
    RampSetTime(&instruments[3].adsr1_amp.release, 50);

    RampSetTime(&instruments[3].adsr_filter.attack, 1000);
    RampSetTime(&instruments[3].adsr_filter.decay, 1000);
    RampSetTime(&instruments[3].adsr_filter.release, 1000);

    instruments[3].lpFilter.z1 = 0;
    instruments[3].lpFilter.z2 = 0;
    instruments[3].lpFilter.Q = 6;
    instruments[3].lpFilter.frequencyMilliHz = 300000;
    LPFilterCalculate(&instruments[3].lpFilter);
    instruments[3].levelOsc04096 = 4096;
    instruments[3].levelOsc14096 = 4096;
    instruments[3].gainCB = -248;

    // instrument 4 - hh
    for (int i = 0; i < NUM_OSC; i++)
    {
        OscillatorSetWaveTable(&instruments[4].oscBank0.oscs[i], &wave_table_noise[0]);
        instruments[4].oscBank0.numOscillators++;
        OscillatorSetWaveTable(&instruments[4].oscBank1.oscs[i], &wave_table_noise[0]);
        instruments[4].oscBank1.numOscillators++;
    }

    instruments[4].oscBank0FreqOffsetMilliHz = 0;
    instruments[4].oscBank1FreqOffsetMilliHz = 10;
    instruments[4].oscBank0FreqRatio10000 = 10000;
    instruments[4].oscBank1FreqRatio10000 = 20000;
    OscillatorBankSetDetune(&instruments[4].oscBank0, 0);
    OscillatorBankSetDetune(&instruments[4].oscBank1, 0);
    Instrument_SetFreq(&instruments[4], 220000);

    ADSRInitialize(&instruments[4].adsr0_amp, 1024);
    ADSRInitialize(&instruments[4].adsr1_amp, 1024);
    ADSRInitialize(&instruments[4].adsr_filter, 1024);

    RampSetTime(&instruments[4].adsr0_amp.attack, 5);
    RampSetTime(&instruments[4].adsr0_amp.decay, 10);
    RampSetTime(&instruments[4].adsr0_amp.release, 10);

    RampSetTime(&instruments[4].adsr1_amp.attack, 2);
    RampSetTime(&instruments[4].adsr1_amp.decay, 5);
    RampSetTime(&instruments[4].adsr1_amp.release, 20);

    RampSetTime(&instruments[4].adsr_filter.attack, 1000);
    RampSetTime(&instruments[4].adsr_filter.decay, 1000);
    RampSetTime(&instruments[4].adsr_filter.release, 1000);

    instruments[4].lpFilter.z1 = 0;
    instruments[4].lpFilter.z2 = 0;
    instruments[4].lpFilter.Q = 6;
    instruments[4].lpFilter.frequencyMilliHz = 10000000;
    LPFilterCalculate(&instruments[4].lpFilter);
    instruments[4].levelOsc04096 = 2048;
    instruments[4].levelOsc14096 = 2048;
    instruments[4].gainCB = -248;
}

void initSamplers()
{
    Sampler_Init(&samplers[0], &sample_kick[0], false);
    Sampler_Init(&samplers[1], &sample_snare[0], false);
    Sampler_Init(&samplers[2], &sample_hh[0], false);
    Sampler_Init(&samplers[3], &sample_horse[0], false);
    Sampler_Init(&samplers[4], &sample_cow[0], false);
    Sampler_Init(&samplers[5], &sample_turkey[0], false);
    Sampler_Init(&samplers[6], &sample_pig[0], false);
    // Sampler_Init(&samplers[7], &sample_rooster[0], false);
    //  Sampler_Init(&samplers[8], &sample_sheep[0], false);
}

void app_main(void)
{
    ESP_LOGI(TAG, "DAC audio example start");
    ESP_LOGI(TAG, "--------------------------------------");

    matrix_init();

    init_MPR121();

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_4, &config));

    initWaveTable_sine();
    initWaveTable_square();
    initWaveTable_triangle();
    initWaveTable_saw();
    initWaveTable_noise();

    initInstruments();
    initSamplers();
    initSequencer();
    initSequencer_addInstrument(&instruments[0], 0);
    initSequencer_addInstrument(&instruments[1], 1);
    // initSequencer_addInstrument(&instruments[2], 2);
    // initSequencer_addInstrument(&instruments[3], 3);
    // initSequencer_addInstrument(&instruments[4], 4);

    initSequencer_addSampler(&samplers[0], 2);
    initSequencer_addSampler(&samplers[1], 3);
    initSequencer_addSampler(&samplers[2], 4);

    initSequencer_arp(0);
    initSequencer_bass(1);
    initSequencer_drums(2, 3, 4);

    lpFilter.z1 = 0;
    lpFilter.z2 = 0;
    lpFilter.Q = 1;
    lpFilter.frequencyMilliHz = 2000000;
    LPFilterCalculate(&lpFilter);

    init_mtof();

    haptic_init();

    initAmplitudeLUT();

    for (int i = 0; i < 16; i++)
        displayData[i] = 0;

    dac_continuous_handle_t dac_handle;
    dac_continuous_config_t cont_cfg = {
        .chan_mask = DAC_CHANNEL_MASK_ALL,
        .desc_num = 4,
        .buf_size = 512,
        .freq_hz = SAMPLE_RATE,
        .offset = 0,
        .clk_src = DAC_DIGI_CLK_SRC_APLL, // Using APLL as clock source to get a wider frequency range
        /* Assume the data in buffer is 'A B C D E F'
         * DAC_CHANNEL_MODE_SIMUL:
         *      - channel 0: A B C D E F
         *      - channel 1: A B C D E F
         * DAC_CHANNEL_MODE_ALTER:
         *      - channel 0: A C E
         *      - channel 1: B D F
         */
        .chan_mode = DAC_CHANNEL_MODE_SIMUL,
    };
    /* Allocate continuous channels */
    ESP_ERROR_CHECK(dac_continuous_new_channels(&cont_cfg, &dac_handle));

    /* Create a queue to transport the interrupt event data */
    QueueHandle_t que = xQueueCreate(10, sizeof(dac_event_data_t));
    assert(que);
    dac_event_callbacks_t cbs = {
        .on_convert_done = dac_on_convert_done_callback,
        .on_stop = NULL,
    };
    /* Must register the callback if using asynchronous writing */
    ESP_ERROR_CHECK(dac_continuous_register_event_callback(dac_handle, &cbs, que));

    /* Enable the continuous channels */
    ESP_ERROR_CHECK(dac_continuous_enable(dac_handle));
    ESP_LOGI(TAG, "DAC initialized success, DAC DMA is ready");

    // size_t audio_size = sizeof(audio_table);

    ESP_ERROR_CHECK(dac_continuous_start_async_writing(dac_handle));
    samplebuffer_readptr = samplebuffer_writeptr = 0;
    dac_write_data_asynchronously(dac_handle, que, (uint8_t *)samplebuffer, sizeof(samplebuffer));

    // dac_write_data_synchronously(dac_handle, (uint8_t *)samplebuffer, SAMPLEBUFFER_SIZE);
}
