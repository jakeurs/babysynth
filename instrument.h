
#define OSCILLATORBANK_MAX_OSC 8
#define ABS(value) (((value) >= 0) ? (value) : -(value))
#define CLAMP(value, min, max) ((value) < (min) ? (min) : (((value) > (max)) ? (max) : (value)))

typedef struct Oscillator
{
    int16_t *pWaveTable;
    uint32_t phase, phaseInc;
    int32_t frequencyMilliHz;
    /* frequency = (frequencyRatio10000/10000) * frequencyMilliHz/1000 */
    /* e.g., 5000 = .5 * oscillator frequency = 1 octive below */
    int32_t frequencyRatio10000; //
    int32_t tableStepSize;
} Oscillator;

typedef struct Sampler
{
    uint8_t *pSampleStart, *pSampleEnd, *pSamplePlayhead;
    uint8_t signal;
    bool playing;
    bool loop;
} Sampler;

typedef struct OscillatorBank
{
    Oscillator oscs[OSCILLATORBANK_MAX_OSC];
    uint8_t numOscillators;
    int32_t detuneMilliHz[OSCILLATORBANK_MAX_OSC];
    int32_t frequencyMilliHz;
} OscillatorBank;

typedef struct Ramp
{
    uint32_t durationSamples;
    float value;
    float oldValue;
    float startingValue;
    float targetValue;
    float multiplyPerSample;
    float addPerSample;
    bool active;
    bool exponential;
    bool steep;
    bool up;
} Ramp;

typedef enum ADSR_STATE
{
    ADSR_STATE_INACTIVE,
    ADSR_STATE_ATTACK,
    ADSR_STATE_DECAY,
    ADSR_STATE_SUSTAIN,
    ADSR_STATE_RELEASE
} ADSR_STATE;

typedef struct ADSR
{
    uint16_t envelope;
    uint16_t envelopeScaled;
    bool noteActive;
    bool noteStart;
    bool noteRelease;
    bool active;
    uint16_t min;
    uint16_t max;
    ADSR_STATE state;
    Ramp attack;
    Ramp decay;
    uint16_t sustainLevel;
    Ramp release;
} ADSR;

typedef struct LPFilter
{
    float a0;
    float a1;
    float a2;
    float b1;
    float b2;
    float Fc;
    int32_t frequencyMilliHz;
    int32_t signal;
    float Q;
    float z1;
    float z2;
} LPFilter;

typedef struct Mixer
{
    uint16_t in_1, in_2, out_1;
} Mixer;

typedef int32_t frequency_t;

#define MIDINOTETOFREQARRAYCOUNT 128
int32_t midiNoteToFreqMilliHz[MIDINOTETOFREQARRAYCOUNT] = {0};
#define SCALECOUNT 12
uint8_t scaleNoteToMidiNote[SCALECOUNT] =
    {
        40, // 0: E2
        43, // 1: G2
        45, // 2: A2
        47, // 3: B2
        50, // 4: D3
        52, // 5: E3
        55, // 6: G3
        57, // 7: A3
        59, // 8: B3
        62, // 9: D4
        64, // 10: E4
        67  // 11: G4
};

#define AMPLITUDE_CB_TO_LINEAR_TABLE_SIZE 1000 // -999 centiBels to 0
#define MAX_VOLUME INT16_MAX
uint16_t amplitude_cb_to_linear_table[AMPLITUDE_CB_TO_LINEAR_TABLE_SIZE];

#define SEQUENCER_NUM_INSTRUMENTS 6
#define SEQUENCER_NUM_BARS 4
#define SEQUENCER_BEATS_PER_BAR 4
#define SEQUENCER_SUBDIVISIONS_PER_BEAT 4
#define SEQUENCER_NUM_EVENTS SEQUENCER_NUM_BARS *SEQUENCER_BEATS_PER_BAR *SEQUENCER_SUBDIVISIONS_PER_BEAT

typedef enum SequencerCommand
{
    SEQUENCERCOMMAND_WAIT = 0,
    SEQUENCERCOMMAND_NOTEOFF,
    SEQUENCERCOMMAND_NOTEON
    /*
    SEQUENCERCOMMAND_NOTEON
    SequencerEvent.data is 8 bit unsigned char array with 4 elements
    0: midi note number
    1: note velocity (0 - 255) *not implemented*
    2: repetitions (0 - 254), 255 = infinite
    3: unused
    */
} SequencerCommand;

typedef struct SequencerEvent
{
    SequencerCommand command;
    uint8_t data[4];
} SequencerEvent;

typedef struct Instrument
{
    OscillatorBank oscBank0, oscBank1;
    ADSR adsr0_amp, adsr1_amp, adsr_filter;
    LPFilter lpFilter;
    int32_t signal, frequencyMilliHz;
    int32_t oscBank0FreqOffsetMilliHz, oscBank1FreqOffsetMilliHz;
    int32_t oscBank0FreqRatio10000, oscBank1FreqRatio10000;
    int16_t levelOsc04096, levelOsc14096, gainCB;
    uint8_t velocity;
} Instrument;
typedef struct InstrumentSequencer
{
    uint8_t instrumentId;
    Instrument *pInst;
    Sampler *pSampler;
    SequencerEvent events[SEQUENCER_NUM_EVENTS]; // 4/4 16th note subdivision
} InstrumentSequencer;

typedef struct MasterSequencer
{
    uint8_t bar;
    uint8_t beat;
    uint8_t subdivision;
    float timeMS;
    float sequenceLengthMS;
    uint32_t barLengthMS;
    uint32_t beatLengthMS;
    uint32_t subdivisionLengthMS;
    uint32_t bpm;
    uint32_t numInstruments;
    InstrumentSequencer *pInstruments[SEQUENCER_NUM_INSTRUMENTS];
} MasterSequencer;

void Debug_SequencerEvent(SequencerEvent *e);
void Debug_MasterSequencer_plot(MasterSequencer *pMasterSequencer);
int32_t OscillatorBankReadOffset(OscillatorBank *oscBank, uint16_t step);
void LPFilterCalculate(LPFilter *filter);
int32_t LPFilterTick(LPFilter *filter, int32_t inputSignal);
int32_t adjustSignalVolume(int32_t signal, int16_t volumeCB);
void OscillatorBankSetFrequency(OscillatorBank *oscBank, int32_t frequencyMilliHz);
void ADSRNoteEvent(ADSR *adsr, bool noteStart, bool noteRelease);
void ADSRTick(ADSR *adsr);
void OscillatorBankAdvance(OscillatorBank *oscBank, uint16_t step);
void Sampler_Play(Sampler *pSampler);
void Sampler_Stop(Sampler *pSampler);
void Instrument_SetVelocity(Instrument *pInst, uint8_t velocity);

void init_mtof()
{
    for (int i = 0; i < MIDINOTETOFREQARRAYCOUNT; i++)
    {
        float a = 440; // frequency of A
        float freqf = (a / 32) * pow(2, ((i - 9) / 12.0));
        midiNoteToFreqMilliHz[i] = (int32_t)floor(freqf * 1000);
    }
}
int32_t scaleNoteToFreqMilliHz(uint8_t note) // note is scale tone less than SCALECOUNT
{
    // ESP_LOGI(TAG, "scaleNoteToFreqMilliHz note = %u freq = %lu", note, midiNoteToFreqMilliHz[scaleNoteToMidiNote[note]]);
    return midiNoteToFreqMilliHz[scaleNoteToMidiNote[note]];
}

void Instrument_Tick(Instrument *pInst, uint16_t step)
{

    int32_t signal_osc0 = OscillatorBankReadOffset(&pInst->oscBank0, 0);
    int32_t signal_osc1 = OscillatorBankReadOffset(&pInst->oscBank1, 0);
    OscillatorBankAdvance(&pInst->oscBank0, 1);
    OscillatorBankAdvance(&pInst->oscBank1, 1);

    ADSRTick(&pInst->adsr0_amp);
    ADSRTick(&pInst->adsr1_amp);
    // ADSRTick(&pInst->adsr_filter);

    signal_osc0 = (signal_osc0 * pInst->adsr0_amp.envelope) / 4096;
    signal_osc1 = (signal_osc1 * pInst->adsr1_amp.envelope) / 4096;

    int32_t signal = 0;
    signal += (signal_osc0 * pInst->levelOsc04096) / 4096;
    signal += (signal_osc1 * pInst->levelOsc14096) / 4096;
    signal /= 2;
    signal = (signal * pInst->velocity) / 255;
    // signal = signal_osc0;
    /*
    if (step % 32 == 0)
    {
        pInst->lpFilter.frequencyMilliHz = (pInst->adsr_filter.envelope) * 244 + 50000; // 0 to 1000
        LPFilterCalculate(&pInst->lpFilter);
    }
    */

    signal = LPFilterTick(&pInst->lpFilter, signal);

    // signal = adjustSignalVolume(signal, pInst->gainCB);
    // pInst->signal = signal_osc0;
    pInst->signal = signal;
}
void Instrument_SetVelocity(Instrument *pInst, uint8_t velocity)
{
    pInst->velocity = velocity;
}
void Instrument_SetFreq(Instrument *pInst, int32_t frequencyMilliHz)
{
    assert(frequencyMilliHz >= 0);
    pInst->frequencyMilliHz = frequencyMilliHz;
    // int32_t targetFreq_0 = (uint32_t)((((int32_t)frequencyMilliHz + pInst->oscBank0FreqOffsetMilliHz) * pInst->oscBank0FreqRatio10000) / 10000);
    int32_t targetFreq_0 = ((frequencyMilliHz + pInst->oscBank0FreqOffsetMilliHz) * (int64_t)pInst->oscBank0FreqRatio10000) / 10000;
    // int32_t targetFreq_1 = (uint32_t)((((int32_t)frequencyMilliHz + pInst->oscBank1FreqOffsetMilliHz) * pInst->oscBank1FreqRatio10000) / 10000);
    int32_t targetFreq_1 = ((frequencyMilliHz + pInst->oscBank1FreqOffsetMilliHz) * (int64_t)pInst->oscBank1FreqRatio10000) / 10000;
    // ESP_LOGI(TAG, "Instrument_SetFreq Instrument: %p oscBank0 setting instrument frequency to %ld millihz (%ld + %ld) * %ld / 10000", pInst, targetFreq_0, frequencyMilliHz, pInst->oscBank0FreqOffsetMilliHz, pInst->oscBank0FreqRatio10000);
    // ESP_LOGI(TAG, "Instrument_SetFreq Instrument: %p oscBank1 setting instrument frequency to %ld millihz (%ld + %ld) * %ld / 10000", pInst, targetFreq_1, frequencyMilliHz, pInst->oscBank1FreqOffsetMilliHz, pInst->oscBank1FreqRatio10000);
    OscillatorBankSetFrequency(&pInst->oscBank0, targetFreq_0);
    OscillatorBankSetFrequency(&pInst->oscBank1, targetFreq_1);
}
void Instrument_NoteOn(Instrument *pInst)
{
    ADSRNoteEvent(&pInst->adsr0_amp, true, false);
    ADSRNoteEvent(&pInst->adsr1_amp, true, false);
    ADSRNoteEvent(&pInst->adsr_filter, true, false);
}
void Instrument_NoteOff(Instrument *pInst)
{
    ADSRNoteEvent(&pInst->adsr0_amp, false, true);
    ADSRNoteEvent(&pInst->adsr1_amp, false, true);
    ADSRNoteEvent(&pInst->adsr_filter, false, true);
}

void initAmplitudeLUT(void)
{
    for (int i = 0; i < AMPLITUDE_CB_TO_LINEAR_TABLE_SIZE; i++)
    {
        amplitude_cb_to_linear_table[i] = (uint16_t)(powf(10, -i / 200.0) * MAX_VOLUME);
        // ESP_LOGI(TAG, "amp table[%d]: %u", i, amplitude_cb_to_linear_table[i]);
    }
}
// volume in centibels e.g. -40dB = -400; 0dB = 0
int32_t adjustSignalVolume(int32_t signal, int16_t volumeCB)
{
    // ESP_LOGI(TAG, "Signal = %ld gain = %u final = %ld", signal, volumeNegCB, (signal * amplitude_cb_to_linear_table[volumeNegCB]) / MAX_VOLUME);
    if (volumeCB < -900)
        return 0;
    return (signal * amplitude_cb_to_linear_table[-volumeCB]) / MAX_VOLUME;
}

uint16_t phaseToWavetableIndex(uint32_t phase)
{
    return (uint64_t)WAVETABLE_SIZE * phase / UINT32_MAX;
}
void OscillatorSetWaveTable(Oscillator *osc, int16_t *pWaveTable)
{
    osc->pWaveTable = pWaveTable;
    osc->tableStepSize = UINT32_MAX / WAVETABLE_SIZE;
}

int32_t OscillatorReadOffset(Oscillator *osc, uint16_t step)
{
    /*
    static Oscillator *pOscOld = NULL;
    if (pOscOld != &osc->pWaveTable[0])
    {
        ESP_LOGI(TAG, "oscillator pointer changed old = %p new = %p", pOscOld, osc);
        pOscOld = &osc->pWaveTable[0];
    }
*/
    uint32_t phase = osc->phase + osc->phaseInc * step;
    uint16_t index1 = phase / osc->tableStepSize;
    uint16_t index2 = (index1 + 1) % WAVETABLE_SIZE;

    uint32_t frac1 = (osc->tableStepSize - (phase % osc->tableStepSize));
    uint32_t frac2 = (osc->tableStepSize - frac1);
    int32_t signal = ((int64_t)osc->pWaveTable[index1] * frac1 + (int64_t)osc->pWaveTable[index2] * frac2) / osc->tableStepSize;
    // int32_t signal = osc->pWaveTable[index1];

    // int32_t signal = osc->pWaveTable[0];

    /*
        ESP_LOGI(TAG,
                 "Reading Oscillator %p: frequency: %lu; phase %lu; phaseInc %lu; index1: %u (%d); index2: %u (%d); frac1: %lu; frac2: %lu; signal: %ld ",
                 (void *)&osc,
                 osc->frequencyMilliHz,
                 phase,
                 osc->phaseInc,
                 index1,
                 osc->pWaveTable[index1],
                 index2,
                 osc->pWaveTable[index2],
                 frac1,
                 frac2,
                 signal);
    */
    return signal;
    // return osc->pWaveTable[index1];
    //  osc->pWaveTable[phaseToWavetableIndex(osc->phase + osc->phaseInc * step)];
}

void OscillatorAdvance(Oscillator *osc, uint16_t step)
{
    // ESP_LOGI(TAG, "oscillator %p phase %lu phaseInc %lu value %ld", (void *)osc, osc->phase, osc->phaseInc, OscillatorReadOffset(osc, 0));
    osc->phase += osc->phaseInc * step;
}

void OscillatorSetFrequency(Oscillator *osc, int32_t frequencyMilliHz)
{
    assert(frequencyMilliHz >= 0);
    osc->frequencyMilliHz = frequencyMilliHz;
    // osc->phaseInc = ((uint64_t)UINT32_MAX / CONFIG_EXAMPLE_AUDIO_SAMPLE_RATE * frequencyMilliHz * osc->frequencyRatio10000) / (1000 * 10000);
    osc->phaseInc = ((uint64_t)UINT32_MAX / SAMPLE_RATE * (uint32_t)frequencyMilliHz) / 1000;
    // ESP_LOGI(TAG, "Setting frequency for oscillator %p  = %ld phaseInc = %lu", (void *)osc, osc->frequencyMilliHz, osc->phaseInc);
}

void OscillatorBankSetFrequency(OscillatorBank *oscBank, int32_t frequencyMilliHz)
{
    assert(frequencyMilliHz >= 0);
    oscBank->frequencyMilliHz = frequencyMilliHz;

    for (int i = 0; i < oscBank->numOscillators; i++)
    {
        // ESP_LOGI(TAG, "oscBank %p: setting oscillator %p frequency to %ld (%ld + %ld)", (void *)oscBank, (void *)&oscBank->oscs[i], frequencyMilliHz + oscBank->detuneMilliHz[i], frequencyMilliHz, oscBank->detuneMilliHz[i]);
        OscillatorSetFrequency(&oscBank->oscs[i], frequencyMilliHz + oscBank->detuneMilliHz[i]);
    }
}

void OscillatorBankSetDetune(OscillatorBank *oscBank, int32_t frequencyMilliHz)
{
    assert(frequencyMilliHz >= 0);
    int32_t detuneMilliHzStep = frequencyMilliHz / oscBank->numOscillators;
    int32_t startingDetuneMilliHz = oscBank->numOscillators / 2 * -detuneMilliHzStep;
    for (uint8_t i = 0; i < oscBank->numOscillators; i++)
    {
        oscBank->detuneMilliHz[i] = startingDetuneMilliHz + detuneMilliHzStep * i;
        // ESP_LOGI(TAG, "oscBank %p: detuneMilliHz[%u]: %ld", (void *)oscBank, i, oscBank->detuneMilliHz[i]);
    }
    OscillatorBankSetFrequency(oscBank, oscBank->frequencyMilliHz);
}

int32_t OscillatorBankReadOffset(OscillatorBank *oscBank, uint16_t step)
{
    int32_t signal = 0;
    for (int i = 0; i < oscBank->numOscillators; i++)
    {
        int32_t val = OscillatorReadOffset(&oscBank->oscs[i], step);
        // ESP_LOGI(TAG, "oscillator %p = %ld", (void *)&oscBank->oscs[i], val);
        signal += val;
    }
    signal = signal / oscBank->numOscillators; // signal scaled to int16_max

    // ESP_LOGI(TAG, "signal raw = %ld", signal);
    //  signal = signal / (oscBank->numOscillators / 2);
    //  ESP_LOGI(TAG, "signal scaled = %ld", signal);
    //   signal = (signal * 127) / (INT16_MAX / 5);

    //  printf("signal scaled: %ld\n", signal);

    // uint8_t out = (uint8_t)(((uint64_t)signal * 255) / INT16_MAX + 127);

    // ESP_LOGI(TAG, "signal out = %u", out);
    //  printf("signal out: %d\n", out);

    return signal;
}

void OscillatorBankAdvance(OscillatorBank *oscBank, uint16_t step)
{
    for (int i = 0; i < oscBank->numOscillators; i++)
    {
        // ESP_LOGI(TAG, "Advancing oscillator %d %u steps", i, step);
        OscillatorAdvance(&oscBank->oscs[i], step);
    }
}
void RampSetTime(Ramp *pRamp, uint32_t durationSamples)
{
    float distance = (float)pRamp->targetValue - (float)pRamp->startingValue;
    pRamp->durationSamples = durationSamples;

    if (pRamp->exponential)
    {
        distance = ABS(distance);
        pRamp->multiplyPerSample = pow((float)distance, 1.0 / durationSamples);
        // printf("distance = %f; 1.0 / durationSamples = %f; pow((float)distance, 1.0 / durationSamples) = %f\n", (float)distance, 1.0 / durationSamples, pow((float)distance, 1.0 / durationSamples));
        // printf("value = %f; target = %f; mult = %f\n", pRamp->value, pRamp->targetValue, pRamp->multiplyPerSample);
    }
    else
    {
        pRamp->addPerSample = (distance / durationSamples);
    }
}

void RampInitialize(Ramp *pRamp, uint32_t durationSamples, uint16_t startingValue, uint16_t targetValue, bool exponential, bool steep)
{
    startingValue = startingValue == 0 ? 1 : startingValue;
    float addPerSample = 0;
    float distance = (float)targetValue - (float)startingValue;
    pRamp->up = distance > 0 ? true : false;

    if (exponential)
    {
        distance = ABS(distance);
        pRamp->multiplyPerSample = pow((float)distance, 1.0 / durationSamples);

        if (!steep)
        {
            pRamp->oldValue = targetValue;
        }

        // printf("distance = %f; 1.0 / durationSamples = %f; pow((float)distance, 1.0 / durationSamples) = %f\n", (float)distance, 1.0 / durationSamples, pow((float)distance, 1.0 / durationSamples));
        // printf("value = %f; target = %f; mult = %f\n", pRamp->value, pRamp->targetValue, pRamp->multiplyPerSample);
    }
    else
    {
        addPerSample = (distance / durationSamples);
    }

    pRamp->durationSamples = durationSamples;
    pRamp->targetValue = targetValue;
    pRamp->startingValue = startingValue;
    pRamp->addPerSample = addPerSample;
    pRamp->steep = steep;
    pRamp->exponential = exponential;

    // pRamp->value = startingValue;
    //  printf("value = %f; target = %f; mult = %f\n", pRamp->value, pRamp->targetValue, pRamp->multiplyPerSample);
}

void RampTo(Ramp *pRamp, uint16_t startingValue, uint16_t targetValue)
{
    startingValue = startingValue == 0 ? 1 : startingValue;
    pRamp->value = (float)startingValue;
    if (pRamp->exponential)
    {
        if (!pRamp->steep)
        {
            pRamp->oldValue = (float)targetValue;
            pRamp->oldValue = pRamp->oldValue == 0 ? 1 : pRamp->oldValue;
        }
        else
        {
            pRamp->oldValue = (float)startingValue;
        }
    }
    pRamp->targetValue = (float)targetValue;
    pRamp->startingValue = (float)startingValue;
    pRamp->active = true;
}

uint16_t RampTick(Ramp *pRamp)
{
    // printf("value = %u; target = %u; mult = %d\n", pRamp->value, pRamp->targetValue, pRamp->multiplyPerSampleTimes1000);
    if (!pRamp->active)
        return 0;

    if (pRamp->exponential)
    {
        if (pRamp->steep)
        {
            pRamp->oldValue = pRamp->value;
            if (pRamp->up)
            {
                pRamp->value = (pRamp->value * pRamp->multiplyPerSample);
            }
            else
            {
                pRamp->value = pRamp->value / pRamp->multiplyPerSample;
                // printf("(pRamp->value * 100) = %u; (pRamp->value * 100) / pRamp->multiplyPerSampleTimes100= %u;\n", (pRamp->value * 100), (pRamp->value * 100) / pRamp->multiplyPerSampleTimes100);
                // if (pRamp->oldValue == pRamp->value) pRamp->value--;
            }
        }
        else
        {
            pRamp->value = pRamp->oldValue;
            if (pRamp->up)
            {
                pRamp->oldValue = pRamp->oldValue / pRamp->multiplyPerSample;
                // printf("pRamp->oldValue = %f; \n", pRamp->oldValue);
                // if (pRamp->oldValue == pRamp->value) pRamp->oldValue++;
                pRamp->value = pRamp->targetValue - pRamp->oldValue;
                // printf("pRamp->value = %f; \n", pRamp->value);
                // if (oldValue == pRamp->value) pRamp->value++;
            }
            else
            {
                pRamp->oldValue = pRamp->oldValue * pRamp->multiplyPerSample;
                // printf("pRamp->oldValue = %f; pRamp->startingValue = %f; \n", pRamp->oldValue, pRamp->startingValue);
                // if (pRamp->oldValue == pRamp->value) pRamp->oldValue++;
                pRamp->value = pRamp->startingValue - pRamp->oldValue;
                // printf("pRamp->value = %f; \n", pRamp->value);
                // if (oldValue == pRamp->value) pRamp->value++;
            }
        }
        // printf("pRamp->value * pRamp->multiplyPerSampleTimes1000 = %u; (pRamp->value * pRamp->multiplyPerSampleTimes1000) / 1000 = %u;\n", pRamp->value * pRamp->multiplyPerSampleTimes1000, (pRamp->value * pRamp->multiplyPerSampleTimes1000) / 1000);
        // printf("value = %u; target = %u; mult = %d\n", pRamp->value, pRamp->targetValue, pRamp->multiplyPerSampleTimes1000);
    }
    else
    {
        pRamp->value += pRamp->addPerSample;
        // printf("value = %u; target = %u; add = %d\n", pRamp->value, pRamp->targetValue, pRamp->addPerSample);
    }

    // overshoot
    if ((pRamp->up && pRamp->value >= (pRamp->targetValue - 0.1)) ||
        (!pRamp->up && pRamp->value <= (pRamp->targetValue + 0.1)))
    {
        pRamp->value = pRamp->targetValue;
        pRamp->active = false;
    }
    return pRamp->value;
}

void testRamp(int length, int start, int stop, bool exp, bool steep)
{
    Ramp r;
    RampInitialize(&r, length, start, stop, exp, steep);
    RampTo(&r, start, stop);
    int i = 0;
    while (r.active == true)
    {
        uint32_t v = RampTick(&r);
        for (int j = 0; j < v; j += 16)
        {
            printf("-");
        }
        printf("|  (%d)\n", i++);
    }
}

void ADSRNoteEvent(ADSR *adsr, bool noteStart, bool noteRelease)
{
    if (noteStart)
    {
        // ESP_LOGI(TAG, "Note DOWN");
        adsr->noteStart = true;
    }
    else if (noteRelease)
    {
        // ESP_LOGI(TAG, "Note UP");
        adsr->noteRelease = true;
    }
}

void ADSRTick(ADSR *adsr)
{
    if (adsr->noteStart)
    { // first tick with key pressed
        adsr->noteStart = false;
        adsr->noteActive = true;
        adsr->state = ADSR_STATE_ATTACK;
        // ESP_LOGI(TAG, "ATTACK to %f envelope = %u ", adsr->attack.targetValue, adsr->envelope);
        RampTo(&adsr->attack, adsr->envelope, adsr->attack.targetValue);
    }
    if (adsr->noteRelease)
    {
        adsr->noteRelease = false;
        adsr->noteActive = false;
    }
    if (adsr->state == ADSR_STATE_INACTIVE) // off
    {
        return;
    }
    if (adsr->state == ADSR_STATE_ATTACK) // attack
    {
        adsr->envelope = RampTick(&adsr->attack);
        if (!adsr->attack.active || (adsr->envelope > adsr->sustainLevel && !adsr->noteActive))
        {
            // ESP_LOGI(TAG, "DECAY to %f envelope = %u ", adsr->decay.targetValue, adsr->envelope);
            adsr->state = ADSR_STATE_DECAY;
            RampTo(&adsr->decay, adsr->envelope, adsr->decay.targetValue);
        }
        else if (!adsr->noteActive)
        {
            // ESP_LOGI(TAG, "RELEASE to %f envelope = %u ", adsr->release.targetValue, adsr->envelope);
            adsr->state = ADSR_STATE_RELEASE;
            RampTo(&adsr->release, adsr->envelope, adsr->release.targetValue);
        }
    }
    else if (adsr->state == ADSR_STATE_DECAY) // decay
    {
        adsr->envelope = RampTick(&adsr->decay);
        if (!adsr->decay.active)
        {
            // ESP_LOGI(TAG, "SUSTAIN at %u envelope = %u ", adsr->sustainLevel, adsr->envelope);
            adsr->state = ADSR_STATE_SUSTAIN;
        }
    }
    else if (adsr->state == ADSR_STATE_SUSTAIN) // sustain
    {
        adsr->envelope = adsr->sustainLevel;
        if (!adsr->noteActive)
        {
            // ESP_LOGI(TAG, "RELEASE to %f envelope = %u ", adsr->release.targetValue, adsr->envelope);
            adsr->state = ADSR_STATE_RELEASE;
            RampTo(&adsr->release, adsr->envelope, adsr->release.targetValue);
        }
    }
    else if (adsr->state == ADSR_STATE_RELEASE) // release
    {

        adsr->envelope = RampTick(&adsr->release);
        if (!adsr->release.active)
        {
            // ESP_LOGI(TAG, "RELEASE FINISHED envelope = %u ", adsr->envelope);
            adsr->state = ADSR_STATE_INACTIVE;
        }
    }
    /*
    if (oldEnvelope - adsr->envelope > 5)
    {
        ESP_LOGI(TAG, "old envelope = %u; envelope = %u ", oldEnvelope, adsr->envelope);
    }*/
}

void plotRamps(ADSR *adsr)
{
    static char s[64];
    static char c = 'a';
    if (adsr->state == ADSR_STATE_INACTIVE)
        return;
    if (adsr->state == ADSR_STATE_ATTACK)
        c = 'a';
    else if (adsr->state == ADSR_STATE_DECAY)
        c = 'd';
    else if (adsr->state == ADSR_STATE_SUSTAIN)
        c = 's';
    else if (adsr->state == ADSR_STATE_RELEASE)
        c = 'r';
    for (int j = 0; j < 63; j++)
    {
        s[j] = j < adsr->envelope / 64 ? c : ' ';
    }
    s[63] = '\0';
    ESP_LOGI(TAG, "%s (%u)", s, adsr->envelope);
}

void ADSRInitialize(ADSR *adsr, uint16_t sustainLevel)
{
    adsr->envelope = 0;
    adsr->noteActive = false;
    adsr->noteStart = false;
    adsr->noteRelease = false;
    adsr->active = false;
    adsr->min = 0;
    adsr->max = 4095;
    adsr->state = ADSR_STATE_INACTIVE;

    adsr->sustainLevel = sustainLevel;
    RampInitialize(&adsr->attack, 200, 0, 4096, false, false);
    RampInitialize(&adsr->decay, 20, 4096, adsr->sustainLevel, false, false);
    RampInitialize(&adsr->release, 400, adsr->sustainLevel, 0, false, true);
}

void LPFilterCalculate(LPFilter *filter)
{
    filter->Fc = (float)filter->frequencyMilliHz / (1000.0 * SAMPLE_RATE);
    filter->Q = filter->Q <= 0.1 ? 0.1 : filter->Q;
    float K = tanf(3.14159 * filter->Fc);
    float norm = 1 / (1 + K / filter->Q + K * K);
    filter->a0 = K * K * norm;
    filter->a1 = 2 * filter->a0;
    filter->a2 = filter->a0;
    filter->b1 = 2 * (K * K - 1) * norm;
    filter->b2 = (1 - K / filter->Q + K * K) * norm;
}

int32_t LPFilterTick(LPFilter *filter, int32_t inputSignal)
{
    float in = (float)inputSignal / (float)INT16_MAX;
    float out = in * filter->a0 + filter->z1;
    filter->z1 = in * filter->a1 + filter->z2 - filter->b1 * out;
    filter->z2 = in * filter->a2 - filter->b2 * out;
    filter->signal = (int32_t)(out * INT16_MAX);
    return filter->signal;
}

void LPFilterDebug(LPFilter *filter)
{
    ESP_LOGI(TAG, "frequencyMilliHz = %lu Q = %.2f signal = %lu a0 = %.2f a1 = %.2f a2 = %.2f b1 = %.2f b2 = %.2f Fc = %.2f z1 = %.2f z2 = %.2f", filter->frequencyMilliHz, filter->Q, filter->signal, filter->a0, filter->a1, filter->a2, filter->b1, filter->b2, filter->Fc, filter->z1, filter->z2);
}

void InstrumentSequencer_InitEvent(SequencerEvent *e)
{
    e->command = SEQUENCERCOMMAND_WAIT;
    for (int j = 0; j < 4; j++)
    {
        e->data[j] = 0;
    }
}
void InstrumentSequencer_Init(InstrumentSequencer *pInstrumentSequencer, uint8_t instrumentId)
{
    pInstrumentSequencer->instrumentId = instrumentId;
    for (int i = 0; i < SEQUENCER_NUM_EVENTS; i++)
    {
        InstrumentSequencer_InitEvent(&pInstrumentSequencer->events[i]);
    }
}
void InstrumentSequencer_NoteOn(InstrumentSequencer *pInstrumentSequencer, uint8_t bar, uint8_t beat, uint8_t subdivision, uint8_t midiNoteNum, uint8_t velocity, uint8_t repetitions)
{
    unsigned int index = bar * SEQUENCER_BEATS_PER_BAR * SEQUENCER_SUBDIVISIONS_PER_BEAT + beat * SEQUENCER_SUBDIVISIONS_PER_BEAT + subdivision;
    assert(index < SEQUENCER_NUM_EVENTS);
    pInstrumentSequencer->events[index].command = SEQUENCERCOMMAND_NOTEON;
    pInstrumentSequencer->events[index].data[0] = midiNoteNum;
    pInstrumentSequencer->events[index].data[1] = velocity;
    pInstrumentSequencer->events[index].data[2] = repetitions;
}
void InstrumentSequencer_NoteOff(InstrumentSequencer *pInstrumentSequencer, uint8_t bar, uint8_t beat, uint8_t subdivision, uint8_t repetitions)
{
    unsigned int index = bar * SEQUENCER_BEATS_PER_BAR * SEQUENCER_SUBDIVISIONS_PER_BEAT + beat * SEQUENCER_SUBDIVISIONS_PER_BEAT + subdivision;
    assert(index < SEQUENCER_NUM_EVENTS);
    pInstrumentSequencer->events[index].command = SEQUENCERCOMMAND_NOTEOFF;
    pInstrumentSequencer->events[index].data[2] = repetitions;
}
void InstrumentSequencer_setPosition(InstrumentSequencer *pInstrumentSequencer, uint8_t bar, uint8_t beat, uint8_t subdivision)
{
    unsigned int index = bar * SEQUENCER_BEATS_PER_BAR * SEQUENCER_SUBDIVISIONS_PER_BEAT + beat * SEQUENCER_SUBDIVISIONS_PER_BEAT + subdivision;
    assert(index < SEQUENCER_NUM_EVENTS);
    SequencerEvent *pEvent = &pInstrumentSequencer->events[index];

    if (pEvent->command == SEQUENCERCOMMAND_WAIT)
    {
        return;
    }

    // printf("[ID %u] ", pInstrumentSequencer->instrumentId);
    // Debug_SequencerEvent(&pInstrumentSequencer->events[index]);
    if (pEvent->data[2] == 0)
    {
        InstrumentSequencer_InitEvent(pEvent);
        return;
    }
    else if (pEvent->data[2] != 255) // repetitions
    {
        pEvent->data[2]--;
    }

    if (pEvent->command == SEQUENCERCOMMAND_NOTEON)
    {

        assert(pEvent->data[0] < MIDINOTETOFREQARRAYCOUNT);
        if (pInstrumentSequencer->pInst != NULL)
        {
            // ESP_LOGI(TAG, "InstrumentSequencer_setPosition [SEQUENCERCOMMAND_NOTEON] freq[%u]: %lu, velocity: %u", pEvent->data[0], midiNoteToFreqMilliHz[pEvent->data[0]], pEvent->data[1]);
            Instrument_SetFreq(pInstrumentSequencer->pInst, midiNoteToFreqMilliHz[pEvent->data[0]]);
            Instrument_SetVelocity(pInstrumentSequencer->pInst, pEvent->data[1]);
            Instrument_NoteOn(pInstrumentSequencer->pInst);
        }
        else if (pInstrumentSequencer->pSampler != NULL)
        {
            Sampler_Play(pInstrumentSequencer->pSampler);
        }
    }
    else if (pEvent->command == SEQUENCERCOMMAND_NOTEOFF)
    {
        if (pInstrumentSequencer->pInst != NULL)
        {
            Instrument_NoteOff(pInstrumentSequencer->pInst);
        }
        else if (pInstrumentSequencer->pSampler != NULL)
        {
            Sampler_Stop(pInstrumentSequencer->pSampler);
        }
    }
}
void InstrumentSequencer_RegisterInstrument(InstrumentSequencer *pInstrumentSeq, Instrument *pInst)
{
    pInstrumentSeq->pSampler = NULL;
    pInstrumentSeq->pInst = pInst;
}
void InstrumentSequencer_RegisterSampler(InstrumentSequencer *pInstrumentSeq, Sampler *pSampler)
{
    pInstrumentSeq->pInst = NULL;
    pInstrumentSeq->pSampler = pSampler;
}
void MasterSequencer_Init(MasterSequencer *pMasterSequencer, uint8_t bpm)
{
    pMasterSequencer->bar = SEQUENCER_NUM_BARS;
    pMasterSequencer->beat = 0;
    pMasterSequencer->subdivision = 0;
    pMasterSequencer->timeMS = 0;
    pMasterSequencer->sequenceLengthMS = (float)(SEQUENCER_NUM_BARS * SEQUENCER_BEATS_PER_BAR) / bpm * 60.0 * 1000.0;
    pMasterSequencer->barLengthMS = (float)(SEQUENCER_BEATS_PER_BAR) / bpm * 60.0 * 1000.0;
    pMasterSequencer->beatLengthMS = 1.0 / bpm * 60.0 * 1000.0;
    pMasterSequencer->subdivisionLengthMS = 1.0 / (bpm * SEQUENCER_SUBDIVISIONS_PER_BEAT) * 60.0 * 1000.0;
    pMasterSequencer->bpm = bpm;
    pMasterSequencer->numInstruments = 0;
}
void MasterSequencer_AddInstrument(MasterSequencer *pMasterSequencer, InstrumentSequencer *pInstrumentSequencer)
{
    assert(pMasterSequencer->numInstruments + 1 <= SEQUENCER_NUM_INSTRUMENTS);
    pMasterSequencer->pInstruments[pMasterSequencer->numInstruments] = pInstrumentSequencer;
    pMasterSequencer->numInstruments++;
}
void MasterSequencer_Step(MasterSequencer *pMasterSequencer, float elapsedMillis)
{
    unsigned int newBar, newBeat, newSubdivision;

    pMasterSequencer->timeMS = pMasterSequencer->timeMS + elapsedMillis;
    while (pMasterSequencer->timeMS > pMasterSequencer->sequenceLengthMS)
        pMasterSequencer->timeMS -= pMasterSequencer->sequenceLengthMS;

    uint16_t timeMSint = pMasterSequencer->timeMS;

    newBar = timeMSint / pMasterSequencer->barLengthMS;
    newBeat = (timeMSint / pMasterSequencer->beatLengthMS) % SEQUENCER_BEATS_PER_BAR;
    newSubdivision = (timeMSint / pMasterSequencer->subdivisionLengthMS) % SEQUENCER_SUBDIVISIONS_PER_BEAT;

    if (newBar != pMasterSequencer->bar || newBeat != pMasterSequencer->beat || newSubdivision != pMasterSequencer->subdivision)
    {
        pMasterSequencer->bar = newBar;
        pMasterSequencer->beat = newBeat;
        pMasterSequencer->subdivision = newSubdivision;
        // Debug_MasterSequencer_plot(pMasterSequencer); // todo: why does play head disappear
        for (int i = 0; i < pMasterSequencer->numInstruments; i++)
        {
            InstrumentSequencer_setPosition(pMasterSequencer->pInstruments[i], newBar, newBeat, newSubdivision);
        }
    }
}

void Debug_SequencerEvent(SequencerEvent *e)
{
    printf("SequencerEvent %u: %u %u %u %u\n", e->command, e->data[0], e->data[1], e->data[2], e->data[3]);
}

void Debug_InstrumentSequencer_plot(InstrumentSequencer *pInstrumentSequencer)
{
    bool noteDown = false;
    printf("[ID %u] ", pInstrumentSequencer->instrumentId);
    for (int i = 0; i < SEQUENCER_NUM_EVENTS; i++)
    {
        if (i % (SEQUENCER_BEATS_PER_BAR * SEQUENCER_SUBDIVISIONS_PER_BEAT) == 0)
            printf("|");
        if (pInstrumentSequencer->events[i].command == SEQUENCERCOMMAND_NOTEON)
            noteDown = true;
        else if (pInstrumentSequencer->events[i].command == SEQUENCERCOMMAND_NOTEOFF)
            noteDown = false;
        if (noteDown)
            printf("%c", i % SEQUENCER_SUBDIVISIONS_PER_BEAT == 0 ? 'X' : 'x');
        else if (i % SEQUENCER_SUBDIVISIONS_PER_BEAT == 0)
            printf("-");
        else
            printf("_");
    }
    printf("|\n");
}
void Debug_InstrumentSequencer_list(InstrumentSequencer *pInstrumentSequencer)
{
    for (int i = 0; i < SEQUENCER_NUM_EVENTS; i++)
    {
        Debug_SequencerEvent(&pInstrumentSequencer->events[i]);
    }
}
void Debug_MasterSequencer_plot(MasterSequencer *pMasterSequencer)
{
    // printf("MasterSequencer %u bpm (%ums). %d bars (%ums) with %d beats (%ums) per bar and %d subdivisions (%ums) per beat\n", pMasterSequencer->bpm, pMasterSequencer->sequenceLengthMS, SEQUENCER_NUM_BARS,pMasterSequencer->barLengthMS, SEQUENCER_BEATS_PER_BAR, pMasterSequencer->beatLengthMS, SEQUENCER_SUBDIVISIONS_PER_BEAT, pMasterSequencer->subdivisionLengthMS);
    // printf("[%u:%u:%u]\n",pMasterSequencer->bar, pMasterSequencer->beat, pMasterSequencer->subdivision);
    printf("       ");
    for (int bar = 0; bar < SEQUENCER_NUM_BARS; bar++)
    {
        printf("|");
        for (int beat = 0; beat < SEQUENCER_BEATS_PER_BAR; beat++)
        {
            for (int sub = 0; sub < SEQUENCER_SUBDIVISIONS_PER_BEAT; sub++)
            {
                if (bar == pMasterSequencer->bar && beat == pMasterSequencer->beat && sub == pMasterSequencer->subdivision)
                {
                    printf("V");
                }
                else
                {
                    printf(".");
                }
            }
        }
    }
    printf("|\n");
    for (int i = 0; i < pMasterSequencer->numInstruments; i++)
    {
        Debug_InstrumentSequencer_plot(pMasterSequencer->pInstruments[i]);
    }
}

// read the length of the sample from the first two bytes
void Sampler_Init(Sampler *pSampler, uint8_t *pSampleStart, bool loop)
{

    printf("pSampler: %p\n", pSampler);
    printf("pSampler->pSampleStart: %p\n", pSampler->pSampleStart);
    printf("pSampleStart[0]: %u\n", pSampleStart[0]);
    printf("pSampleStart[1]: %u\n", pSampleStart[1]);
    uint16_t sampleLength = (pSampleStart[1] << 8) + pSampleStart[0];
    pSampler->pSampleStart = pSampler->pSamplePlayhead = pSampleStart + 2;
    pSampler->pSampleEnd = pSampleStart + sampleLength;
    pSampler->signal = 0;
    pSampler->playing = false;
    pSampler->loop = loop;
}
void Sampler_Tick(Sampler *pSampler)
{
    if (pSampler->playing)
    {
        pSampler->signal = *pSampler->pSamplePlayhead;
        pSampler->pSamplePlayhead++;
        if (pSampler->pSamplePlayhead > pSampler->pSampleEnd)
        {
            pSampler->pSamplePlayhead = pSampler->pSampleStart;
            if (!pSampler->loop)
            {
                pSampler->playing = false;
                pSampler->signal = 127;
            }
        }
    }
}
void Sampler_Play(Sampler *pSampler)
{
    pSampler->pSamplePlayhead = pSampler->pSampleStart;
    pSampler->playing = true;
}
void Sampler_Stop(Sampler *pSampler)
{
    pSampler->pSamplePlayhead = pSampler->pSampleStart;
    pSampler->playing = false;
    pSampler->signal = 127; // could make pops - todo: add state machine to manage partial playback stopping
}

void Sampler_Debug(Sampler *pSampler, bool plot)
{
    if (plot)
    {
        uint8_t val = pSampler->signal / 8;
        char bar[32];
        for (uint8_t i = 0; i < 32; i++)
        {
            bar[i] = i < val ? '-' : i == val ? 'X'
                                              : ' ';
        }
        printf("%s\n", bar);
    }
    else
    {
        uint16_t sampleLength = pSampler->pSampleEnd - pSampler->pSampleStart;
        printf("Sampler Debug [%p] -- pSampleStart: [%p] length: %u pSampleEnd: [%p] pSamplePlayhead: [%p] signal: %u playing: %d\n", (void *)pSampler, (void *)pSampler->pSampleStart, sampleLength, (void *)pSampler->pSampleEnd, (void *)pSampler->pSamplePlayhead, pSampler->signal, pSampler->playing);
    }
}