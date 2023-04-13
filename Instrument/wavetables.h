#ifndef _WAVETABLES_H_
#define _WAVETABLES_H_
#include "../utils.h"
#define WAVETABLE_SIZE 1024


int16_t wave_table_sine[WAVETABLE_SIZE];
int16_t wave_table_square[WAVETABLE_SIZE];
int16_t wave_table_saw[WAVETABLE_SIZE];
int16_t wave_table_triangle[WAVETABLE_SIZE];
int16_t wave_table_noise[WAVETABLE_SIZE];

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
        wave_table_noise[i] = a * (UINT16_MAX/16);
    }
}
void initWaveTables(void) {
    initWaveTable_sine();
    initWaveTable_square();
    initWaveTable_triangle();
    initWaveTable_saw();
    initWaveTable_noise();
}
#endif