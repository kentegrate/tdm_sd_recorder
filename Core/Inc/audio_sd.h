/*
 * audio_sd.h
 *
 *  Created on: Mar 13, 2025
 *      Author: ken
 */

#ifndef INC_AUDIO_SD_H_
#define INC_AUDIO_SD_H_
#include "stdio.h"
#include <string.h>
#include "stm32l4xx_hal.h"


//#define WAV_WRITE_SAMPLE_COUNT 2048

void sd_card_init();
void start_recording(uint32_t fs, char* file_name, uint8_t n_channels);
void write2wave_file(uint8_t *data, uint16_t data_size);
void stop_recording();


#endif /* INC_AUDIO_SD_H_ */
