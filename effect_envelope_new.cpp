/* Audio Library for Teensy 3.X
 * Copyright (c) 2017, Paul Stoffregen, paul@pjrc.com
 *
 * Development of this audio library was funded by PJRC.COM, LLC by sales of
 * Teensy and Audio Adaptor boards.  Please support PJRC's efforts to develop
 * open source software by purchasing Teensy or other PJRC products.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <Arduino.h>
#include "effect_envelope_new.h"

#define STATE_IDLE	0
#define STATE_DELAY	1
#define STATE_ATTACK	2
#define STATE_HOLD	3
#define STATE_DECAY	4
#define STATE_SUSTAIN	5
#define STATE_RELEASE	6
#define STATE_FORCED	7


void AudioEffectEnvelopeNew::update(void)
{
	audio_block_t *block;
	int16_t *p, *end;

	block = receiveWritable();
	if (!block) return;
	if (state == STATE_IDLE) {
		release(block);
		return;
	}
	p = block->data;
	end = p + AUDIO_BLOCK_SAMPLES;

	while (p < end) {
		// we only care about the state when completing a region
		if (count == 0) {
			if (state == STATE_ATTACK) {
				count = hold_count;
				if (count > 0) {
					state = STATE_HOLD;
					mult_hires = 0x40000000;
					inc_hires = 0;
				} else {
					state = STATE_DECAY;
					count = decay_count;
					inc_hires = (sustain_mult - 0x40000000) / (int32_t)count;
				}
				continue;
			} else if (state == STATE_HOLD) {
				state = STATE_DECAY;
				count = decay_count;
				inc_hires = (sustain_mult - 0x40000000) / (int32_t)count;
				continue;
			} else if (state == STATE_DECAY) {
				state = STATE_SUSTAIN;
				count = 0xFFFF;
				mult_hires = sustain_mult;
				inc_hires = 0;
			} else if (state == STATE_SUSTAIN) {
				count = 0xFFFF;
			} else if (state == STATE_RELEASE) {
			        //smoothed_hires = 0 ;
				state = STATE_IDLE;
				while (p < end) {
					*p++ = 0;
					*p++ = 0;
					*p++ = 0;
					*p++ = 0;
				}
				break;
			} else if (state == STATE_FORCED) {
				mult_hires = 0;
				//smoothed_hires = 0;
				count = delay_count;
				if (count > 0) {
					state = STATE_DELAY;
					inc_hires = 0;
				} else {
					state = STATE_ATTACK;
					count = attack_count;
					inc_hires = 0x40000000 / (int32_t)count;
				}
			} else if (state == STATE_DELAY) {
				state = STATE_ATTACK;
				count = attack_count;
				inc_hires = 0x40000000 / count;
				continue;
			}
		}


		// process 8 samples, using only mult and inc (16 bit resolution)
		for (int j = 0 ; j < 8 ; j++)
		{
		  int16_t sample = *p ;

		  mult_hires += inc_hires>>3 ;
		  smoothed_hires += (mult_hires - smoothed_hires) >> 6 ;
		
		  *p++ = ((smoothed_hires>>16) * sample) >> 14 ;
		}
		count--;
	}
	transmit(block);
	release(block);
}

