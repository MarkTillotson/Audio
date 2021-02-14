/* Audio Library for Teensy 3.X
 * Copyright (c) 2018, Paul Stoffregen, paul@pjrc.com
 * enhancement 2021, Mark Tillotson
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

#if !defined(__IMXRT1052__) && !defined(__IMXRT1062__)
 
#include <Arduino.h>
#include "input_pdm_raw.h"
//#include "utility/dspinst.h"

// Decrease this for more mic gain, increase for range to accommodate loud sounds
//#define RSHIFT  2

// Pulse Density Modulation (PDM) is a tech trade-off of questionable value.
// The only advantage is your delta-sigma based ADC can be less expensive,
// since it can omit the digital low-pass filter.  But it limits the ADC
// to a single bit modulator, and it imposes the filtering requirement onto
// your microcontroller.  Generally digital filtering is much less expensive
// to implement with dedicated digital logic than firmware in a general
// purpose microcontroller.  PDM probably makes more sense with an ASIC or
// highly integrated SoC, or maybe even with a "real" DSP chip.  Using a
// microcontroller, maybe not so much?
//
// This code imposes considerable costs.  It consumes 39% of the CPU time
// when running at 96 MHz, and uses 2104 bytes of RAM for buffering and 32768
// bytes of flash for a large table lookup to optimize the filter computation.
//
// On the plus side, this filter is a 512 tap FIR with approximately +/- 1 dB
// gain flatness to 10 kHz bandwidth.  That won't impress any audio enthusiasts,
// but its performance should be *much* better than the rapid passband rolloff
// of Cascaded Integrator Comb (CIC) or moving average filters.

DMAMEM __attribute__((aligned(32))) static uint32_t pdm_buffer[AUDIO_BLOCK_SAMPLES*4];
static uint32_t leftover[14];
audio_block_t * AudioInputPDMRaw::block_left = NULL;
bool AudioInputPDMRaw::update_responsibility = false;
DMAChannel AudioInputPDMRaw::dma(false);

// MCLK needs to be 48e6 / 1088 * 256 = 11.29411765 MHz -> 44.117647 kHz sample rate
//
#if F_CPU == 96000000 || F_CPU == 48000000 || F_CPU == 24000000
  // PLL is at 96 MHz in these modes
  #define MCLK_MULT 2
  #define MCLK_DIV  17
#elif F_CPU == 72000000
  #define MCLK_MULT 8
  #define MCLK_DIV  51
#elif F_CPU == 120000000
  #define MCLK_MULT 8
  #define MCLK_DIV  85
#elif F_CPU == 144000000
  #define MCLK_MULT 4
  #define MCLK_DIV  51
#elif F_CPU == 168000000
  #define MCLK_MULT 8
  #define MCLK_DIV  119
#elif F_CPU == 180000000
  #define MCLK_MULT 16
  #define MCLK_DIV  255
  #define MCLK_SRC  0
#elif F_CPU == 192000000
  #define MCLK_MULT 1
  #define MCLK_DIV  17
#elif F_CPU == 216000000
  #define MCLK_MULT 12
  #define MCLK_DIV  17
  #define MCLK_SRC  1
#elif F_CPU == 240000000
  #define MCLK_MULT 2
  #define MCLK_DIV  85
  #define MCLK_SRC  0
#elif F_CPU == 256000000
  #define MCLK_MULT 12
  #define MCLK_DIV  17
  #define MCLK_SRC  1
#elif F_CPU == 16000000
  #define MCLK_MULT 12
  #define MCLK_DIV  17
#else
  #error "This CPU Clock Speed is not supported by the Audio library";
#endif

#ifndef MCLK_SRC
#if F_CPU >= 20000000
  #define MCLK_SRC  3  // the PLL
#else
  #define MCLK_SRC  0  // system clock
#endif
#endif

void AudioInputPDMRaw::begin(void)
{
	dma.begin(true); // Allocate the DMA channel first

	SIM_SCGC6 |= SIM_SCGC6_I2S;
	SIM_SCGC7 |= SIM_SCGC7_DMA;
	SIM_SCGC6 |= SIM_SCGC6_DMAMUX;

        // enable MCLK output
        I2S0_MCR = I2S_MCR_MICS(MCLK_SRC) | I2S_MCR_MOE;
        while (I2S0_MCR & I2S_MCR_DUF) ;
        I2S0_MDR = I2S_MDR_FRACT((MCLK_MULT-1)) | I2S_MDR_DIVIDE((MCLK_DIV-1));

        // configure transmitter
        I2S0_TMR = 0;
        I2S0_TCR1 = I2S_TCR1_TFW(1);  // watermark at half fifo size
        I2S0_TCR2 = I2S_TCR2_SYNC(0) | I2S_TCR2_BCP | I2S_TCR2_MSEL(1)
                | I2S_TCR2_BCD | I2S_TCR2_DIV(1);
        I2S0_TCR3 = I2S_TCR3_TCE;
        I2S0_TCR4 = I2S_TCR4_FRSZ(1) | I2S_TCR4_SYWD(31) | I2S_TCR4_MF
                | I2S_TCR4_FSE | I2S_TCR4_FSP | I2S_TCR4_FSD;
        I2S0_TCR5 = I2S_TCR5_WNW(31) | I2S_TCR5_W0W(31) | I2S_TCR5_FBT(31);

        // configure receiver (sync'd to transmitter clocks)
        I2S0_RMR = 0;
        I2S0_RCR1 = I2S_RCR1_RFW(2);
        I2S0_RCR2 = I2S_RCR2_SYNC(1) | I2S_TCR2_BCP | I2S_RCR2_MSEL(1)
                | I2S_RCR2_BCD | I2S_RCR2_DIV(1);
        I2S0_RCR3 = I2S_RCR3_RCE;
        I2S0_RCR4 = I2S_RCR4_FRSZ(1) | I2S_RCR4_SYWD(31) | I2S_RCR4_MF
                /* | I2S_RCR4_FSE */ | I2S_RCR4_FSP | I2S_RCR4_FSD;
        I2S0_RCR5 = I2S_RCR5_WNW(31) | I2S_RCR5_W0W(31) | I2S_RCR5_FBT(31);

        CORE_PIN9_CONFIG  = PORT_PCR_MUX(6); // pin  9, PTC3, I2S0_TX_BCLK
	CORE_PIN13_CONFIG = PORT_PCR_MUX(4); // pin 13, PTC5, I2S0_RXD0

#if defined(KINETISK)
	dma.TCD->SADDR = &I2S0_RDR0;
	dma.TCD->SOFF = 0;
	dma.TCD->ATTR = DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_DSIZE(2);
	dma.TCD->NBYTES_MLNO = 4;
	dma.TCD->SLAST = 0;
	dma.TCD->DADDR = pdm_buffer;
	dma.TCD->DOFF = 4;
	dma.TCD->CITER_ELINKNO = sizeof(pdm_buffer) / 4;
	dma.TCD->DLASTSGA = -sizeof(pdm_buffer);
	dma.TCD->BITER_ELINKNO = sizeof(pdm_buffer) / 4;
	dma.TCD->CSR = DMA_TCD_CSR_INTHALF | DMA_TCD_CSR_INTMAJOR;
#endif
	dma.triggerAtHardwareEvent(DMAMUX_SOURCE_I2S0_RX);
	update_responsibility = update_setup();
	dma.enable();

	I2S0_RCSR |= I2S_RCSR_RE | I2S_RCSR_BCE | I2S_RCSR_FRDE | I2S_RCSR_FR;
	I2S0_TCSR |= I2S_TCSR_TE | I2S_TCSR_BCE; // TX clock enable, because sync'd to TX
	dma.attachInterrupt(isr);
}

void AudioInputPDMRaw::isr(void)
{
	uint32_t daddr;
	const uint32_t *src;
	audio_block_t *left;

	//digitalWriteFast(3, HIGH);
#if defined(KINETISK)
	daddr = (uint32_t)(dma.TCD->DADDR);
#endif
	dma.clearInterrupt();

	if (daddr < (uint32_t)pdm_buffer + sizeof(pdm_buffer) / 2) {
		// DMA is receiving to the first half of the buffer
		// need to remove data from the second half
		src = pdm_buffer + AUDIO_BLOCK_SAMPLES*2;
	} else {
		// DMA is receiving to the second half of the buffer
		// need to remove data from the first half
		src = pdm_buffer;
	}
	if (update_responsibility) AudioStream::update_all();
	left = block_left;
	if (left != NULL) {
		int16_t *dest = left->data;
		for (unsigned int i = 0 ; i < AUDIO_BLOCK_SAMPLES*2 ; i += 2)
		{
		  uint32_t word0 = src[i] ;
		  uint8_t s0 = count (word0 >> 16) ;
		  uint8_t s1 = count (word0 & 0xFFFF) ;
		  uint32_t word1 = src[i+1] ;
		  uint8_t s2 = count (word1 >> 16) ;
		  uint8_t s3 = count (word1 & 0xFFFF) ;
		  *dest++ = (s0 << 12) | (s1 << 8) | (s2 << 4) | s3 ;
		}
		//left->data[0] = 0x7FFF;
	}
	//digitalWriteFast(3, LOW);
}

uint8_t count (uint16_t bits)
{
  int8_t c0 = discrep_table [bits >> 8] ;
  int8_t c1 = discrep_table [bits & 0xFF] ;
  c0 += c1 ;
  return (uint8_t) (c0 == 8 ? 7 : c0) ;
}


void AudioInputPDMRaw::update(void)
{
	audio_block_t *new_left, *out_left;
	new_left = allocate();
	__disable_irq();
	out_left = block_left;
	block_left = new_left;
	__enable_irq();
	if (out_left) {
		transmit(out_left, 0);
		release(out_left);
	}
}

const uint8_t discrep_table [256] =
  {
    -4, -3, -3, -2, -3, -2, -2, -1, -3, -2, -2, -1, -2, -1, -1,  0,
    -3, -2, -2, -1, -2, -1, -1,  0, -2, -1, -1,  0, -1,  0,  0, +1,
    -3, -2, -2, -1, -2, -1, -1,  0, -2, -1, -1,  0, -1,  0,  0, +1,
    -2, -1, -1,  0, -1,  0,  0, +1, -1,  0,  0, +1,  0, +1, +1, +2,
    -3, -2, -2, -1, -2, -1, -1,  0, -2, -1, -1,  0, -1,  0,  0, +1,
    -2, -1, -1,  0, -1,  0,  0, +1, -1,  0,  0, +1,  0, +1, +1, +2,
    -2, -1, -1,  0, -1,  0,  0, +1, -1,  0,  0, +1,  0, +1, +1, +2,
    -1,  0,  0, +1,  0, +1, +1, +2,  0, +1, +1, +2, +1, +2, +2, +3,
    
    -3, -2, -2, -1, -2, -1, -1,  0, -2, -1, -1,  0, -1,  0,  0, +1,
    -2, -1, -1,  0, -1,  0,  0, +1, -1,  0,  0, +1,  0, +1, +1, +2,
    -2, -1, -1,  0, -1,  0,  0, +1, -1,  0,  0, +1,  0, +1, +1, +2,
    -1,  0,  0, +1,  0, +1, +1, +2,  0, +1, +1, +2, +1, +2, +2, +3,
    -2, -1, -1,  0, -1,  0,  0, +1, -1,  0,  0, +1,  0, +1, +1, +2,
    -1,  0,  0, +1,  0, +1, +1, +2,  0, +1, +1, +2, +1, +2, +2, +3,
    -1,  0,  0, +1,  0, +1, +1, +2,  0, +1, +1, +2, +1, +2, +2, +3,
     0, +1, +1, +2, +1, +2, +2, +3, +1, +2, +2, +3, +2, +3, +3, +4
  };


#endif
