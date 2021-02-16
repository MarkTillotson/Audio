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
audio_block_t * AudioInputPDMRaw::block0 = NULL;
audio_block_t * AudioInputPDMRaw::block1 = NULL;
audio_block_t * AudioInputPDMRaw::block2 = NULL;
audio_block_t * AudioInputPDMRaw::block3 = NULL;

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
  audio_block_t * blk0, * blk1, * blk2, * blk3 ;
  
#if defined(KINETISK)
  daddr = (uint32_t)(dma.TCD->DADDR);
#endif
  dma.clearInterrupt();

  if (daddr < (uint32_t)pdm_buffer + sizeof(pdm_buffer) / 2)
    // DMA is receiving to the first half of the buffer; need to remove data from the second half
    src = pdm_buffer + AUDIO_BLOCK_SAMPLES*2;
  else
    // DMA is receiving to the second half of the buffer; need to remove data from the first half
    src = pdm_buffer;

  
  if (update_responsibility)
    AudioStream::update_all();
  
  blk0 = block0 ;
  blk1 = block1 ;
  blk2 = block2 ;
  blk3 = block3 ;

  if (blk0 != NULL && blk1 != NULL && blk2 != NULL && blk3 != NULL)
  {
    int16_t * dst0 = blk0->data;
    int16_t * dst1 = blk1->data;
    int16_t * dst2 = blk2->data;
    int16_t * dst3 = blk3->data;
    for (unsigned int i = 0 ; i < AUDIO_BLOCK_SAMPLES*2 ; i += 2)
    {
      uint32_t word0 = src[i] ;
      uint32_t word1 = src[i+1] ;
      *dst0++ = word0 >> 16 ;
      *dst1++ = word0 & 0xFFFF ;
      *dst2++ = word1 >> 16 ;
      *dst3++ = word1 & 0xFFFF ;
    }
  }
}

void AudioInputPDMRaw::update(void)
{
  // allocate new set of buffers
  audio_block_t * new_blk0 = allocate();
  audio_block_t * new_blk1 = allocate();
  audio_block_t * new_blk2 = allocate();
  audio_block_t * new_blk3 = allocate();

  // allocate issue?  abandon
  if (new_blk0 == NULL || new_blk1 == NULL || new_blk2 == NULL || new_blk3 == NULL)
  {
    if (new_blk0) release (new_blk0) ;
    if (new_blk1) release (new_blk1) ;
    if (new_blk2) release (new_blk2) ;
    if (new_blk3) release (new_blk3) ;
    return ;
  }

  // swap buffers
  __disable_irq();
  audio_block_t * out_blk0 = block0 ;  block0 = new_blk0 ;
  audio_block_t * out_blk1 = block1 ;  block1 = new_blk1 ;
  audio_block_t * out_blk2 = block2 ;  block2 = new_blk2 ;
  audio_block_t * out_blk3 = block3 ;  block3 = new_blk3 ;
  __enable_irq();

  // transmit buffers and release
  if (out_blk0)  { transmit(out_blk0, 0); release(out_blk0); }
  if (out_blk1)  { transmit(out_blk1, 1); release(out_blk1); }
  if (out_blk2)  { transmit(out_blk2, 2); release(out_blk2); }
  if (out_blk3)  { transmit(out_blk3, 3); release(out_blk3); }

}





extern const int16_t lo_table_acc1 [256];
extern const int16_t lo_table_acc2 [256];
extern const int16_t lo_table_acc3 [256];
extern const int16_t lo_table_acc4 [256];
extern const int16_t lo_table_acc5 [256];

extern const int16_t hi_table_acc1 [256];
extern const int16_t hi_table_acc2 [256];
extern const int16_t hi_table_acc3 [256];
extern const int16_t hi_table_acc4 [256];
extern const int16_t hi_table_acc5 [256];


inline int16_t AudioConvertFromPDM::cicfilt (uint16_t b)
{
  uint16_t hi = b >> 8 ;
  uint16_t lo = b & 0xFF ;

  // aggregated effect of 5 integrators clocked 16 times
  a2 += (a1 << 4) ;
  a3 += (a2 << 4) - 120 * a1 ;
  a4 += (a3 << 4) - 120 * a2 + 560 * a1 ;
  a5 += (a4 << 4) - 120 * a3 + 560 * a2 - 1820 * a1 ;

  // linearly combined with the effect of the incoming bits, pretabulated.
  a1 += lo_table_acc1 [lo] + hi_table_acc1 [hi] ;
  a2 += lo_table_acc2 [lo] + hi_table_acc2 [hi] ;
  a3 += lo_table_acc3 [lo] + hi_table_acc3 [hi] ;
  a4 += lo_table_acc4 [lo] + hi_table_acc4 [hi] ;
  a5 += lo_table_acc5 [lo] + hi_table_acc5 [hi] ;

  /*  The slower way running all the integrators 16 times explicitly.
  for (int shft = 14 ; shft >= 0 ; shft --)
  {
    a1 += ((b >> shft) & 2) - 1 ;
    a2 += a1 ;
    a3 += a2 ;
    a4 += a3 ;
    a5 += a4 ;
  }
  
  a1 += ((b<<1) & 2) - 1 ;
  a2 += a1 ;
  a3 += a2 ;
  a4 += a3 ;
  a5 += a4 ;
  */
  
  // now implicitly downsample by 16

  // and differentiate
  uint32_t t1 = a5 - d1 ;
  d1 = a5 ;
  uint32_t t2 = t1 - d2 ;
  d2 = t1 ;
  uint32_t t3 = t2 - d3 ;
  d3 = t2 ;
  uint32_t t4 = t3 - d4 ;
  d4 = t3 ;
  uint32_t t5 = t4 - d5 ;
  d5 = t4 ;

  return (int16_t) (((int32_t) t5) >> 5) ;
}

#define FO 8
#define FF 4


// Class to pull in 4 samples and decimate by a further factor of 4
void AudioConvertFromPDM::update (void)
{

  digitalWriteFast(3, HIGH);
  audio_block_t * in0 = receiveReadOnly (0) ;
  audio_block_t * in1 = receiveReadOnly (1) ;
  audio_block_t * in2 = receiveReadOnly (2) ;
  audio_block_t * in3 = receiveReadOnly (3) ;
  if (in0 == NULL || in1 == NULL || in2 == NULL || in3 == NULL)
  {
    if (in0) release (in0) ;
    if (in1) release (in1) ;
    if (in2) release (in2) ;
    if (in3) release (in3) ;
    return ;
  }

  audio_block_t * out = allocate () ;
  if (out == NULL)
    return ;
  //if (in1) digitalWriteFast(3, LOW);


  int16_t * src0 = in0->data ;
  int16_t * src1 = in1->data ;
  int16_t * src2 = in2->data ;
  int16_t * src3 = in3->data ;
  int16_t * dst = out->data ;

  // simple 1st order low pass
  for (int i = 0 ; i < AUDIO_BLOCK_SAMPLES ; i++)
  {
    /*
    sum += cicfilt (*src0++) ;
    sum -= (sum+FO) >> FF ;
    sum += cicfilt (*src1++) ;
    sum -= (sum+FO) >> FF ;
    sum += cicfilt (*src2++) ;
    sum -= (sum+FO) >> FF ;
    sum += cicfilt (*src3++) ;
    sum -= (sum+FO) >> FF ;
    int16_t sample = (int16_t) (sum >> FF) ;
    */

    // [2, 0, -10, 0, 41, 0, -122, 0, 306, 0, -678, 0, 1404, 0, -3019, 0, 10269, 16384, 10269, 0, -3019, 0, 1404, 0, -678, 0, 306, 0, -122, 0, 41, 0, -10, 0, 2]
    //
    
    // IIR compensation filter,  1 / (z^2 - 1.5z + 0.75)  (poles   0.75 +/-  0.433j)
    
    int32_t t ;
    t = cicfilt (*src0++) << 8 ;
    t += 3 * (del1+del1-del2) >> 2 ;
    del2 = del1 ;    del1 = t ;
    
    t = cicfilt (*src1++) << 8 ;
    t += 3 * (del1+del1-del2) >> 2 ;
    del2 = del1 ;    del1 = t ;
    
    t = cicfilt (*src2++) << 8 ;
    t += 3 * (del1+del1-del2) >> 2 ;
    del2 = del1 ;    del1 = t ;
    
    t = cicfilt (*src3++) << 8 ;
    t += 3 * (del1+del1-del2) >> 2 ;
    del2 = del1 ;    del1 = t ;


    
    int16_t sample = (int16_t) (t>>10) ;
      
    *dst++ = sample ;
  }
  release (in0) ;
  release (in1) ;
  release (in2) ;
  release (in3) ;
  transmit (out) ;
  release (out) ;

  digitalWriteFast(3, LOW);
}



void AudioConvertToPDM::update (void)
{
  digitalWriteFast(3, HIGH);

  audio_block_t * in = receiveReadOnly (0) ;
  if (in == NULL)
    return ;
  audio_block_t * out0 = allocate () ;
  audio_block_t * out1 = allocate () ;
  audio_block_t * out2 = allocate () ;
  audio_block_t * out3 = allocate () ;
  if (out0 == NULL || out1 == NULL || out2 == NULL || out3 == NULL)
  {
    release (in) ;
    if (out0) release (out0) ;
    if (out1) release (out1) ;
    if (out2) release (out2) ;
    if (out3) release (out3) ;
    return ;
  }

  int16_t * src = in->data ;
  int16_t * dests[4] ;
  dests[0] = out0->data ;
  dests[1] = out1->data ;
  dests[2] = out2->data ;
  dests[3] = out3->data ;

  for (int i = 0 ; i < AUDIO_BLOCK_SAMPLES ; i++)
  {
    int32_t curr = ((int32_t) src[i]) << (16 - 6) ; 
    for (int j = 0 ; j < 4 ; j++)
    {
      uint16_t bits = 0 ;
      for (int k = 0 ; k < 16 ; k++)
      {
	accum += curr;
	if (accum >= 0)
	{
	  accum -= (0x8000 << (16-6)) ;
	  bits = (bits << 1) | 1 ;
	}
	else
	{
	  accum += (0x8000 << (16-6)) ;
	  bits = bits << 1 ;
	}
      }
      *(dests[j])++ = (int16_t) bits ;
    }
  }

  release (in) ;
  transmit (out0, 0) ;  release (out0) ;
  transmit (out1, 1) ;  release (out1) ;
  transmit (out2, 2) ;  release (out2) ;
  transmit (out3, 3) ;  release (out3) ;

  digitalWriteFast(3, LOW);
}


const int16_t lo_table_acc1 [256] =
{
    -8,   -6,   -6,   -4,   -6,   -4,   -4,   -2,   -6,   -4,   -4,   -2,   -4,   -2,   -2,    0, 
    -6,   -4,   -4,   -2,   -4,   -2,   -2,    0,   -4,   -2,   -2,    0,   -2,    0,    0,    2, 
    -6,   -4,   -4,   -2,   -4,   -2,   -2,    0,   -4,   -2,   -2,    0,   -2,    0,    0,    2, 
    -4,   -2,   -2,    0,   -2,    0,    0,    2,   -2,    0,    0,    2,    0,    2,    2,    4, 
    -6,   -4,   -4,   -2,   -4,   -2,   -2,    0,   -4,   -2,   -2,    0,   -2,    0,    0,    2, 
    -4,   -2,   -2,    0,   -2,    0,    0,    2,   -2,    0,    0,    2,    0,    2,    2,    4, 
    -4,   -2,   -2,    0,   -2,    0,    0,    2,   -2,    0,    0,    2,    0,    2,    2,    4, 
    -2,    0,    0,    2,    0,    2,    2,    4,    0,    2,    2,    4,    2,    4,    4,    6, 
    -6,   -4,   -4,   -2,   -4,   -2,   -2,    0,   -4,   -2,   -2,    0,   -2,    0,    0,    2, 
    -4,   -2,   -2,    0,   -2,    0,    0,    2,   -2,    0,    0,    2,    0,    2,    2,    4, 
    -4,   -2,   -2,    0,   -2,    0,    0,    2,   -2,    0,    0,    2,    0,    2,    2,    4, 
    -2,    0,    0,    2,    0,    2,    2,    4,    0,    2,    2,    4,    2,    4,    4,    6, 
    -4,   -2,   -2,    0,   -2,    0,    0,    2,   -2,    0,    0,    2,    0,    2,    2,    4, 
    -2,    0,    0,    2,    0,    2,    2,    4,    0,    2,    2,    4,    2,    4,    4,    6, 
    -2,    0,    0,    2,    0,    2,    2,    4,    0,    2,    2,    4,    2,    4,    4,    6, 
     0,    2,    2,    4,    2,    4,    4,    6,    2,    4,    4,    6,    4,    6,    6,    8, 
};

const int16_t lo_table_acc2 [256] =
{
   -36,  -34,  -32,  -30,  -30,  -28,  -26,  -24,  -28,  -26,  -24,  -22,  -22,  -20,  -18,  -16, 
   -26,  -24,  -22,  -20,  -20,  -18,  -16,  -14,  -18,  -16,  -14,  -12,  -12,  -10,   -8,   -6, 
   -24,  -22,  -20,  -18,  -18,  -16,  -14,  -12,  -16,  -14,  -12,  -10,  -10,   -8,   -6,   -4, 
   -14,  -12,  -10,   -8,   -8,   -6,   -4,   -2,   -6,   -4,   -2,    0,    0,    2,    4,    6, 
   -22,  -20,  -18,  -16,  -16,  -14,  -12,  -10,  -14,  -12,  -10,   -8,   -8,   -6,   -4,   -2, 
   -12,  -10,   -8,   -6,   -6,   -4,   -2,    0,   -4,   -2,    0,    2,    2,    4,    6,    8, 
   -10,   -8,   -6,   -4,   -4,   -2,    0,    2,   -2,    0,    2,    4,    4,    6,    8,   10, 
     0,    2,    4,    6,    6,    8,   10,   12,    8,   10,   12,   14,   14,   16,   18,   20, 
   -20,  -18,  -16,  -14,  -14,  -12,  -10,   -8,  -12,  -10,   -8,   -6,   -6,   -4,   -2,    0, 
   -10,   -8,   -6,   -4,   -4,   -2,    0,    2,   -2,    0,    2,    4,    4,    6,    8,   10, 
    -8,   -6,   -4,   -2,   -2,    0,    2,    4,    0,    2,    4,    6,    6,    8,   10,   12, 
     2,    4,    6,    8,    8,   10,   12,   14,   10,   12,   14,   16,   16,   18,   20,   22, 
    -6,   -4,   -2,    0,    0,    2,    4,    6,    2,    4,    6,    8,    8,   10,   12,   14, 
     4,    6,    8,   10,   10,   12,   14,   16,   12,   14,   16,   18,   18,   20,   22,   24, 
     6,    8,   10,   12,   12,   14,   16,   18,   14,   16,   18,   20,   20,   22,   24,   26, 
    16,   18,   20,   22,   22,   24,   26,   28,   24,   26,   28,   30,   30,   32,   34,   36, 
};

const int16_t lo_table_acc3 [256] =
{
  -120, -118, -114, -112, -108, -106, -102, -100, -100,  -98,  -94,  -92,  -88,  -86,  -82,  -80, 
   -90,  -88,  -84,  -82,  -78,  -76,  -72,  -70,  -70,  -68,  -64,  -62,  -58,  -56,  -52,  -50, 
   -78,  -76,  -72,  -70,  -66,  -64,  -60,  -58,  -58,  -56,  -52,  -50,  -46,  -44,  -40,  -38, 
   -48,  -46,  -42,  -40,  -36,  -34,  -30,  -28,  -28,  -26,  -22,  -20,  -16,  -14,  -10,   -8, 
   -64,  -62,  -58,  -56,  -52,  -50,  -46,  -44,  -44,  -42,  -38,  -36,  -32,  -30,  -26,  -24, 
   -34,  -32,  -28,  -26,  -22,  -20,  -16,  -14,  -14,  -12,   -8,   -6,   -2,    0,    4,    6, 
   -22,  -20,  -16,  -14,  -10,   -8,   -4,   -2,   -2,    0,    4,    6,   10,   12,   16,   18, 
     8,   10,   14,   16,   20,   22,   26,   28,   28,   30,   34,   36,   40,   42,   46,   48, 
   -48,  -46,  -42,  -40,  -36,  -34,  -30,  -28,  -28,  -26,  -22,  -20,  -16,  -14,  -10,   -8, 
   -18,  -16,  -12,  -10,   -6,   -4,    0,    2,    2,    4,    8,   10,   14,   16,   20,   22, 
    -6,   -4,    0,    2,    6,    8,   12,   14,   14,   16,   20,   22,   26,   28,   32,   34, 
    24,   26,   30,   32,   36,   38,   42,   44,   44,   46,   50,   52,   56,   58,   62,   64, 
     8,   10,   14,   16,   20,   22,   26,   28,   28,   30,   34,   36,   40,   42,   46,   48, 
    38,   40,   44,   46,   50,   52,   56,   58,   58,   60,   64,   66,   70,   72,   76,   78, 
    50,   52,   56,   58,   62,   64,   68,   70,   70,   72,   76,   78,   82,   84,   88,   90, 
    80,   82,   86,   88,   92,   94,   98,  100,  100,  102,  106,  108,  112,  114,  118,  120, 
};

const int16_t lo_table_acc4 [256] =
{
  -330, -328, -322, -320, -310, -308, -302, -300, -290, -288, -282, -280, -270, -268, -262, -260, 
  -260, -258, -252, -250, -240, -238, -232, -230, -220, -218, -212, -210, -200, -198, -192, -190, 
  -218, -216, -210, -208, -198, -196, -190, -188, -178, -176, -170, -168, -158, -156, -150, -148, 
  -148, -146, -140, -138, -128, -126, -120, -118, -108, -106, -100,  -98,  -88,  -86,  -80,  -78, 
  -162, -160, -154, -152, -142, -140, -134, -132, -122, -120, -114, -112, -102, -100,  -94,  -92, 
   -92,  -90,  -84,  -82,  -72,  -70,  -64,  -62,  -52,  -50,  -44,  -42,  -32,  -30,  -24,  -22, 
   -50,  -48,  -42,  -40,  -30,  -28,  -22,  -20,  -10,   -8,   -2,    0,   10,   12,   18,   20, 
    20,   22,   28,   30,   40,   42,   48,   50,   60,   62,   68,   70,   80,   82,   88,   90, 
   -90,  -88,  -82,  -80,  -70,  -68,  -62,  -60,  -50,  -48,  -42,  -40,  -30,  -28,  -22,  -20, 
   -20,  -18,  -12,  -10,    0,    2,    8,   10,   20,   22,   28,   30,   40,   42,   48,   50, 
    22,   24,   30,   32,   42,   44,   50,   52,   62,   64,   70,   72,   82,   84,   90,   92, 
    92,   94,  100,  102,  112,  114,  120,  122,  132,  134,  140,  142,  152,  154,  160,  162, 
    78,   80,   86,   88,   98,  100,  106,  108,  118,  120,  126,  128,  138,  140,  146,  148, 
   148,  150,  156,  158,  168,  170,  176,  178,  188,  190,  196,  198,  208,  210,  216,  218, 
   190,  192,  198,  200,  210,  212,  218,  220,  230,  232,  238,  240,  250,  252,  258,  260, 
   260,  262,  268,  270,  280,  282,  288,  290,  300,  302,  308,  310,  320,  322,  328,  330, 
};

const int16_t lo_table_acc5 [256] =
{
  -792, -790, -782, -780, -762, -760, -752, -750, -722, -720, -712, -710, -692, -690, -682, -680, 
  -652, -650, -642, -640, -622, -620, -612, -610, -582, -580, -572, -570, -552, -550, -542, -540, 
  -540, -538, -530, -528, -510, -508, -500, -498, -470, -468, -460, -458, -440, -438, -430, -428, 
  -400, -398, -390, -388, -370, -368, -360, -358, -330, -328, -320, -318, -300, -298, -290, -288, 
  -372, -370, -362, -360, -342, -340, -332, -330, -302, -300, -292, -290, -272, -270, -262, -260, 
  -232, -230, -222, -220, -202, -200, -192, -190, -162, -160, -152, -150, -132, -130, -122, -120, 
  -120, -118, -110, -108,  -90,  -88,  -80,  -78,  -50,  -48,  -40,  -38,  -20,  -18,  -10,   -8, 
    20,   22,   30,   32,   50,   52,   60,   62,   90,   92,  100,  102,  120,  122,  130,  132, 
  -132, -130, -122, -120, -102, -100,  -92,  -90,  -62,  -60,  -52,  -50,  -32,  -30,  -22,  -20, 
     8,   10,   18,   20,   38,   40,   48,   50,   78,   80,   88,   90,  108,  110,  118,  120, 
   120,  122,  130,  132,  150,  152,  160,  162,  190,  192,  200,  202,  220,  222,  230,  232, 
   260,  262,  270,  272,  290,  292,  300,  302,  330,  332,  340,  342,  360,  362,  370,  372, 
   288,  290,  298,  300,  318,  320,  328,  330,  358,  360,  368,  370,  388,  390,  398,  400, 
   428,  430,  438,  440,  458,  460,  468,  470,  498,  500,  508,  510,  528,  530,  538,  540, 
   540,  542,  550,  552,  570,  572,  580,  582,  610,  612,  620,  622,  640,  642,  650,  652, 
   680,  682,  690,  692,  710,  712,  720,  722,  750,  752,  760,  762,  780,  782,  790,  792, 
};

const int16_t hi_table_acc1 [256] =
{
    -8,   -6,   -6,   -4,   -6,   -4,   -4,   -2,   -6,   -4,   -4,   -2,   -4,   -2,   -2,    0, 
    -6,   -4,   -4,   -2,   -4,   -2,   -2,    0,   -4,   -2,   -2,    0,   -2,    0,    0,    2, 
    -6,   -4,   -4,   -2,   -4,   -2,   -2,    0,   -4,   -2,   -2,    0,   -2,    0,    0,    2, 
    -4,   -2,   -2,    0,   -2,    0,    0,    2,   -2,    0,    0,    2,    0,    2,    2,    4, 
    -6,   -4,   -4,   -2,   -4,   -2,   -2,    0,   -4,   -2,   -2,    0,   -2,    0,    0,    2, 
    -4,   -2,   -2,    0,   -2,    0,    0,    2,   -2,    0,    0,    2,    0,    2,    2,    4, 
    -4,   -2,   -2,    0,   -2,    0,    0,    2,   -2,    0,    0,    2,    0,    2,    2,    4, 
    -2,    0,    0,    2,    0,    2,    2,    4,    0,    2,    2,    4,    2,    4,    4,    6, 
    -6,   -4,   -4,   -2,   -4,   -2,   -2,    0,   -4,   -2,   -2,    0,   -2,    0,    0,    2, 
    -4,   -2,   -2,    0,   -2,    0,    0,    2,   -2,    0,    0,    2,    0,    2,    2,    4, 
    -4,   -2,   -2,    0,   -2,    0,    0,    2,   -2,    0,    0,    2,    0,    2,    2,    4, 
    -2,    0,    0,    2,    0,    2,    2,    4,    0,    2,    2,    4,    2,    4,    4,    6, 
    -4,   -2,   -2,    0,   -2,    0,    0,    2,   -2,    0,    0,    2,    0,    2,    2,    4, 
    -2,    0,    0,    2,    0,    2,    2,    4,    0,    2,    2,    4,    2,    4,    4,    6, 
    -2,    0,    0,    2,    0,    2,    2,    4,    0,    2,    2,    4,    2,    4,    4,    6, 
     0,    2,    2,    4,    2,    4,    4,    6,    2,    4,    4,    6,    4,    6,    6,    8, 
};

const int16_t hi_table_acc2 [256] =
{
  -100,  -82,  -80,  -62,  -78,  -60,  -58,  -40,  -76,  -58,  -56,  -38,  -54,  -36,  -34,  -16, 
   -74,  -56,  -54,  -36,  -52,  -34,  -32,  -14,  -50,  -32,  -30,  -12,  -28,  -10,   -8,   10, 
   -72,  -54,  -52,  -34,  -50,  -32,  -30,  -12,  -48,  -30,  -28,  -10,  -26,   -8,   -6,   12, 
   -46,  -28,  -26,   -8,  -24,   -6,   -4,   14,  -22,   -4,   -2,   16,    0,   18,   20,   38, 
   -70,  -52,  -50,  -32,  -48,  -30,  -28,  -10,  -46,  -28,  -26,   -8,  -24,   -6,   -4,   14, 
   -44,  -26,  -24,   -6,  -22,   -4,   -2,   16,  -20,   -2,    0,   18,    2,   20,   22,   40, 
   -42,  -24,  -22,   -4,  -20,   -2,    0,   18,  -18,    0,    2,   20,    4,   22,   24,   42, 
   -16,    2,    4,   22,    6,   24,   26,   44,    8,   26,   28,   46,   30,   48,   50,   68, 
   -68,  -50,  -48,  -30,  -46,  -28,  -26,   -8,  -44,  -26,  -24,   -6,  -22,   -4,   -2,   16, 
   -42,  -24,  -22,   -4,  -20,   -2,    0,   18,  -18,    0,    2,   20,    4,   22,   24,   42, 
   -40,  -22,  -20,   -2,  -18,    0,    2,   20,  -16,    2,    4,   22,    6,   24,   26,   44, 
   -14,    4,    6,   24,    8,   26,   28,   46,   10,   28,   30,   48,   32,   50,   52,   70, 
   -38,  -20,  -18,    0,  -16,    2,    4,   22,  -14,    4,    6,   24,    8,   26,   28,   46, 
   -12,    6,    8,   26,   10,   28,   30,   48,   12,   30,   32,   50,   34,   52,   54,   72, 
   -10,    8,   10,   28,   12,   30,   32,   50,   14,   32,   34,   52,   36,   54,   56,   74, 
    16,   34,   36,   54,   38,   56,   58,   76,   40,   58,   60,   78,   62,   80,   82,  100, 
};

const int16_t hi_table_acc3 [256] =
{
  -696, -606, -586, -496, -564, -474, -454, -364, -540, -450, -430, -340, -408, -318, -298, -208, 
  -514, -424, -404, -314, -382, -292, -272, -182, -358, -268, -248, -158, -226, -136, -116,  -26, 
  -486, -396, -376, -286, -354, -264, -244, -154, -330, -240, -220, -130, -198, -108,  -88,    2, 
  -304, -214, -194, -104, -172,  -82,  -62,   28, -148,  -58,  -38,   52,  -16,   74,   94,  184, 
  -456, -366, -346, -256, -324, -234, -214, -124, -300, -210, -190, -100, -168,  -78,  -58,   32, 
  -274, -184, -164,  -74, -142,  -52,  -32,   58, -118,  -28,   -8,   82,   14,  104,  124,  214, 
  -246, -156, -136,  -46, -114,  -24,   -4,   86,  -90,    0,   20,  110,   42,  132,  152,  242, 
   -64,   26,   46,  136,   68,  158,  178,  268,   92,  182,  202,  292,  224,  314,  334,  424, 
  -424, -334, -314, -224, -292, -202, -182,  -92, -268, -178, -158,  -68, -136,  -46,  -26,   64, 
  -242, -152, -132,  -42, -110,  -20,    0,   90,  -86,    4,   24,  114,   46,  136,  156,  246, 
  -214, -124, -104,  -14,  -82,    8,   28,  118,  -58,   32,   52,  142,   74,  164,  184,  274, 
   -32,   58,   78,  168,  100,  190,  210,  300,  124,  214,  234,  324,  256,  346,  366,  456, 
  -184,  -94,  -74,   16,  -52,   38,   58,  148,  -28,   62,   82,  172,  104,  194,  214,  304, 
    -2,   88,  108,  198,  130,  220,  240,  330,  154,  244,  264,  354,  286,  376,  396,  486, 
    26,  116,  136,  226,  158,  248,  268,  358,  182,  272,  292,  382,  314,  404,  424,  514, 
   208,  298,  318,  408,  340,  430,  450,  540,  364,  454,  474,  564,  496,  586,  606,  696, 
};

const int16_t hi_table_acc4 [256] =
{
  -3546, -3216, -3106, -2776, -2974, -2644, -2534, -2204, -2818, -2488, -2378, -2048, -2246, -1916, -1806, -1476, 
  -2636, -2306, -2196, -1866, -2064, -1734, -1624, -1294, -1908, -1578, -1468, -1138, -1336, -1006, -896, -566, 
  -2426, -2096, -1986, -1656, -1854, -1524, -1414, -1084, -1698, -1368, -1258, -928, -1126, -796, -686, -356, 
  -1516, -1186, -1076, -746, -944, -614, -504, -174, -788, -458, -348,  -18, -216,  114,  224,  554, 
  -2186, -1856, -1746, -1416, -1614, -1284, -1174, -844, -1458, -1128, -1018, -688, -886, -556, -446, -116, 
  -1276, -946, -836, -506, -704, -374, -264,   66, -548, -218, -108,  222,   24,  354,  464,  794, 
  -1066, -736, -626, -296, -494, -164,  -54,  276, -338,   -8,  102,  432,  234,  564,  674, 1004, 
  -156,  174,  284,  614,  416,  746,  856, 1186,  572,  902, 1012, 1342, 1144, 1474, 1584, 1914, 
  -1914, -1584, -1474, -1144, -1342, -1012, -902, -572, -1186, -856, -746, -416, -614, -284, -174,  156, 
  -1004, -674, -564, -234, -432, -102,    8,  338, -276,   54,  164,  494,  296,  626,  736, 1066, 
  -794, -464, -354,  -24, -222,  108,  218,  548,  -66,  264,  374,  704,  506,  836,  946, 1276, 
   116,  446,  556,  886,  688, 1018, 1128, 1458,  844, 1174, 1284, 1614, 1416, 1746, 1856, 2186, 
  -554, -224, -114,  216,   18,  348,  458,  788,  174,  504,  614,  944,  746, 1076, 1186, 1516, 
   356,  686,  796, 1126,  928, 1258, 1368, 1698, 1084, 1414, 1524, 1854, 1656, 1986, 2096, 2426, 
   566,  896, 1006, 1336, 1138, 1468, 1578, 1908, 1294, 1624, 1734, 2064, 1866, 2196, 2306, 2636, 
  1476, 1806, 1916, 2246, 2048, 2378, 2488, 2818, 2204, 2534, 2644, 2974, 2776, 3106, 3216, 3546, 
};

const int16_t hi_table_acc5 [256] =
{
  -14712, -13722, -13282, -12292, -12710, -11720, -11280, -10290, -11982, -10992, -10552, -9562, -9980, -8990, -8550, -7560, 
  -11072, -10082, -9642, -8652, -9070, -8080, -7640, -6650, -8342, -7352, -6912, -5922, -6340, -5350, -4910, -3920, 
  -9952, -8962, -8522, -7532, -7950, -6960, -6520, -5530, -7222, -6232, -5792, -4802, -5220, -4230, -3790, -2800, 
  -6312, -5322, -4882, -3892, -4310, -3320, -2880, -1890, -3582, -2592, -2152, -1162, -1580, -590, -150,  840, 
  -8592, -7602, -7162, -6172, -6590, -5600, -5160, -4170, -5862, -4872, -4432, -3442, -3860, -2870, -2430, -1440, 
  -4952, -3962, -3522, -2532, -2950, -1960, -1520, -530, -2222, -1232, -792,  198, -220,  770, 1210, 2200, 
  -3832, -2842, -2402, -1412, -1830, -840, -400,  590, -1102, -112,  328, 1318,  900, 1890, 2330, 3320, 
  -192,  798, 1238, 2228, 1810, 2800, 3240, 4230, 2538, 3528, 3968, 4958, 4540, 5530, 5970, 6960, 
  -6960, -5970, -5530, -4540, -4958, -3968, -3528, -2538, -4230, -3240, -2800, -1810, -2228, -1238, -798,  192, 
  -3320, -2330, -1890, -900, -1318, -328,  112, 1102, -590,  400,  840, 1830, 1412, 2402, 2842, 3832, 
  -2200, -1210, -770,  220, -198,  792, 1232, 2222,  530, 1520, 1960, 2950, 2532, 3522, 3962, 4952, 
  1440, 2430, 2870, 3860, 3442, 4432, 4872, 5862, 4170, 5160, 5600, 6590, 6172, 7162, 7602, 8592, 
  -840,  150,  590, 1580, 1162, 2152, 2592, 3582, 1890, 2880, 3320, 4310, 3892, 4882, 5322, 6312, 
  2800, 3790, 4230, 5220, 4802, 5792, 6232, 7222, 5530, 6520, 6960, 7950, 7532, 8522, 8962, 9952, 
  3920, 4910, 5350, 6340, 5922, 6912, 7352, 8342, 6650, 7640, 8080, 9070, 8652, 9642, 10082, 11072, 
  7560, 8550, 8990, 9980, 9562, 10552, 10992, 11982, 10290, 11280, 11720, 12710, 12292, 13282, 13722, 14712, 
};


#endif
