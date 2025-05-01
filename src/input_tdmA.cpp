/* Audio Library for Teensy 3.X
*** 16 bit samples correctly transferred (RP)
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
#include "input_tdmA.h"
#include "output_tdmA.h"
#if defined(KINETISK) || defined(__IMXRT1062__)
#include "utility/imxrt_hw.h"

// DMA buffer for AUDIO_BLOCK_SAMPLES frames à 256 bits, times two for alternate transfer
// this is transferred by DMA in 32 bit chunks
DMAMEM __attribute__((aligned(32)))
static uint8_t tdm_rx_buffer[AUDIO_BLOCK_SAMPLES*(256/sizeof(uint8_t))*2];

audio_block_t * AudioInputTDM_A::block_incoming[16] = {
	nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
	nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr
};
bool AudioInputTDM_A::update_responsibility = false;
DMAChannel AudioInputTDM_A::dma(false);
static int _sampleLengthI;

void AudioInputTDM_A::begin(int sampleLength)
{
	_sampleLengthI = sampleLength;
	dma.begin(true); // Allocate the DMA channel first

	// TODO: should we set & clear the I2S_RCSR_SR bit here?

	// Audio clocks are programmed, according to AUDIO_SAMPLE_RATE_EXACT constant
	// TODO: 96 kHz case might not be correctly covered yet
	AudioOutputTDM_A::config_tdm();

	// DMA always transfers 256 bits per frame (16 x 16 bits = half tdm_rx_buffer)
	// The transfer alternates between the first and the second half of the tdm_rx_buffer.
	// this is as received from the bit block, independent of the actual audio format
	// Must be correctly interpreted in the isr callback

#if defined(KINETISK)
	CORE_PIN13_CONFIG = PORT_PCR_MUX(4); // pin 13, PTC5, I2S0_RXD0
	dma.TCD->SADDR = &I2S0_RDR0;
	dma.TCD->SOFF = 0;
	dma.TCD->ATTR = DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_DSIZE(2);
	dma.TCD->NBYTES_MLNO = 4;
	dma.TCD->SLAST = 0;
	dma.TCD->DADDR = tdm_rx_buffer;
	dma.TCD->DOFF = 4;
	dma.TCD->CITER_ELINKNO = sizeof(tdm_rx_buffer) / 4;
	dma.TCD->DLASTSGA = -sizeof(tdm_rx_buffer);
	dma.TCD->BITER_ELINKNO = sizeof(tdm_rx_buffer) / 4;
	dma.TCD->CSR = DMA_TCD_CSR_INTHALF | DMA_TCD_CSR_INTMAJOR;
	dma.triggerAtHardwareEvent(DMAMUX_SOURCE_I2S0_RX);
	update_responsibility = update_setup();
	dma.enable();

	I2S0_RCSR |= I2S_RCSR_RE | I2S_RCSR_BCE | I2S_RCSR_FRDE | I2S_RCSR_FR;
	I2S0_TCSR |= I2S_TCSR_TE | I2S_TCSR_BCE; // TX clock enable, because sync'd to TX
	dma.attachInterrupt(isr);
#elif defined(__IMXRT1062__)
	CORE_PIN8_CONFIG  = 3;  //RX_DATA0
	IOMUXC_SAI1_RX_DATA0_SELECT_INPUT = 2;
	dma.TCD->SADDR = &I2S1_RDR0;
	dma.TCD->SOFF = 0;
	dma.TCD->ATTR = DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_DSIZE(2);
	dma.TCD->NBYTES_MLNO = 4;
	dma.TCD->SLAST = 0;
	dma.TCD->DADDR = tdm_rx_buffer;
	dma.TCD->DOFF = 4;
	dma.TCD->CITER_ELINKNO = sizeof(tdm_rx_buffer) / 4;
	dma.TCD->DLASTSGA = -sizeof(tdm_rx_buffer);
	dma.TCD->BITER_ELINKNO = sizeof(tdm_rx_buffer) / 4;
	dma.TCD->CSR = DMA_TCD_CSR_INTHALF | DMA_TCD_CSR_INTMAJOR;
	dma.triggerAtHardwareEvent(DMAMUX_SOURCE_SAI1_RX);
	update_responsibility = update_setup();
	dma.enable();

	I2S1_RCSR = I2S_RCSR_RE | I2S_RCSR_BCE | I2S_RCSR_FRDE | I2S_RCSR_FR;
	dma.attachInterrupt(isr);	
#endif

//	printf("DMA size: %d\n", sizeof(tdm_rx_buffer));
}


template<int bits, int channels, int frames=AUDIO_BLOCK_SAMPLES>
static void transfer(const uint8_t *dma_src, audio_block_t **block_dst)
{
	// hop size in bytes from channel to channel
	const int sample_bytes = bits / sizeof(*dma_src);
	// hop size in bytes from frame to frame (frame size is fixed to 256 bits)
	const int frame_bytes = 256 / sizeof(*dma_src);

	if(bits == 16) {
		for(int i = 0; i < channels; ++i, dma_src += sample_bytes) {
			int16_t *dest = block_dst[i]->data;
			const uint8_t *inp = dma_src;
			for(int f = 0; f < frames; ++f, inp += frame_bytes)
				dest[f] = *(const int16_t *)inp;
		}
	}
	else if(bits == 24 || bits == 32) {
		// we pass incoming 24/32-bit data to hi/lo pairs of 16-bit output
		for(int i = 0; i < channels; ++i, dma_src += sample_bytes) {
			int16_t *desthi = block_dst[i*2]->data;
			int16_t *destlo = block_dst[i*2+1]->data;
			const uint8_t *inp = dma_src;
			for(int f = 0; f < frames; ++f, inp += frame_bytes) {
				// MSB is first
				desthi[f] = *(const int16_t *)inp;
				if(bits == 32)
					destlo[f] = *(const int16_t *)(inp+2);
				else
					// 24 bit data is promoted to 32 bits
					destlo[f] = (int16_t)(*(const uint8_t *)(inp+2))<<8;
			}
		}
	}
	else {
		// we don't cover 20 bits
	}
}

static void zeroout(int16_t *dst, int samples)
{
	for(int i = 0; i < samples; ++i) dst[i] = 0;
}

void AudioInputTDM_A::isr(void)
{
	uint32_t daddr = (uint32_t)(dma.TCD->DADDR);

	dma.clearInterrupt();

	const uint8_t *src;
	if (daddr < (uint32_t)tdm_rx_buffer + sizeof(tdm_rx_buffer)/2) {
		// DMA is receiving to the first half of the buffer
		// need to remove data from the second half
		src = tdm_rx_buffer+sizeof(tdm_rx_buffer)/2;
	} else {
		// DMA is receiving to the second half of the buffer
		// need to remove data from the first half
		src = tdm_rx_buffer;
	}

	if (block_incoming[0] != nullptr) {
		#if IMXRT_CACHE_ENABLED >=1
		arm_dcache_delete((void*)src, sizeof(tdm_rx_buffer)/2);
		#endif
	}

	// by DMA, we receive AUDIO_BLOCK_SAMPLES frames à 256 bits
	if(_sampleLengthI == 16) {
		// we receive AUDIO_BLOCK_SAMPLES x 16 channels x 16 bits
//		transfer<16,16>(src, block_incoming);
		for(int i = 0; i < 16; ++i)
			zeroout(block_incoming[i]->data, AUDIO_BLOCK_SAMPLES);
//			memset(block_incoming[i]->data, 0, sizeof(block_incoming[i]->data));
	}
	else if(_sampleLengthI == 32) {
		transfer<32,8>(src, block_incoming);
	}
	else if(_sampleLengthI == 24) {
		// We receive 10 channels per DMA, but deliver only 8 x 16-bit output pairs
		transfer<24,8>(src, block_incoming);
	}
	else { // 20 bit case not covered
		for(int i = 0; i < 16; ++i)
			zeroout(block_incoming[i]->data, AUDIO_BLOCK_SAMPLES);
	}

	if (update_responsibility) update_all();
}


void AudioInputTDM_A::update(void)
{
	unsigned int i, j;
	audio_block_t *new_block[16];
	audio_block_t *out_block[16];

	// allocate 16 new blocks.  If any fails, allocate none
	for (i=0; i < 16; i++) {
		new_block[i] = allocate();
		if (new_block[i] == nullptr) {
			for (j=0; j < i; j++) {
				release(new_block[j]);
			}
			memset(new_block, 0, sizeof(new_block));
			break;
		}
	}
	__disable_irq();
	memcpy(out_block, block_incoming, sizeof(out_block));
	memcpy(block_incoming, new_block, sizeof(block_incoming));
	__enable_irq();
	if (out_block[0] != nullptr) {		
		// if we got 1 block, all 16 are filled
		for (i=0; i < 16; i++) {
			transmit(out_block[i], i);
			release(out_block[i]);
		}
	}
}


#endif
