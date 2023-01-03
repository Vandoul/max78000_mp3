/*
 * codec.c
 *
 *  Created on: 2022Äê12ÔÂ26ÈÕ
 *      Author: wakoj
 */

#include <stdio.h>
#include <stdint.h>
#include "mxc.h"
#include "max9867.h"

#undef USE_I2S_INTERRUPTS

#define CODEC_I2C MXC_I2C0
#define CODEC_I2C_FREQ 100000

#define CODEC_MCLOCK 12288000
#define BITS_PER_CHANNEL 16
#define CHANNELS_PER_FRAME 2
#define SAMPLE_RATE 24000

#define BIT_CLK (SAMPLE_RATE * CHANNELS_PER_FRAME * BITS_PER_CHANNEL)
#define CLK_DIV (((CODEC_MCLOCK / 2) / BIT_CLK) - 1)

#define I2S_DMA_BUFFER_SIZE 2048

volatile int dma_flag;
uint32_t i2s_rx_buffer[2][I2S_DMA_BUFFER_SIZE];
int dma_ch_tx, dma_ch_rx;
uint32_t *rxBufPtr = i2s_rx_buffer;

void blink_halt(const char *msg)
{
    if (msg && *msg)
        puts(msg);

    for (;;) {
        LED_On(LED1);
        LED_Off(LED2);
        MXC_Delay(MXC_DELAY_MSEC(500));
        LED_On(LED2);
        LED_Off(LED1);
        MXC_Delay(MXC_DELAY_MSEC(500));
    }
}

void dma_handler(void)
{
    dma_flag = 1;
    MXC_DMA_Handler();
}

void dma_init(void)
{
    MXC_NVIC_SetVector(DMA0_IRQn, dma_handler);
    MXC_NVIC_SetVector(DMA1_IRQn, dma_handler);
    NVIC_EnableIRQ(DMA0_IRQn);
    NVIC_EnableIRQ(DMA1_IRQn);
}

void dma_callback(int channel, int result)
{
    static uint32_t *tx_buf = i2s_rx_buffer + I2S_DMA_BUFFER_SIZE;

    if (channel == dma_ch_tx) {
        MXC_DMA_ReleaseChannel(dma_ch_tx);
        dma_ch_tx = MXC_I2S_TXDMAConfig(tx_buf, I2S_DMA_BUFFER_SIZE * sizeof(i2s_rx_buffer[0]));

    } else if (channel == dma_ch_rx) {
        tx_buf = rxBufPtr;

        if (rxBufPtr == i2s_rx_buffer) {
            rxBufPtr = i2s_rx_buffer + I2S_DMA_BUFFER_SIZE;
        } else {
            rxBufPtr = i2s_rx_buffer;
        }
        MXC_DMA_ReleaseChannel(dma_ch_rx);
        dma_ch_rx = MXC_I2S_RXDMAConfig(rxBufPtr, I2S_DMA_BUFFER_SIZE * sizeof(i2s_rx_buffer[0]));
    }
}

void dma_work_loop(void)
{
    int trig = 0;

    dma_init();
    MXC_I2S_RegisterDMACallback(dma_callback);
    dma_ch_tx = MXC_I2S_TXDMAConfig(i2s_rx_buffer + I2S_DMA_BUFFER_SIZE,
                                    I2S_DMA_BUFFER_SIZE * sizeof(i2s_rx_buffer[0]));
    dma_ch_rx = MXC_I2S_RXDMAConfig(i2s_rx_buffer, I2S_DMA_BUFFER_SIZE * sizeof(i2s_rx_buffer[0]));

    for (;;) {
        if (dma_flag) {
            dma_flag = 0;
            /*
        dma activity triggered work
      */
            if (++trig == SAMPLE_RATE / I2S_DMA_BUFFER_SIZE) {
                trig = 0;
                LED_Toggle(LED2);
            }
        }
        /*
      non-dma activity triggered work
    */
    }
}

void codec_buff_write(uint8_t *buf, int len)
{
	;
}

void i2c_init(void)
{
    if (MXC_I2C_Init(CODEC_I2C, 1, 0) != E_NO_ERROR)
        blink_halt("Error initializing I2C controller");

    MXC_I2C_SetFrequency(CODEC_I2C, CODEC_I2C_FREQ);
}

void codec_init(void)
{
    if (max9867_init(CODEC_I2C, CODEC_MCLOCK, 1) != E_NO_ERROR)
        blink_halt("Error initializing MAX9867 CODEC");

    if (max9867_enable_playback(1) != E_NO_ERROR)
        blink_halt("Error enabling playback path");

    if (max9867_playback_volume(-6, -6) != E_NO_ERROR)
        blink_halt("Error setting playback volume");

//    if (max9867_enable_record(1) != E_NO_ERROR)
//        blink_halt("Error enabling record path");

//    if (max9867_adc_level(-12, -12) != E_NO_ERROR)
//        blink_halt("Error setting ADC level");

//    if (max9867_linein_gain(-6, -6) != E_NO_ERROR)
//        blink_halt("Error setting Line-In gain");
}

void i2s_init(void)
{
    mxc_i2s_req_t req;

#define I2S_CRUFT_PTR (void *)UINT32_MAX
#define I2S_CRUFT_LEN UINT32_MAX

    req.wordSize = MXC_I2S_DATASIZE_HALFWORD;
    req.sampleSize = MXC_I2S_SAMPLESIZE_SIXTEEN;
    req.justify = MXC_I2S_MSB_JUSTIFY;
    req.wsPolarity = MXC_I2S_POL_NORMAL;
    req.channelMode = MXC_I2S_INTERNAL_SCK_WS_0;
    req.stereoMode = MXC_I2S_STEREO;

    req.bitOrder = MXC_I2S_MSB_FIRST;
    req.clkdiv = CLK_DIV;

    req.rawData = NULL;
    req.txData = I2S_CRUFT_PTR;
    req.rxData = I2S_CRUFT_PTR;
    req.length = I2S_CRUFT_LEN;

    if (MXC_I2S_Init(&req) != E_NO_ERROR)
        blink_halt("Error initializing I2S");

    MXC_I2S_SetFrequency(MXC_I2S_EXTERNAL_SCK_EXTERNAL_WS, 0);
}

#include "ff.h"
#include "mad.h"
static enum mad_flow input(void *data,
		    struct mad_stream *stream)
{
  struct buffer *buffer = data;

  if (!buffer->length)
    return MAD_FLOW_STOP;

  mad_stream_buffer(stream, buffer->start, buffer->length);

  buffer->length = 0;

  return MAD_FLOW_CONTINUE;
}

static inline signed int scale(mad_fixed_t sample)
{
  /* round */
  sample += (1L << (MAD_F_FRACBITS - 16));

  /* clip */
  if (sample >= MAD_F_ONE)
    sample = MAD_F_ONE - 1;
  else if (sample < -MAD_F_ONE)
    sample = -MAD_F_ONE;

  /* quantize */
  return sample >> (MAD_F_FRACBITS + 1 - 16);
}

static enum mad_flow output(void *data,
		     struct mad_header const *header,
		     struct mad_pcm *pcm)
{
  unsigned int nchannels, nsamples;
  mad_fixed_t const *left_ch, *right_ch;

  /* pcm->samplerate contains the sampling frequency */

  nchannels = pcm->channels;
  nsamples  = pcm->length;
  left_ch   = pcm->samples[0];
  right_ch  = pcm->samples[1];

  for(int i=0; i< nsamples; i++) {
	  i2s_rx_buffer[0][i] = scale(*left_ch++);
	  i2s_rx_buffer[1][i] = scale(*right_ch++);
  }
  return MAD_FLOW_CONTINUE;
}
static enum mad_flow error(void *data,
		    struct mad_stream *stream,
		    struct mad_frame *frame)
{
  struct buffer *buffer = data;

  fprintf(stderr, "decoding error 0x%04x (%s)\n",
	  stream->error, mad_stream_errorstr(stream));

  /* return MAD_FLOW_BREAK here to stop decoding (and propagate an error) */

  return MAD_FLOW_CONTINUE;
}

int decode(const char *path)
{
  struct mad_decoder decoder;
  int result;
  FRESULT err; //FFat Result (Struct)
  FIL file;

  if ((err = f_open(&file, (const TCHAR*)path, FA_READ)) != FR_OK) {
      printf("Error opening file: %s\n", FF_ERRORS[err]);
      return -1;
  }

  printf("File opened!\n");

  /* initialize our private message structure */

  /* configure input, output, and error functions */

  mad_decoder_init(&decoder, &file,
		   input, 0 /* header */, 0 /* filter */, output,
		   error, 0 /* message */);

  /* start decoding */

  result = mad_decoder_run(&decoder, MAD_DECODER_MODE_SYNC);

  /* release the decoder */

  mad_decoder_finish(&decoder);

  return result;
}
#if 0
int main(void)
{
#if defined(BOARD_FTHR_REVA)
    /* Wait for PMIC 1.8V to become available, about 180ms after power up. */
    MXC_Delay(MXC_DELAY_MSEC(200));
#endif

    /* Switch to 100 MHz clock */
    MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
    SystemCoreClockUpdate();

    printf("***** MAX9867 CODEC DMA Loopback Example *****\n");

    printf("Waiting...\n");

    /* DO NOT DELETE THIS LINE: */
    MXC_Delay(MXC_DELAY_SEC(2)); /* Let debugger interrupt if needed */

    printf("Running...\n");

    i2c_init();
    codec_init();
    i2s_init();

    dma_work_loop();
}
#endif
