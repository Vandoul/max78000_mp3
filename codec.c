/*
 * codec.c
 *
 *  Created on: 2022��12��26��
 *      Author: wakoj
 */

#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include "mxc.h"
#include "max9867.h"
#include <ff.h>
#include "libmad/mad.h"

#undef USE_I2S_INTERRUPTS

#define CODEC_I2C MXC_I2C1
#define CODEC_I2C_FREQ 100000

#define CODEC_MCLOCK 12288000
#define BITS_PER_CHANNEL 16
#define CHANNELS_PER_FRAME 2
#define SAMPLE_RATE 24000

#define BIT_CLK(srate, ssize)           ((srate) * CHANNELS_PER_FRAME * (ssize))
#define CLK_DIV(srate, ssize)           (((CODEC_MCLOCK / 2) / BIT_CLK(srate, ssize)) - 1)

#if 0
static char printBuff[1024];
void myprintf(const char *fmt, ...)
{
    va_list va;
    va_start(va, fmt);
    vsnprintf(printBuff, sizeof(printBuff), fmt, va);
    va_end(va);
    puts(printBuff);
}
#else
#define myprintf(fmt,args...)		printf(fmt,##args)
#endif

#define FRAME_BUFFER_SIZE   2048
#define FRAME_BUFFER_NUM    2

volatile int dma_is_run = 0;
int dma_ch_tx, dma_ch_rx;
#define FRAME_BUFFER_FLAG_EMPTY_NO_OWNER        0xFF01
#define FRAME_BUFFER_FLAG_READY_NO_OWNER        0xFF00
struct frame_buffer{
    int index;
    union {
        struct {
            uint8_t is_empty;
            int8_t channel;
        };
        uint16_t flag;
    };
    uint16_t bytes;
    uint16_t buff[FRAME_BUFFER_SIZE];
};
#define READ_BUFF_SIZE				2048
struct read_buffer{
    FIL *f;
    int is_first_frame;
    int size;
    uint8_t buff[READ_BUFF_SIZE];
};
struct frame_buffer frame_buffer[FRAME_BUFFER_NUM] __aligned(4);
// buff init
void initBuff(void)
{
    for(int i=0; i<FRAME_BUFFER_NUM; i++) {
        frame_buffer[i].index = i;
        frame_buffer[i].flag = FRAME_BUFFER_FLAG_EMPTY_NO_OWNER;
        frame_buffer[i].bytes = 0;
    }
}
// set buff flag
inline void setBuffFlag(struct frame_buffer *buff, uint16_t flag)
{
    buff->flag = flag;
}
// buff is empty
struct frame_buffer *getEmptyBuff(void)
{
    for(int i=0; i<FRAME_BUFFER_NUM; i++) {
        if(frame_buffer[i].is_empty) {
            return &frame_buffer[i];
        }
    }
    return NULL;
}
// set buff owner
void setBuffChannel(struct frame_buffer *buff, int ch)
{
    buff->channel = ch;
}
// set buff ready
void setBuffReady(struct frame_buffer *buff)
{
    buff->is_empty = 0;
}
// get ready buff
struct frame_buffer *getReadyBuff(void)
{
    for(int i=0; i<FRAME_BUFFER_NUM; i++) {
        // if((frame_buffer[i].is_empty == 0) && (frame_buffer[i].channel == -1)) {
        if(frame_buffer[i].flag == FRAME_BUFFER_FLAG_READY_NO_OWNER) {
            return &frame_buffer[i];
        }
    }
    return NULL;
}
// release buff
void releaseReadyBuffAuto()
{
    for(int i=0; i<FRAME_BUFFER_NUM; i++) {
        if(frame_buffer[i].channel != -1) {
            setBuffFlag(&frame_buffer[i], FRAME_BUFFER_FLAG_EMPTY_NO_OWNER);
        }
    }
}
void printBuffFlags()
{
    myprintf("0x%04x,0x%04x\n", frame_buffer[0].flag, frame_buffer[1].flag);
}

/* hardware interface */
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
    // dma_flag = 1;
    MXC_DMA_Handler();
}

void dma_init(void)
{
    MXC_NVIC_SetVector(DMA0_IRQn, dma_handler);
    MXC_NVIC_SetVector(DMA1_IRQn, dma_handler);
    NVIC_EnableIRQ(DMA0_IRQn);
    NVIC_EnableIRQ(DMA1_IRQn);
}
// char cb_msg[2]={"*0"};
void dma_callback(int channel, int result)
{
    (void) result;
    releaseReadyBuffAuto();
    struct frame_buffer *buff = getReadyBuff();
    MXC_DMA_ReleaseChannel(channel);
    if (buff) {
        channel = MXC_I2S_TXDMAConfig(buff->buff, buff->bytes);
        dma_is_run = 1;
        setBuffChannel(buff, channel);
        // myprintf("[%d:%p:%d]\n", buff->index, buff->buff,buff->bytes);
    } else {
        dma_is_run = 0;
        // cb_msg[1] = '0'+channel;
        // puts(cb_msg);
    }
}
#if 0
void dma_work_loop(void)
{
    int trig = 0;

    dma_init();
    MXC_I2S_RegisterDMACallback(dma_callback);
    dma_ch_tx = MXC_I2S_TXDMAConfig(i2s_rx_buffer[1], sizeof(i2s_rx_buffer[0]));
    dma_ch_rx = MXC_I2S_RXDMAConfig(i2s_rx_buffer[0], sizeof(i2s_rx_buffer[0]));

    for (;;) {
        if (dma_flag) {
            dma_flag = 0;
            /* dma activity triggered work */
            if (++trig == SAMPLE_RATE / I2S_DMA_BUFFER_SIZE) {
                trig = 0;
                LED_Toggle(LED2);
            }
        }
        /* non-dma activity triggered work */
    }
}
#endif
void i2c_init(void)
{
    if (MXC_I2C_Init(CODEC_I2C, 1, 0) != E_NO_ERROR)
        blink_halt("Error initializing I2C controller");

    MXC_I2C_SetFrequency(CODEC_I2C, CODEC_I2C_FREQ);
}
void i2c_deinit(void)
{
    MXC_I2C_Reset(CODEC_I2C);
}

void codec_init(void)
{
    if (max9867_init(CODEC_I2C, CODEC_MCLOCK, 1) != E_NO_ERROR)
        blink_halt("Error initializing MAX9867 CODEC");

    if (max9867_enable_playback(1) != E_NO_ERROR)
        blink_halt("Error enabling playback path");

    // if (max9867_playback_volume(-6, -6) != E_NO_ERROR)
    if (max9867_playback_volume(0, 0) != E_NO_ERROR)
        blink_halt("Error setting playback volume");
}

void i2s_init(int srate, int ssize)
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
    req.clkdiv = CLK_DIV(srate, ssize);

    req.rawData = NULL;
    req.txData = I2S_CRUFT_PTR;
    req.rxData = I2S_CRUFT_PTR;
    req.length = I2S_CRUFT_LEN;

    if (MXC_I2S_Init(&req) != E_NO_ERROR)
        blink_halt("Error initializing I2S");

    // change channel mode
    MXC_I2S_SetFrequency(MXC_I2S_EXTERNAL_SCK_EXTERNAL_WS, 0);
}
void i2s_deinit()
{
    MXC_I2S_Shutdown();
}

void max9867_hwif_init(int sr, int ss)
{
    i2c_init();
    codec_init();
    i2s_init(sr, ss);
    dma_init();
    MXC_I2S_RegisterDMACallback(dma_callback);
}
void max9867_hwif_deinit(int sr)
{
    i2s_deinit();
    i2c_deinit();
}
/* libmad callbacks. */
int readMP3_frame(FIL *f, uint8_t *buff, int *br, int *sr);
static enum mad_flow input(void *data,
		    struct mad_stream *stream)
{
    struct read_buffer *buff = (struct read_buffer *)data;
    if(buff->size > 0) {
        mad_stream_buffer(stream, buff->buff, READ_BUFF_SIZE/* buff->size */);
    } else {
        return MAD_FLOW_STOP;
    }

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
static inline uint32_t scale_combine(mad_fixed_t lsample, mad_fixed_t rsample)
{
    uint32_t combine = (scale(lsample)&0xFFFF)|((scale(rsample)&0xFFFF)<<16);
    return combine;
}

/*
1：收码显示为HEX格式。
2：下位机发送自定义数据，格式为：0x88+FUN+LEN+DATA+SUM
      FUN可以是 0xA1到0xAA，共10个；LEN为DATA的长度（不包括0x88、FUN、LEN、SUM）。
      SUM是0x88一直到DATA最后一字节的和，uint8格式。
    （记得打开需要使用帧的开关，更改设置后点击保存设置使设置生效）
3：数据可以是uint8、int16、uint16、int32、float这几个常用格式，多字节数据高位在前。
4：共有20个数据存储器，每个存储器的数据可以分别设置为来自10个自定义帧的30个数据。
5：高速通讯时（2ms一帧数据或者更快），请关闭高级收码页面的数据显示按钮和基本收码，否则更新过快有可
      能会造成程序卡死。
6：飞控显示对应的帧FUN为0xAF，（帧格式：0x88+0xAF+0x1C+ACC DATA+GYRO DATA+MAG DATA+ANGLE DATA
      + 0x00 0x00 + 0x00 0x00+SUM，共32字节，ACC/GYRO/MAG/ANGLE(roll/pitch/yaw)数据为int16格式，其
      中ANGLE的roll和pitch数据为实际值乘以100以后得到的整数值，yaw为乘以10以后得到的整数值，
      上位机在显示时再 除以100和10）。
7：遥控,电机pwm,电压显示对应的帧FUN为0xAE，（帧格式：0x88+0xAE+0x12+THROT YAW ROLL PITCH 
      +AUX1 2 3 4 5 + PWM:1 2 3 4 + VOTAGE + SUM，共28字节），数据为uint16格式，遥控数据最小在1000左右，
      最大在2000左右。数据都为uint16格式,其中pwm范围1-100,votage为实际值*100。
      小技巧：如果高速通讯时是为了画波形，就只开波形显示，并只保留需要观察的波形，如果是为了观察数
      据，就关闭波形显示，只保留收码显示，这样可以加快程序响应速度。
7：最快通讯速度测试过下位机用500K波特率，每1ms发送32字节的数据，上位机显示其中6条波形，OK！
    （有可能和电脑配置有关）
三：波形显示
1：共有20条波形，对应20个数据存储器。
2：双击波形绘制区域，可以打开波形显示开关。
3：按住Ctrl用鼠标左键点击某一条波形，可以显示数据标签，再次点击隐藏。
4：按住鼠标左键，在绘图区域从一点向右下方拖动，然后松开，可以放大显示框住的波形区域，可以多次放大；
5：按住鼠标左键，在绘图区域从一点向左上方拖动，然后松开，可以将放大后的波形还原。
6：按住鼠标右键，在绘图区域上下左右拖动，可以移动波形。
7：显示波形时按F9键，可以打开波形高级设置。
*/
static mxc_uart_regs_t *const console_uart = MXC_UART_GET_UART(CONSOLE_UART);
void uart_out_data(uint32_t ldata, uint32_t rdata)
{
    //0x88+FUN+LEN+DATA+SUM;FUN=0xA1~0xAA,LEN=2
    uint8_t buff[16];
    int size = 16;
    //frame1
    buff[0] = 0x88;
    buff[1] = 0xA1;
    buff[2] = 0x02;
    buff[3] = (ldata>>24)&0xFF;
    buff[4] = (ldata>>16)&0xFF;
    buff[5] = (ldata>>8)&0xFF;
    buff[6] = (ldata)&0xFF;
    buff[7] = buff[0]+buff[1]+buff[2]+buff[3]+buff[4]+buff[5]+buff[6];
    //frame2
    buff[8] = 0x88;
    buff[9] = 0xA2;
    buff[10] = 0x02;
    buff[11] = (rdata>>24)&0xFF;
    buff[12] = (rdata>>16)&0xFF;
    buff[13] = (rdata>>8)&0xFF;
    buff[14] = (rdata)&0xFF;
    buff[15] = buff[8]+buff[9]+buff[10]+buff[11]+buff[12]+buff[13]+buff[14];
    MXC_UART_Write(console_uart, buff, &size);
}
#if 1
static enum mad_flow output(void *data,
        struct mad_header const *header,
        struct mad_pcm *pcm)
{
    struct read_buffer *buff = (struct read_buffer *)data;
    unsigned int channels, nsamples;
    mad_fixed_t const *left_ch, *right_ch;
    struct frame_buffer *fbuff = NULL;
    do {
        fbuff = getEmptyBuff(); //wait frame buff ready.
    } while(fbuff == NULL);

    /* pcm->samplerate contains the sampling frequency */
    (void)channels;
    // channels  = pcm->channels;
    nsamples  = pcm->length;
    left_ch   = pcm->samples[0];
    right_ch  = pcm->samples[1];

    for(int i=0; i< nsamples; i++) {
        fbuff->buff[i] = scale_combine(*left_ch++, *right_ch++);
    }
    nsamples = sizeof(uint16_t)*nsamples;
    // myprintf("<%d,%d>\n", fbuff->index, (int)nsamples);
    fbuff->bytes = nsamples;
    setBuffReady(fbuff);
    channels = buff->is_first_frame;
    if(buff->is_first_frame) {
        buff->is_first_frame = 0;
        max9867_hwif_init(header->samplerate, BITS_PER_CHANNEL);
    }
    if(!dma_is_run) {
        if(channels) {
            myprintf("+<%d,%d,%d>\n", fbuff->index, (int)channels, (int)nsamples);
            printBuffFlags();
        }
        channels = MXC_I2S_TXDMAConfig(fbuff->buff, nsamples);
        setBuffChannel(fbuff, channels);
        dma_is_run = 1;
    }
    // load new data.
    buff->size = readMP3_frame(buff->f, &buff->buff[0], NULL, NULL);
    if(buff->size < 0) {
        buff->size = readMP3_frame(buff->f, &buff->buff[0], NULL, NULL);
        if(buff->size < 0) {
            buff->size = 0;
        }
    }
    return MAD_FLOW_CONTINUE;
}
#else
static enum mad_flow output(void *data,
        struct mad_header const *header,
        struct mad_pcm *pcm)
{
    struct read_buffer *buff = (struct read_buffer *)data;
    unsigned int nsamples;
    mad_fixed_t const *left_ch, *right_ch;
    /* pcm->samplerate contains the sampling frequency */
    nsamples  = pcm->length;
    left_ch   = pcm->samples[0];
    right_ch  = pcm->samples[1];
    for(int i=0; i< nsamples; i++) {
        uart_out_data(*left_ch++, *right_ch++);
    }
    // load new data.
    buff->size = readMP3_frame(buff->f, &buff->buff[0], NULL, NULL);
    if(buff->size < 0) {
        buff->size = readMP3_frame(buff->f, &buff->buff[0], NULL, NULL);
        if(buff->size < 0) {
            buff->size = 0;
        }
    }
    return MAD_FLOW_CONTINUE;
}
#endif
static enum mad_flow error(void *data,
		    struct mad_stream *stream,
		    struct mad_frame *frame)
{
    return MAD_FLOW_CONTINUE;
}
extern const char* FF_ERRORS[20];
struct ID3V2_raw{
    char header[3];
    char ver;
    char revision;
    char flag;
    char size[4];
};
struct ID3V2_info{
    char ver;
    uint8_t flag;
    int size;
};
union ID3V2_frame_header{
    uint8_t u8_data[4];
    uint32_t u32_data;
    struct {
        //byte1
        uint32_t emphasis:2;
        uint32_t original:1;
        uint32_t copyright:1;
        uint32_t mode_extension:2;
        uint32_t channel_mode:2;
        //byte2
        uint32_t private_bit:1;
        uint32_t padding_bit:1;
        uint32_t sampling_rate:2;
        uint32_t bitrate:4;
        //byte3-4
        uint32_t protect_bit:1;
        uint32_t layer:2;
        uint32_t version:2;
        uint32_t sync:11;
    }bit;
};
struct ID3V2_tag{
    char tag[4];
    uint8_t size[4];
    uint16_t flag;
};

int loadMP3_info(FIL *f, struct ID3V2_info *info)
{
    struct ID3V2_raw raw;
    UINT size;
    f_lseek(f, 0);
    int err = f_read(f, &raw, sizeof(struct ID3V2_raw), &size);
    if(FR_OK != err) {
        myprintf("Error reading file: %s\n", FF_ERRORS[err]);
        return -1;
    }
    if(strncmp(raw.header, "ID3", 3) == 0) {
        info->ver = raw.ver;
        info->flag = raw.flag;
        info->size = raw.size[3]<<21|raw.size[2]<<14|raw.size[1]<<7|raw.size[0];
    } else {
        myprintf("invalid header: %s\n", raw.header);
        return -1;
    }
    return 0;
}
#define MP3_TAG_IS_TTAG(tag)    (((tag)[0] == 'T') && \
    ((((tag)[1]>='0') && ((tag)[1]<='9')) || (((tag)[1]>='A') && ((tag)[1]<='Z'))) && \
    ((((tag)[2]>='0') && ((tag)[2]<='9')) || (((tag)[2]>='A') && ((tag)[2]<='Z'))) && \
    ((((tag)[3]>='0') && ((tag)[3]<='9')) || (((tag)[3]>='A') && ((tag)[3]<='Z'))))
#define MP3_TAG_IS_APIC(tag)    (((tag)[0]=='A') && ((tag)[1]=='P') && ((tag)[2]=='I') && ((tag)[3]=='C'))
#define MP3_TAG_IS_GEOB(tag)    (((tag)[0]=='G') && ((tag)[1]=='E') && ((tag)[2]=='O') && ((tag)[3]=='B'))
#define MP3_TAG_CHECK(tag)      (MP3_TAG_IS_GEOB(tag) || MP3_TAG_IS_APIC(tag) || MP3_TAG_IS_TTAG(tag))
void jumpMP3_tag(FIL *f)
{
    struct ID3V2_tag tag;
    UINT size;
    int err = FR_OK;
    while(1) {
        f_read(f, &tag, sizeof(struct ID3V2_tag), &size);
        if(FR_OK != err) {
            myprintf("Error reading file: %s\n", FF_ERRORS[err]);
            return ;
        }
        if(size != sizeof(struct ID3V2_tag)) {
            myprintf("Read tag failed!");
            f_lseek(f, f_tell(f) - size);
            return ;
        }
        if(MP3_TAG_CHECK(tag.tag)) {
            size = tag.size[0]<<24|tag.size[1]<<16|tag.size[2]<<8|tag.size[3];
            f_lseek(f, f_tell(f) + size);
            continue;
        }
        f_lseek(f, f_tell(f) - size);
        break;
    }
}
static int findMP3_symb(uint8_t *buff, int size)
{
    for(int i=1; i<size; i++) {
        if((buff[i-1] == 0xff) && ((buff[i] & 0xe0) == 0xe0)) {
            return (i - 1);
        }
    }
    return -1;
}
/**
 * @return: >0=frame size, others=invalid frame.
 * */
static int getMP3_frame_size(union ID3V2_frame_header *header, int *br, int *sr)
{
    int size = 0;
    int bitrate = 0;
    int samplingrate = 0;
    uint8_t tmp = (header->bit.bitrate<<4)|(header->bit.version<<2)|header->bit.layer;
    switch(tmp) {
    case 0b00011111://0001,V1,L1
    case 0b00011110://0001,V1,L2
    case 0b00011101://0001,V1,L3
    case 0b00011011://0001,V2,L1
        bitrate = 32000;
        break;
    case 0b01101111://0110,V1,L1
    case 0b10101110://1010,V1,L2
    case 0b10111101://1011,V1,L3
    case 0b10111011://1100,V2,L1
        bitrate = 192000;
        break;
    case 0b10101111://1010,V1,L1
    case 0b11011110://1101,V1,L2
    case 0b11101101://1110,V1,L3
        bitrate = 320000;
        break;
    default:
        myprintf("unknow bitrate!!!0x%02x\n", tmp);
        return -1;
    }
    tmp = (header->bit.sampling_rate<<2)|(header->bit.version);
    switch(tmp) {
    case 0b0011: //00,MPEG1
        samplingrate = 44100;
        break;
    case 0b0111: //01,MPEG1
        samplingrate = 48000;
        break;
    case 0b1011: //10,MPEG1
        samplingrate = 32000;
        break;
    case 0b0010: //00,MPEG2
        samplingrate = 22050;
        break;
    case 0b0110: //01,MPEG2
        samplingrate = 24000;
        break;
    case 0b1010: //10,MPEG2
        samplingrate = 16000;
        break;
    default:
        myprintf("unknown samplingrate!!!0x%02x\n", tmp);
        return -1;
    }
    tmp = (header->bit.version<<2)|header->bit.layer;
    switch(tmp) {
    case 0b1111://V1,L1
        size = 48;
        break;
    case 0b1110://V1,L2
    case 0b1101://V1,L3
        size = 144;
        break;
    case 0b1011://V2,L1
        size = 24;
        break;
    default:
        myprintf("unknow size!!!0x%02x\n", tmp);
        return -1;
    }
    if(br) {
        *br = bitrate;
    }
    if(sr) {
        *sr = samplingrate;
    }
    size = ((size*bitrate)/samplingrate+header->bit.padding_bit);
    // myprintf("frame size:%d,br:%d,sr:%d\n", size, bitrate, samplingrate);
    return size;
}
struct read_buffer readBuff;
int readMP3_frame(FIL *f, uint8_t *buff, int *br, int *sr)
{
    int err;
    UINT size;
    UINT size1;
    UINT tmp;
    int pos = 0;
    uint8_t *preadbuff = &readBuff.buff[0];
    f_lseek(f, f_tell(f));
    for(;;) {
        err = f_read(f, preadbuff+pos, READ_BUFF_SIZE-pos, &size);
        if(FR_OK == err) {
            size += pos;
            pos = findMP3_symb(preadbuff, size);
            if(pos != -1) {
                size1 = size - pos; //valid data size.
                memcpy(preadbuff, preadbuff+pos, size1); //move data.
                tmp = f_tell(f) - size1;
                // myprintf("find symb:%p,s:%d,s2:%d,p:%d\n", tmp, size, size1, pos);
                // myprintf("@0x%08x,%d\n", tmp, pos);
                if(size1 < sizeof(union ID3V2_frame_header)) {
                    size = 0;
                    err = f_read(f, preadbuff+size1, sizeof(union ID3V2_frame_header)-size1, &size);
                    size += size1;
                    if((FR_OK != err) || (size < sizeof(union ID3V2_frame_header))) {
                        myprintf("read frame header failed!!!%s,%d,%p\n", FF_ERRORS[err], size, tmp);
                        return -1;
                    }
                } else {
                    size = size1;
                }
                union ID3V2_frame_header frame;
                frame.u8_data[3] = preadbuff[0];
                frame.u8_data[2] = preadbuff[1];
                frame.u8_data[1] = preadbuff[2];
                frame.u8_data[0] = preadbuff[3];
                int fsize = getMP3_frame_size(&frame, br, sr);
                if(fsize <= 0) {
                    size -= 1;
                    memcpy(preadbuff, preadbuff+1, size);
                    pos = size;
                    continue;
                }
                if(fsize > size) {
                    size1 = 0;
                    err = f_read(f, preadbuff+size, fsize-size, &size1);
                    size += size1;
                    if((err != FR_OK) || (fsize != size)) {
                        myprintf("read frame data failed!!!%s,%d!=%d,%p\n", FF_ERRORS[err], size, fsize, tmp);
                        return -1;
                    }
                } else if(fsize < size) {
                    f_lseek(f, f_tell(f) - (size - fsize));
                }
                memcpy(buff, preadbuff, fsize);
                return fsize;
            } else if(size > 1) {
                pos = 1;
                preadbuff[0] = preadbuff[size-1];
                continue;
            } else {
                break;
            }
        } else {
            tmp = f_tell(f);
            myprintf("read buff failed! %s,%p,%d\n", FF_ERRORS[err], tmp, pos);
            return -1;
        }
    }
    return 0;
}
int decode(const char *path)
{
    struct ID3V2_info info;
    #if 0
    struct mad_decoder decoder;
    #else
    struct mad_stream stream;
    struct mad_frame frame;
    struct mad_synth synth;
    #endif
    int result = 0;
    FRESULT err; //FFat Result (Struct)
    FIL file;

    if ((err = f_open(&file, (const TCHAR*)path, FA_READ)) != FR_OK) {
        myprintf("Error opening file: %s\n", FF_ERRORS[err]);
        return -1;
    }

    myprintf("File opened!%d\n", f_tell(&file));
    do {
        if(loadMP3_info(&file, &info)) {
            myprintf("load info failed!\n");
            break;
        } else {
            myprintf("ver:%d, flag:%02x, size:%d(0x%x)\n", info.ver, info.flag, info.size, info.size);
        }

        jumpMP3_tag(&file);

        /* initialize our private message structure */
        /* init frame buffer. */
        int bitrate, samplingrate;
        readBuff.f = &file;
        readBuff.size = readMP3_frame(&file, readBuff.buff, &bitrate, &samplingrate);
        readBuff.is_first_frame = 1;
        myprintf("fs:%d,br:%d,sr:%d\n", readBuff.size, bitrate, samplingrate);
#if 0
        // /* configure input, output, and error functions */
        // mad_decoder_init(&decoder, &readBuff,
        //         input, 0 /* header */, 0 /* filter */, output,
        //         error, 0 /* message */);

        /* start decoding */
        // result = mad_decoder_run(&decoder, MAD_DECODER_MODE_SYNC);

        /* release the decoder */
        // mad_decoder_finish(&decoder);
#else
        mad_stream_init(&stream);
        mad_frame_init(&frame);
        mad_synth_init(&synth);
        mad_stream_options(&stream, 0);
        initBuff();
        while(1) {
            // input
            switch(input(&readBuff, &stream)) {
            case MAD_FLOW_STOP:
                goto done;
            case MAD_FLOW_BREAK:
                goto fail;
            case MAD_FLOW_CONTINUE:
            break;
            case MAD_FLOW_IGNORE:
            default:
            continue;
            }
            // decode
            if (mad_frame_decode(&frame, &stream) == -1) {
                if (!MAD_RECOVERABLE(stream.error))
                    break;
                switch (error(&readBuff, &stream, &frame)) {
                case MAD_FLOW_STOP:
                    goto done;
                case MAD_FLOW_BREAK:
                    goto fail;
                case MAD_FLOW_IGNORE:
                break;
                case MAD_FLOW_CONTINUE:
                default:
                continue;
                }
            }
            // 
            mad_synth_frame(&synth, &frame);
            // output
            switch(output(&readBuff, &frame.header, &synth.pcm)) {
            case MAD_FLOW_STOP:
                goto done;
            case MAD_FLOW_BREAK:
                goto fail;
            case MAD_FLOW_IGNORE:
            break;
            case MAD_FLOW_CONTINUE:
            default:
            continue;
            }
        }
        fail:
        result = -1;
        done:
        mad_synth_finish(&synth);
        mad_frame_finish(&frame);
        mad_stream_finish(&stream);
#endif
    } while(0);
    myprintf("decode end\n");

    f_close(&file);

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

    myprintf("***** MAX9867 CODEC DMA Loopback Example *****\n");

    myprintf("Waiting...\n");

    /* DO NOT DELETE THIS LINE: */
    MXC_Delay(MXC_DELAY_SEC(2)); /* Let debugger interrupt if needed */

    myprintf("Running...\n");

    i2c_init();
    codec_init();
    i2s_init();

    dma_work_loop();
}
#endif
