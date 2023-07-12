#include <main.h>

// send SR and BR ISSOURCE/ISSINK info in the BroadCast data
#ifdef ESP32
static const char *TAG = "ESP_NOW_32";
#endif
#ifdef ESP32_S3
static const char *TAG = "ESP_NOW_32-S3";
#endif

// #pragma region ESPNOW_CONFIG
static uint16_t ESPNOW_SEND_COUNT = 64; // set so that count reaches zero before next batch of data is retrieved
static uint8_t ESPNOW_SEND_LEN = 250;   // set to SAMPLE_RATE / x  <= 240 and   SAMPLE_RATE % SEND_LEN = 0
#ifdef IS_SOURCE
static uint16_t ESPNOW_SEND_DELAY = 0; // 2; //set so that all data sent before next batch of i2s data // Delay between sending two ESPNOW data, unit: ms. //  range 0 65535 default 1000
#else
#ifdef IS_SINK
static bool paired_w_source;
static uint16_t ESPNOW_SEND_DELAY = 1000; // 2; //set so that all data sent before next batch of i2s data // Delay between sending two ESPNOW data, unit: ms. //  range 0 65535 default 1000
#endif
#endif

#define I2S_BUF_SIZ 10240
// #pragma endregion ESPNOW_CONFIG

// #pragma region I2S_CONFIG
static uint32_t SAMPLE_RATE = 48000;                             // 96000U;
static i2s_data_bit_width_t BIT_DEPTH = I2S_DATA_BIT_WIDTH_32BIT; // I2S_DATA_BIT_WIDTH_32BIT;
static uint16_t DMA_FRAME_NUM = 128;
static uint8_t DMA_DESC_NUM = 2; // 3 or 4 seems to work best with minimal delay
static uint8_t bytes_per_sample = 4;
static size_t bytes_to_read;
//ring buffer index
uint32_t rbuffer_start, rbuffer_end;
uint8_t PAYLOAD_SIZE;

// static uint16_t I2S_BUF_SIZ = 2048;
RingbufHandle_t rbuf_handle;
static bool paired;
SemaphoreHandle_t xsh = NULL;
SemaphoreHandle_t i2s_sync_h = NULL;
#ifdef IS_SINK
uint32_t ESPNOW_I2S_BUF[I2S_BUF_SIZ];
#endif
volatile size_t bSent;
static uint16_t espnow_seq[ESPNOW_DATA_MAX] = {0, 0};
static uint8_t filler_data[ESPNOW_MAX_PAYLOAD_SIZE] = {0};
//led indication for paired

//remove pairing after no data is received for few seconds

//make this a ring buffer


#ifdef IS_SOURCE
DMA_ATTR uint32_t I2S_RX_BUF[I2S_BUF_SIZ];
size_t bRead;
i2s_chan_handle_t i2s0_rx_handler;
i2s_chan_config_t i2s_ch_conf = {
    .id = I2S_NUM_0,
    .role = I2S_ROLE_SLAVE,//MASTER,
    .dma_desc_num = 4,
    .dma_frame_num = 256,
    .auto_clear = 1,
};
// I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
i2s_std_config_t i2s_port_conf = {
    .clk_cfg = {
        .sample_rate_hz = 96000,
#ifdef ESP32
        .clk_src = I2S_CLK_SRC_APLL, //I2S_CLK_SRC_PLL_160M,// I2S_CLK_SRC_APLL,
#endif
#if ESP32_S3
        .clk_src = I2S_CLK_SRC_DEFAULT,
#endif
        .mclk_multiple = I2S_MCLK_MULTIPLE_512,//I2S_MCLK_MULTIPLE_512,
    },
    // I2S_STD_CLK_DEFAULT_CONFIG(96000),
    .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
    .gpio_cfg = {
        .mclk = I2S_GPIO_UNUSED,
        .bclk = I2S_BCK_IO,
        .ws = I2S_WS_IO,
        .dout = I2S_GPIO_UNUSED,
        .din = I2S_DI_IO,
        .invert_flags = {
            .mclk_inv = false,
            .bclk_inv = false,
            .ws_inv = false,
        },
    },
};
#endif

#ifdef IS_SINK
size_t bWritten;
DMA_ATTR uint32_t I2S_TX_BUF[I2S_BUF_SIZ];
i2s_chan_handle_t i2s0_tx_handler;
i2s_chan_config_t i2s_ch_conf = {
    .id = I2S_NUM_0,
    .role = I2S_ROLE_MASTER,
    .dma_desc_num = 8,
    .dma_frame_num = 256,
    .auto_clear = 1,
};

// I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
i2s_std_config_t i2s_port_conf = {
    .clk_cfg = {
        .sample_rate_hz = 96000,
        .clk_src = I2S_CLK_SRC_DEFAULT,
        .mclk_multiple = I2S_MCLK_MULTIPLE_256,
    },
    // I2S_STD_CLK_DEFAULT_CONFIG(96000),
    .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
    .gpio_cfg = {
        .mclk = I2S_GPIO_UNUSED,
        .bclk = I2S_BCK_IO,
        .ws = I2S_WS_IO,
        .dout = I2S_DO_IO,
        .din = I2S_GPIO_UNUSED,
        .invert_flags = {
            .mclk_inv = false,
            .bclk_inv = false,
            .ws_inv = false,
        },
    },
};

// static int32_t I2S_ESPNOW_BUF[I2S_BUF_SIZ];//double buffer
#endif

// #pragma endregion I2S_CONFIG

//



static void set_i2s_params()
{
    bytes_per_sample = BIT_DEPTH / 8;
    /*Taking audio as our example - suppose we are sampling in stereo at 44.1KHz with 16 bits per sample - this gives a data transfer rate of around 176KBytes per second.

If we had a DMA buffer size of 8 samples, we’d be interrupting the CPU every 181 microseconds.

If we had a buffer size of 1024 samples, we’d be interrupting the CPU every 23 milliseconds.*/

    // I2S_BUF_SIZ = i2s_ch_conf.dma_frame_num * I2S_SLOT_MODE_STEREO * I2S_SLOT_BIT_WIDTH_32BIT / 8;
    // frame num
    /*Determine the interrupt interval.
    Generally, when data lost happens, the bigger the interval, the better, which helps to reduce the interrupt times.
    This means dma_frame_num should be as big as possible while the DMA buffer size is below the maximum value of 4092. The relationships are:*/
    /*interrupt_interval(unit: sec) = dma_frame_num / sample_rate
    dma_buffer_size = dma_frame_num * slot_num * data_bit_width / 8 <= 4092*/

    // dma_desc_num
    /*Determine dma_desc_num. dma_desc_num is decided by the maximum time of i2s_channel_read polling cycle.
     All the received data is supposed to be stored between two i2s_channel_read. This cycle can be measured by a timer or an outputting GPIO signal. The relationship is:*/
    // dma_desc_num > polling_cycle / interrupt_interval

    // buffersize
    /*Determine the receiving buffer size.
    The receiving buffer offered by users in i2s_channel_read should be able to take all the data in all DMA buffers,
     which means that it should be larger than the total size of all the DMA buffers:*/

    // recv_buffer_size > dma_desc_num * dma_buffer_size

    /*
    sample_rate = 144000 Hz
    data_bit_width = 32 bits
    slot_num = 2
    polling_cycle = 10 ms*/

    /*dma_frame_num * slot_num * data_bit_width / 8 = dma_buffer_size <= 4092
    dma_frame_num <= 511
    interrupt_interval = dma_frame_num / sample_rate = 511 / 144000 = 0.003549 s = 3.549 ms
    dma_desc_num > polling_cycle / interrupt_interval = cell(10 / 3.549) = cell(2.818) = 3
    recv_buffer_size > dma_desc_num * dma_buffer_size = 3 * 4092 = 12276 bytes*/
}

static void espnow_deinit(espnow_send_param_t *send_param)
{
    free(send_param->buffer);
    free(send_param);
    vSemaphoreDelete(espnow_queue);
    esp_now_deinit();
}

// static uint16_t set_espnow_send_delay(){
//     //must send 12288 bytes before next i2s
//     //uint16_t delay;//ms
//     //uint32_t bytes_ps = SAMPLE_RATE * BIT_DEPTH * 2 / 8; //sample_rate * BIT_DEPTH* 2 / 8 = sample bytes per second
//     //I2S_BUF_SIZ / bytes_ps = seconds

//     //delay = I2S_BUF_SIZ;

//    // return delay;
// }

// set payload size small if sink
//  static uint8_t set_espnow_payload_size(){
//      uint8_t payload_size;
//      uint32_t i2s_data_byte_count = (BIT_DEPTH * SAMPLE_RATE * 2) / 8; //per second
//      payload_size = (uint8_t)ESPNOW_MAX_PAYLOAD_SIZE - sizeof(espnow_data_t);

//     while(i2s_data_byte_count % payload_size > 0 && i2s_data_byte_count ){

//     }

//     return payload_size;
// }

// static uint8_t set_espnow_send_count(){
//     //send count should be max by next i2s data
//     uint8_t payload_size;
//     uint32_t i2s_data_byte_count = (BIT_DEPTH * SAMPLE_RATE * 2) / 8; //per second
//     payload_size = (uint8_t)ESPNOW_MAX_PAYLOAD_SIZE - sizeof(espnow_data_t);

//     while(i2s_data_byte_count % payload_size > 0 && i2s_data_byte_count ){

//     }

//     return payload_size;
// }

#ifdef IS_SOURCE
// Task to read audio data from I2S and send it via ESP-NOW
IRAM_ATTR static void i2s_rec_task(void *pvParameters)
{
    size_t bytes_read;
    // uint8_t audio_buffer[1024];
    bytes_to_read = i2s_ch_conf.dma_desc_num * i2s_ch_conf.dma_frame_num * BIT_DEPTH * 2 / 8;
    while (1)
    {

        //recalculate bytes to read if the config changes

        // Read audio data from I2S
        // i2s_read(I2S_NUM, audio_buffer, sizeof(audio_buffer), &bytes_read, portMAX_DELAY);
        esp_err_t err_status;
        // ESP_LOGI(TAG, "Start I2S");

        // ESP_LOGI(TAG, "Start I2S Read");

        err_status = i2s_channel_read(i2s0_rx_handler, I2S_RX_BUF, bytes_to_read, &bytes_read, pdMS_TO_TICKS(1000));//portMAX_DELAY);
         //ESP_LOGI(TAG, "I2S Read, Copying to ESPNOW buffer");
         
        if(xRingbufferGetCurFreeSize(rbuf_handle) < bytes_read){//xRingbufferGetMaxItemSize(rbuf_handle) /4 ){//<= bytes_read){

            xSemaphoreTake(i2s_sync_h, pdMS_TO_TICKS(1000));
            continue;
        }
        bRead = bytes_read;
        //memcpy(ESPNOW_I2S_BUF, I2S_RX_BUF, bRead);
       // ESP_LOGI(TAG, "Took Semaphore");
       // if(xRingbufferGetCurFreeSize(rbuf_handle) < bytes_read){
        //    ESP_LOGE(TAG, "Overrun");
       // }
        if(xRingbufferSend(rbuf_handle, I2S_RX_BUF, bytes_read, pdMS_TO_TICKS(1000)) != pdTRUE){
            ESP_LOGI(TAG, "ERR failed to dump data into ring buffer");
        }
       // ESP_LOGI(TAG, "Dumped data into buffer %u", bytes_read);
        xSemaphoreGive(xsh);
        // ESP_LOGI(TAG, "I2S, Starting next read");
        // xSemaphoreTake(xsh);
        // xSemaphoreGive(i2s_sync_h);

        if (err_status == ESP_OK)
        {
        }

        // Send audio data via ESP-NOW
        // esp_now_send(NULL, (uint8_t*)I2S_RX_BUF, bytes_read);
    }
}
#ifdef EN_SPDIF
static void i2s_spdif_task(void *pvParameters)
{
    // size_t bytes_read;
    // uint8_t audio_buffer[1024];

    while (1)
    {
        esp_err_t err_status;
        spdif_write(I2S_TX_BUF, sizeof(I2S_TX_BUF));
        if (err_status == ESP_OK)
        {
        }
    }
}

#endif
#else
#ifdef IS_SINK
// Task to read audio data from I2S and send it via ESP-NOW
static void i2s_tx_task(void *pvParameters)
{
    // size_t bytes_read;
    // uint8_t audio_buffer[1024];

    while (1)
    {
        // Read audio data from I2S
        esp_err_t err_status;
        err_status = i2s_channel_write(i2s0_tx_handler, I2S_TX_BUF, bSent, &bWritten, portMAX_DELAY);
        if (err_status == ESP_OK)
        {
        }
    }
}
#ifdef EN_SPDIF
static void i2s_spdif_task(void *pvParameters)
{
    // size_t bytes_read;
    // uint8_t audio_buffer[1024];
    uint8_t *frame_buf_ptr;
    //uint8_t *frame_buf_ptr_wrap;
    size_t data_siz;
   // size_t data_siz_remaining;
    while (1)
    {
        // wait until data is ready
       // xSemaphoreGive(xsh);
        //xSemaphoreTake(i2s_sync_h, portMAX_DELAY);
        
        //try retrieve data from the ringbuffer
        frame_buf_ptr = (uint8_t*)xRingbufferReceiveUpTo(rbuf_handle, &data_siz, pdMS_TO_TICKS(5000), bSent);

        //two calls to RingbufferReceiveUpTo() if data wraps around
        //if()
        //esp_err_t err_status;
        //check that data was available in buffer
        if(frame_buf_ptr != NULL){
            //check for data wrap around
            if( data_siz < bSent ){
                //data wraps around so append wrapped data to retrieved data
                memcpy(I2S_TX_BUF, frame_buf_ptr, data_siz);
                vRingbufferReturnItem(rbuf_handle,(void*) frame_buf_ptr);
                frame_buf_ptr = (uint8_t*)xRingbufferReceiveUpTo(rbuf_handle, &data_siz, pdMS_TO_TICKS(5000), bSent-data_siz);
                memcpy(((uint8_t*)I2S_TX_BUF) + (bSent - data_siz), frame_buf_ptr, data_siz);
                spdif_write(I2S_TX_BUF, bSent);
                //spdif_write(I2S_TX_BUF, bSent);
                vRingbufferReturnItem(rbuf_handle,(void*) frame_buf_ptr);

            }else{
                //no wrap around so process as is
                spdif_write(frame_buf_ptr, data_siz);
                //spdif_write(I2S_TX_BUF, bSent);
                vRingbufferReturnItem(rbuf_handle,(void*) frame_buf_ptr);
            }
        }else{
            ESP_LOGI(TAG, "I2S too fast, Buffer Underrun waiting for more data");
            xSemaphoreTake(i2s_sync_h, portMAX_DELAY);
        }
        xSemaphoreGive(xsh);
        //xSemaphoreGive(xsh);
        // if (err_status == ESP_OK)
        // {
        // }
    }
}

#endif

#endif
#endif

// get i2s data
#ifdef IS_SOURCE
IRAM_ATTR /* inline */ void espnow_prepare_data(espnow_send_param_t *send_param)
#endif
#ifdef IS_SINK
    /* inline */ void espnow_prepare_data(espnow_send_param_t *send_param)
#endif
{
    espnow_data_t *buf = (espnow_data_t *)send_param->buffer;

    // send_param->len >= sizeof(espnow_data_t);
#ifdef IS_SOURCE
    // xSemaphoreTake(xsh, portMAX_DELAY);
#endif
    // remove type

    buf->type = IS_BROADCAST_ADDR(send_param->dest_mac) ? ESPNOW_DATA_BROADCAST : ESPNOW_DATA_UNICAST;
    buf->state = send_param->state;
    // xSemaphoreCreateBinary
    // buf->seq_num = espnow_seq[buf->type]++;
    buf->crc = 0;
    // buf->src_sink = send_param->src_sink;
    //  xsemaphorere
    // static uint32_t idx;
    size_t tmp;

#ifdef IS_SOURCE
    buf->src_sink = espnow_bd_source;
#endif
#ifdef IS_SINK
    buf->src_sink = espnow_bd_sink;
#endif

    if (!IS_BROADCAST_ADDR(send_param->dest_mac))
    {
#ifdef IS_SOURCE

    //implements ring buffer
        size_t buffer_size = 0;
        uint8_t *payload_buf;
        if (bSent == 0)
        {
            //ESP_LOGI(TAG, "Waiting for I2S data");
            buf->src_sink |= espnow_bd_SOF_L;
            espnow_seq[buf->type] = 0;
            //xSemaphoreGive(i2s_sync_h);
            //xSemaphoreTake(xsh, portMAX_DELAY);
             if (xRingbufferGetCurFreeSize(rbuf_handle) == xRingbufferGetMaxItemSize(rbuf_handle))
             {
                //     ESP_LOGE(TAG, "Underrun");
                while (xRingbufferGetCurFreeSize(rbuf_handle) > xRingbufferGetMaxItemSize(rbuf_handle) / 2)
                {
                    xSemaphoreTake(xsh, pdMS_TO_TICKS(50));
                }
             }
        }
           // payload_buf = xRingbufferReceive(rbuf_handle, &buffer_size, pdMS_TO_TICKS(100) );
            
           // memcpy(buf->payload, payload_buf, buffer_size);
        //ESP_LOGI(TAG, "transmitted  %d / %d", bSent, bRead);
        //  if(xRingbufferGetCurFreeSize(rbuf_handle) == xRingbufferGetMaxItemSize(rbuf_handle)){
        // //     ESP_LOGE(TAG, "Underrun");
        
        //     xSemaphoreTake(xsh, pdMS_TO_TICKS(1000));
        //  }
        if(bSent + PAYLOAD_SIZE > bRead){
            payload_buf = (uint8_t*)xRingbufferReceiveUpTo(rbuf_handle, &buffer_size, pdMS_TO_TICKS(100), bRead - bSent);//(size_t)PAYLOAD_SIZE);
            //ESP_LOGI(TAG, "Reading Buffer %u", bRead - bSent);
        }else{
            payload_buf = (uint8_t*)xRingbufferReceiveUpTo(rbuf_handle, &buffer_size, pdMS_TO_TICKS(100), PAYLOAD_SIZE);//(size_t)PAYLOAD_SIZE);
            //ESP_LOGI(TAG, "Reading Buffer %u", PAYLOAD_SIZE);
        }
        if(payload_buf != NULL){
            //check and process wrap around

            if(!(buffer_size == bRead - bSent) && !(buffer_size == PAYLOAD_SIZE)){
                memcpy(buf->payload, payload_buf, buffer_size);
                vRingbufferReturnItem(rbuf_handle, (void*)payload_buf);
                if(bSent + PAYLOAD_SIZE > bRead){
                    payload_buf = (uint8_t*)xRingbufferReceiveUpTo(rbuf_handle, &buffer_size, pdMS_TO_TICKS(100), bRead - bSent - buffer_size);//(size_t)PAYLOAD_SIZE);
                    memcpy(buf->payload + bRead - bSent, payload_buf, buffer_size);

                    //ESP_LOGI(TAG, "Reading Buffer %u", bRead - bSent);
                }else{
                    payload_buf = (uint8_t*)xRingbufferReceiveUpTo(rbuf_handle, &buffer_size, pdMS_TO_TICKS(100), PAYLOAD_SIZE - buffer_size);//(size_t)PAYLOAD_SIZE);
                    memcpy(buf->payload + PAYLOAD_SIZE - buffer_size, payload_buf, buffer_size);
                    //ESP_LOGI(TAG, "Reading Buffer %u", PAYLOAD_SIZE);
                }
                vRingbufferReturnItem(rbuf_handle, (void*)payload_buf);
            }else{
                //ESP_LOGI(TAG, "Read buffer %u",buffer_size );
                memcpy(buf->payload, payload_buf, buffer_size);
                vRingbufferReturnItem(rbuf_handle, (void*)payload_buf);
            }


            bSent += buffer_size;
            buf->seq_num = espnow_seq[buf->type]++;
            send_param->len = buffer_size + sizeof(espnow_data_t);
            if (bSent == bRead)
            {
                // allow i2s to retrieve new data
                // xSemaphoreGive(i2s_sync_h);
            // ESP_LOGI(TAG, "END  %d / %d", bSent, bRead);
                ESP_LOGI(TAG, "END  %d / %d ", xRingbufferGetCurFreeSize(rbuf_handle), xRingbufferGetMaxItemSize(rbuf_handle));

                buf->src_sink |= espnow_bd_EOF;
                //ESP_LOGI(TAG, "Send Done");
                bSent = 0;
                //bRead = 0;
            }
        }
        //ESP_LOGI(TAG, "Send  %d / %d", bSent, bRead);





        // uint8_t *ptr = ((uint8_t *)ESPNOW_I2S_BUF) + bSent;
        // tmp = bRead - bSent;
        // buf->seq_num = espnow_seq[buf->type]++;

        // if (tmp >= ESPNOW_MAX_PAYLOAD_SIZE - sizeof(espnow_data_t))
        // {
        //     memcpy(buf->payload, ptr, ESPNOW_MAX_PAYLOAD_SIZE - sizeof(espnow_data_t));
        //     bSent += ESPNOW_MAX_PAYLOAD_SIZE - sizeof(espnow_data_t);
        //     send_param->len = ESPNOW_MAX_PAYLOAD_SIZE;
        // }else
        // {
        //     memcpy(buf->payload, ptr, tmp);
        //     bSent += tmp;
        //     send_param->len = tmp + sizeof(espnow_data_t);
        // }
        // //  if(bSent == bRead){
        // //      ESP_LOGI(TAG, "Buf processed: %d, Buf Total:%d", bSent, bRead);
        // //  }
        // // ESP_LOGI(TAG, "data %d / %d sent", bsent, bRead);
        // // ESP_LOGI(TAG, "data %d / %d", bSent, bRead);
        // if (bSent == bRead)
        // {
        //     // allow i2s to retrieve new data
        //     // xSemaphoreGive(i2s_sync_h);
        //     //ESP_LOGI(TAG, "END  %d / %d", bSent, bRead);
        //     buf->src_sink |= espnow_bd_EOF;
        //     // ESP_LOGI(TAG, "Send Done");
        //     bSent = 0;
        //     bRead = 0;
        // }


#endif
    }
    else
    {
        buf->seq_num = espnow_seq[buf->type]++;
        // if broadcast send SR, BR, SOURCE/SINK;
        esp_fill_random(buf->payload, 10); // sizeof(espnow_data_t));
                                           // uint8_t idx = 0;
#ifdef IS_SOURCE
        // when sameple / bit rate changes send broadcast again and reconfigure i2s

        send_param->len = 6 + sizeof(espnow_data_t);
        // buf->payload[0] = espnow_bd_source;
        buf->payload[1] = (uint8_t)((SAMPLE_RATE >> 24) & 0xff);
        buf->payload[2] = (uint8_t)((SAMPLE_RATE >> 16) & 0xFF);
        buf->payload[3] = (uint8_t)((SAMPLE_RATE >> 8 & 0xFF));
        buf->payload[4] = (uint8_t)(SAMPLE_RATE & 0xFF);
        buf->payload[5] = BIT_DEPTH;
        // UINT16_MAX;
        // set sample rate
        // set bit rate
#else
#ifdef IS_SINK
        buf->src_sink = espnow_bd_sink;
        // get sink/source id
        send_param->len = sizeof(espnow_data_t) + 1;
        buf->payload[0] = espnow_broadcast_ok;
        // SAMPLE_RATE =
#endif //  esp_fill_random(buf->payload, send_param->len - sizeof(espnow_data_t));//sizeof(espnow_data_t));
#endif

        // idx++;
    }

    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);
#ifdef IS_SOURCE
    // xSemaphoreGive(xsh);
#endif
}

// maybe send ack from sink when data received.
// if no ack is received then remove from peer until the peer is back online?

// ESP-NOW send callback
void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{

    espnow_event_t event;
    espnow_event_send_cb_t *send_cb = &event.info.send_cb;

    if (mac_addr == NULL)
    {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }
    event.id = ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    if (xQueueSend(espnow_queue, &event, ESPNOW_MAXDELAY) != pdTRUE)
    {
        ESP_LOGW(TAG, "Send send queue fail");
    }

    // if (status == ESP_NOW_SEND_SUCCESS) {
    //     ESP_LOGI(TAG, "Message sent successfully");
    // } else {
    //     ESP_LOGE(TAG, "Message send failed");
    // }
}

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    espnow_event_t evt;
    espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
    uint8_t *mac_addr = recv_info->src_addr;

    if (mac_addr == NULL || data == NULL || len <= 0)
    {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    evt.id = ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL)
    {
        ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    if (xQueueSend(espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE)
    {
        ESP_LOGW(TAG, "Send receive queue fail");
        free(recv_cb->data);
    }
}

/* Parse received ESPNOW data. */
#ifdef IS_SOURCE
int espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *seq, int *src_sink)
#endif
#ifdef IS_SINK
    IRAM_ATTR int espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *seq, int *src_sink)
#endif
// IRAM_ATTR int espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *seq, int *src_sink)
{

    // when sameple / bit rate changes receive broadcast again and reconfigure i2s

    espnow_data_t *buf = (espnow_data_t *)data;
    uint16_t crc, crc_cal = 0;
    static uint32_t idx = 0;
    static uint16_t seq_prev = 0;
    if (data_len < sizeof(espnow_data_t))
    {
        ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
        return -1;
    }

    if (buf->type == ESPNOW_DATA_BROADCAST)
    {
        *state = 1;
        // do thigs with data
        // uint8_t idx = 0;
        if (buf->payload[0] == espnow_broadcast_ok)
        {
            // add to peer
            // stop sending broadcast messages
        }
#ifdef IS_SOURCE

#else
#ifdef IS_SINK
        if (buf->src_sink == espnow_bd_source)
        {
            paired = true;
            SAMPLE_RATE = ((uint32_t)buf->payload[1] << 24) | ((uint32_t)buf->payload[2] << 16) | ((uint32_t)buf->payload[3] << 8) | (uint32_t)buf->payload[4];
            BIT_DEPTH = buf->payload[5];
            ESP_LOGI(TAG, "SampleRate %lu: BitRate:%u", SAMPLE_RATE, BIT_DEPTH);
            // i2s_port_conf.clk_cfg.sample_rate_hz = SAMPLE_RATE;
            // i2s_port_conf.slot_cfg.data_bit_width = BIT_DEPTH;
            // if (i2s0_tx_handler == NULL)
            // {
            //     i2s_new_channel(&i2s_ch_conf, &i2s0_tx_handler, NULL);
            //     i2s_channel_init_std_mode(i2s0_tx_handler, &i2s_port_conf);
            // }
            // else
            // {
            //     i2s_channel_disable(i2s0_tx_handler);
            //     i2s_channel_reconfig_std_clock(i2s0_tx_handler, &i2s_port_conf.clk_cfg);
            //     i2s_channel_reconfig_std_slot(i2s0_tx_handler, &i2s_port_conf.slot_cfg);
            // }

// do this after receiving broadcast
#ifdef IS_SINK

            // i2s_channel_enable(i2s0_tx_handler);
            //  xTaskCreate(i2s_tx_task, "i2s_tx_task", 4096, NULL, 2, NULL);//2048
#ifdef EN_SPDIF
            spdif_init(SAMPLE_RATE, BIT_DEPTH);
            //
            if(xTaskGetHandle("i2s_spdif_task") == NULL){
                xTaskCreate(i2s_spdif_task, "i2s_spdif_task", 8192, NULL, 13, NULL); // 2048
            }
#endif
#endif
        }
        else
        {
            paired = false;
        }
#endif
#endif
    }
    else if (buf->type == ESPNOW_DATA_UNICAST)
    {
#ifdef IS_SOURCE

#else
#ifdef IS_SINK

        uint8_t *ptr = (uint8_t *)ESPNOW_I2S_BUF;
        // ESP_LOGI(TAG, "Processing unicast payload size %u num: %d", data_len, buf->seq_num);
        if (seq_prev == 0 && idx == 0 && buf->seq_num > 0)
        {
            seq_prev = buf->seq_num - 1;
        }
        else if ((seq_prev != buf->seq_num - 1) && !((buf->seq_num == 0 && seq_prev > 0) || (buf->seq_num == 0 && seq_prev == 0))) // && !(seq_prev != UINT16_MAX && *seq != 0)){// && !(seq_prev == UINT16_MAX && *seq == 0))
        {
            ESP_LOGI(TAG, "Packet Lost prev: %d current %d", seq_prev, buf->seq_num);
            while (seq_prev < buf->seq_num - 1)
            {
                if (idx + data_len - sizeof(espnow_data_t) > sizeof(ESPNOW_I2S_BUF))
                {
                    memset(ptr, 0, idx + data_len - sizeof(espnow_data_t) - sizeof(ESPNOW_I2S_BUF));
                    // memcpy(ptr + idx, buf->payload, idx + data_len - sizeof(espnow_data_t) - sizeof(ESPNOW_I2S_BUF));
                    idx += idx + data_len - sizeof(espnow_data_t) - sizeof(ESPNOW_I2S_BUF);
                }
                else
                {
                    memset(ptr, 0, data_len - sizeof(espnow_data_t));
                    idx += data_len - sizeof(espnow_data_t);
                }

                // memset(ptr, 0, data_len - sizeof(espnow_data_t));
                seq_prev++;
                // idx += data_len - sizeof(espnow_data_t);
                if (idx == sizeof(ESPNOW_I2S_BUF))
                {
                    //memcpy(I2S_TX_BUF, ESPNOW_I2S_BUF, sizeof(ESPNOW_I2S_BUF));
                    xRingbufferSend(rbuf_handle, ESPNOW_I2S_BUF, idx, pdMS_TO_TICKS(1000));
                    idx = 0;
                }
            }
        }
        // copy payload data into i2s tx buffer

        if (idx + data_len - sizeof(espnow_data_t) >= sizeof(ESPNOW_I2S_BUF))
        {
            // ESP_LOGI(TAG, "IF 1 Processing unicast payload size %lu / %u", idx, sizeof(ESPNOW_I2S_BUF));
            memcpy(ptr + idx, buf->payload, sizeof(ESPNOW_I2S_BUF) - idx);
            idx += sizeof(ESPNOW_I2S_BUF) - idx;
        }
        else
        {
            // ESP_LOGI(TAG, "IF 2 Processing unicast payload size %lu / %u", idx, sizeof(ESPNOW_I2S_BUF));
            memcpy(ptr + idx, buf->payload, data_len - sizeof(espnow_data_t));
            idx += data_len - sizeof(espnow_data_t);
        }
        seq_prev = buf->seq_num;
        if ((buf->src_sink & espnow_bd_EOF) == espnow_bd_EOF)
        {
          
            //xSemaphoreTake(xsh, portMAX_DELAY);// == pdTRUE;
           // memcpy(I2S_TX_BUF, ESPNOW_I2S_BUF, idx);

           //dump the accumulated data into ring buffer for the i2s peripheral to process when ready
           
            while(xRingbufferGetCurFreeSize(rbuf_handle) < idx){
                ESP_LOGI(TAG, "I2S too slow, buffer overrun. Waiting for more free space in buffer");
                xSemaphoreTake(xsh, portMAX_DELAY);

            }
            if(xRingbufferSend(rbuf_handle, ESPNOW_I2S_BUF, idx, pdMS_TO_TICKS(1000)) != pdTRUE){
                ESP_LOGI(TAG, "Ring Buffer Send Error");
            }
            //if(xSemaphoreTake(i2s_sync_h, 0) == pdTRUE){
                xSemaphoreGive(i2s_sync_h);
            //}else{
              //  xSemaphoreGive(i2s_sync_h);
            //}
            
            //ESP_LOGI(TAG, "End of frame %lu / %u", idx, sizeof(ESPNOW_I2S_BUF));
            //xSemaphoreGive(i2s_sync_h);
            bSent = idx;
            idx = 0;
            // }
                    seq_prev = 0;
        }
        else if (idx == sizeof(ESPNOW_I2S_BUF) && (buf->src_sink & espnow_bd_EOF) != espnow_bd_EOF)
        {
            ESP_LOGE(TAG, "EOF missing Copying to I2S Buffer from ESPNOW buffer");
            xSemaphoreTake(xsh, portMAX_DELAY);
            //memcpy(I2S_TX_BUF, ESPNOW_I2S_BUF, sizeof(ESPNOW_I2S_BUF));
            xSemaphoreGive(i2s_sync_h);

            idx = 0;
        }


        // ESP_LOGI(TAG, "%lu / %d", idx, sizeof(ESPNOW_I2S_BUF) );
        // xSemaphoreGive(xsh);
#endif
#endif
    }

    *seq = buf->seq_num;
    *src_sink = buf->src_sink;
    crc = buf->crc;
    buf->crc = 0;
    crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);

    if (crc_cal == crc)
    {
        return buf->type;
    }

    return -1;
}

IRAM_ATTR static void espnow_task(void *pvParameter)
{
    // static uint8_t source_paired = 0xff;
    espnow_event_t evt;
    uint8_t recv_state = 0;
    uint16_t recv_seq = 0;
    int recv_src_sink = 0;
    bool is_broadcast = false;
    int ret;

    espnow_send_param_t *send_param = (espnow_send_param_t *)pvParameter;
    if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK)
    {
        ESP_LOGE(TAG, "Send error init");
        espnow_deinit(send_param);
        vTaskDelete(NULL);
    }
    else
    {
        ESP_LOGI(TAG, "Sent Initial Broadcast");
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    while (xQueueReceive(espnow_queue, &evt, portMAX_DELAY) == pdTRUE)
    {
        switch (evt.id)
        {
        case ESPNOW_SEND_CB:
        {
            espnow_event_send_cb_t *send_cb = &evt.info.send_cb;
            is_broadcast = IS_BROADCAST_ADDR(send_cb->mac_addr);
            if (is_broadcast)
            {
                if (send_param->broadcast == false)
                {
                    break;
                }
                ESP_LOGD(TAG, "Sent Broadcast_x");
                /* Delay a while before sending the next data. */
                if (send_param->delay > 0)
                {
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                }
            }

            // ESP_LOGI(TAG, "send data to "MACSTR"", MAC2STR(send_cb->mac_addr));

            // maybe repeat here
            memcpy(send_param->dest_mac, send_cb->mac_addr, ESP_NOW_ETH_ALEN);
            espnow_prepare_data(send_param);

            if (send_param->unicast == true)
            {
                //esp_err_t err;
                if ((/* err =  */esp_now_send(NULL, send_param->buffer, send_param->len)) != ESP_OK)
                // if (esp_now_send(NULL, send_param->buffer, send_param->len) != ESP_OK)
                {
                    //ESP_LOGE(TAG, "Send error Multicast err:%d", err);
                    espnow_deinit(send_param);
                    vTaskDelete(NULL);
                }
            }
            else
            {
                if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK)
                // if (esp_now_send(NULL, send_param->buffer, send_param->len) != ESP_OK)
                {
                    ESP_LOGE(TAG, "Send error Unicast"); 
                    espnow_deinit(send_param);
                    vTaskDelete(NULL);
                }
            }
            /* Send the next data after the previous data is sent. */
            // if esp_now_get_peer_num(esp_now_peer_num_t *num) > 1 then multicast
            // if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK)
            // // if (esp_now_send(NULL, send_param->buffer, send_param->len) != ESP_OK)
            // {
            //     ESP_LOGE(TAG, "Send error");
            //     espnow_deinit(send_param);
            //     vTaskDelete(NULL);
            // }
            break;
        }
        case ESPNOW_RECV_CB:
        {
            // maybe separate threads for rec and send
            espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

            ret = espnow_data_parse(recv_cb->data, recv_cb->data_len, &recv_state, &recv_seq, &recv_src_sink);
            free(recv_cb->data);
            if (ret == ESPNOW_DATA_BROADCAST)
            {
                ESP_LOGI(TAG, "Receive %dth broadcast data from: " MACSTR ", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);
                // is sink find source and add single peer
                // if source wait for broadcast and add, then send broadcast back with required info
                /* If MAC address does not exist in peer list, add it to peer list. */
                // #ifdef IS_SOURCE
                if (esp_now_is_peer_exist(recv_cb->mac_addr) == false)
                {
                    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
                    ESP_LOGI(TAG, "Added Peer " MACSTR ", seq: %d size: %u", MAC2STR(recv_cb->mac_addr), recv_seq, recv_cb->data_len);
                    if (peer == NULL)
                    {
                        ESP_LOGE(TAG, "Malloc peer information fail");
                        espnow_deinit(send_param);
                        vTaskDelete(NULL);
                    }
                    memset(peer, 0, sizeof(esp_now_peer_info_t));
                    peer->channel = CONFIG_ESPNOW_CHANNEL;
                    peer->ifidx = ESP_IF_WIFI_STA; // ESPNOW_WIFI_IF;
                    peer->encrypt = true;
                    memcpy(peer->lmk, ESPNOW_LMK, ESP_NOW_KEY_LEN);
                    memcpy(peer->peer_addr, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                    ESP_ERROR_CHECK(esp_now_add_peer(peer));

                    free(peer);

                    // peer = malloc(sizeof(esp_now_peer_info_t));
                    // memset(peer, 0, sizeof(esp_now_peer_info_t));
                    // peer->channel = CONFIG_ESPNOW_CHANNEL;
                    // peer->ifidx = ESP_IF_WIFI_STA; // ESPNOW_WIFI_IF;
                    // peer->encrypt = true;
                    // memcpy(peer->lmk, ESPNOW_LMK, ESP_NOW_KEY_LEN);
                    // memcpy(peer->peer_addr, multicasttest_mac, ESP_NOW_ETH_ALEN);
                    // ESP_ERROR_CHECK(esp_now_add_peer(peer));

                    // free(peer);
                }
                else
                {
                    // resend broadcast info if broadcast sequence number is 0
                    if (recv_seq == 0)
                    {
                        ESP_LOGI(TAG, "Resent broacast to Peer " MACSTR ", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr));
                        memcpy(send_param->dest_mac, broadcast_mac, ESP_NOW_ETH_ALEN);
                        // send_param->broadcast = true;
                        // send_param->unicast = false;
                        // send_param->state = 0;
                        espnow_prepare_data(send_param);

                        // send_param->state = 0;
                        if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK)
                        {
                            ESP_LOGE(TAG, "Send error resend broadcast");
                            espnow_prepare_data(send_param);
                            vTaskDelete(NULL);
                        }
                        else
                        {
                            send_param->broadcast = false;
                            send_param->unicast = true;
                            send_param->state = 1;
                        }
                    }
                }

                /* Indicates that the device has received broadcast ESPNOW data. */
                if (send_param->state == 0)
                {
                    send_param->state = 1;
                }

                /* If receive broadcast ESPNOW data which indicates that the other device has received
                 * broadcast ESPNOW data and the local src_sink number is bigger than that in the received
                 * broadcast ESPNOW data, stop sending broadcast ESPNOW data and start sending unicast
                 * ESPNOW data.
                 */
                if (recv_state == 1)
                {
                    /* The device which has the bigger src_sink number sends ESPNOW data, the other one
                     * receives ESPNOW data.
                     */
                    // make source start unicast
                    if (send_param->unicast == false && (recv_src_sink == espnow_bd_sink))
                    {
#ifdef IS_SOURCE
                        // send_param->src_sink = recv_src_sink++;

                        ESP_LOGI(TAG, "Start sending unicast data to peer " MACSTR "", MAC2STR(recv_cb->mac_addr));
                        // ESP_LOGI(TAG, "send data to "MACSTR"", MAC2STR(recv_cb->mac_addr));

                        /* Start sending unicast ESPNOW data. */
                        memcpy(send_param->dest_mac, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                        espnow_prepare_data(send_param);
                        if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK)
                        {
                            ESP_LOGE(TAG, "Send error unicast change");
                            espnow_prepare_data(send_param);
                            vTaskDelete(NULL);
                        }
                        else
                        {
                            send_param->broadcast = false;
                            send_param->unicast = true;
                        }
#endif
                    }
                    else if ((recv_src_sink & espnow_bd_source) == espnow_bd_source)
                    {
#ifdef IS_SINK
                        // maybe send data back indicating receive starting
                        ESP_LOGI(TAG, "Start receiving from " MACSTR "", MAC2STR(recv_cb->mac_addr));
                        send_param->broadcast = false;
                        // unicast for ack to stay paired?
                        send_param->unicast = true;
#endif
                    }
                }
            }

            else if (ret == ESPNOW_DATA_UNICAST)
            {
                // send ack every second to stay connected
                //  ESP_LOGI(TAG, "Receive %dth unicast data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);
                //  ESP_LOGI(TAG, "Receive %dth unicast data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);
                // data processing for
                // send_param->broadcast = false;
            }

            else
            {
                ESP_LOGI(TAG, "Receive error data from: " MACSTR "", MAC2STR(recv_cb->mac_addr));
            }
            break;
        }
        default:
            ESP_LOGE(TAG, "Callback type error: %d", evt.id);
            break;
        }
    }
}

static esp_err_t espnow_init()
{   //54 has too high of an corruption/loss rate
    //esp_wifi_internal_set_fix_rate(ESP_IF_WIFI_STA, true, WIFI_PHY_RATE_MCS2_SGI);//WIFI_PHY_RATE_MCS7_SGI
    esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_ABOVE);
    //esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    ESP_ERROR_CHECK(esp_wifi_config_espnow_rate(ESP_IF_WIFI_STA, WIFI_PHY_RATE_MCS2_SGI)); // WIFI_PHY_RATE_54M));//WIFI_PHY_RATE_MCS7_SGI);// WIFI_PHY_RATE_54M));
    // esp_wifi_set_protocol(ESP_IF_WIFI_STA,WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR)
    // esp_wifi_set_protocol()
    //  Initialize ESP-NOW

    espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_t));

    if (espnow_queue == NULL)
    {
        ESP_LOGE(TAG, "Create mutex fail");
        return ESP_FAIL;
    }
    ESP_ERROR_CHECK(esp_now_init());

    // esp_now_set_peer_rate_config();
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL)
    {
        ESP_LOGE(TAG, "Malloc peer information fail");
        // vSemaphoreDelete(s_example_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }

    esp_now_set_pmk((uint8_t *)ESPNOW_PMK);

    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESP_IF_WIFI_STA;
    peer->encrypt = false;
    memcpy(peer->peer_addr, broadcast_mac, ESP_NOW_ETH_ALEN);
    esp_now_add_peer(peer);
    free(peer);







    espnow_send_param_t *send_param;
    send_param = malloc(sizeof(espnow_send_param_t));
    if (send_param == NULL)
    {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        vSemaphoreDelete(espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(send_param, 0, sizeof(espnow_send_param_t));
    send_param->unicast = false;
    send_param->broadcast = true;
    send_param->state = 0;
    // #ifdef IS_SOURCE
    //     send_param->src_sink = esp_random();
    // #else
#ifdef IS_SINK
    send_param->src_sink = espnow_bd_sink;
#endif
#ifdef IS_SOURCE
    send_param->src_sink = espnow_bd_source;
#endif
    // #endif
    send_param->count = ESPNOW_SEND_COUNT;

    send_param->delay = ESPNOW_SEND_DELAY;

    send_param->len = ESPNOW_MAX_PAYLOAD_SIZE;
    send_param->buffer = malloc(ESPNOW_MAX_PAYLOAD_SIZE);
    if (send_param->buffer == NULL)
    {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(send_param);
        vSemaphoreDelete(espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memcpy(send_param->dest_mac, broadcast_mac, ESP_NOW_ETH_ALEN);
    espnow_prepare_data(send_param);
    // ESP_NOW_MAX_DATA_LEN;
    xTaskCreate(espnow_task, "espnow_task", 4096, send_param, 16, NULL); // 2048
    return ESP_OK;
}

void app_main()
{

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize I2S
    // i2s_port_conf
    // xsh = xSemaphoreCreateMutex(); // xSemaphoreCreateBinary();
    xsh = xSemaphoreCreateBinary(); // xSemaphoreCreateBinary();
    i2s_sync_h = xSemaphoreCreateBinary();
    // xSemaphoreCreateBinary;
    // Initialize WiFi
    esp_netif_init();
    esp_event_loop_create_default();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();


    // esp_wifi_set_channel(ESPNOW);
    espnow_init();

    // set all i2s config here

    //
    //separate unicast tx thread
    PAYLOAD_SIZE = ESPNOW_MAX_PAYLOAD_SIZE - sizeof(espnow_data_t);
#ifdef IS_SINK
    rbuf_handle = xRingbufferCreate(I2S_BUF_SIZ * sizeof(uint32_t) * 2 , RINGBUF_TYPE_BYTEBUF);// RINGBUF_TYPE_BYTEBUF);// _NOSPLIT);
    if(rbuf_handle == NULL){
        ESP_LOGE(TAG, "ERR Failed to create buffer");   
    }else{
        ESP_LOGI(TAG, "Created Ring Buffer");   
    }
#endif
#ifdef IS_SOURCE
    rbuf_handle = xRingbufferCreate(I2S_BUF_SIZ * sizeof(uint32_t) /* * 2 */, RINGBUF_TYPE_BYTEBUF);// RINGBUF_TYPE_BYTEBUF);// _NOSPLIT);
    if(rbuf_handle == NULL){
        ESP_LOGE(TAG, "ERR Failed to create buffer");   
    }else{
        ESP_LOGI(TAG, "Created Ring Buffer remaining heap %lu", esp_get_free_heap_size() );   
    }
#endif



#ifdef IS_SOURCE
    i2s_port_conf.clk_cfg.sample_rate_hz = SAMPLE_RATE;
    i2s_port_conf.slot_cfg.data_bit_width = BIT_DEPTH;
    i2s_ch_conf.dma_desc_num = DMA_DESC_NUM;
    i2s_ch_conf.dma_frame_num = DMA_FRAME_NUM;
    i2s_new_channel(&i2s_ch_conf, NULL, &i2s0_rx_handler);
    i2s_channel_init_std_mode(i2s0_rx_handler, &i2s_port_conf);
    i2s_channel_enable(i2s0_rx_handler);
    xTaskCreate(i2s_rec_task, "i2s_rec_task", 2048, NULL, 16, NULL); // 2048
#ifdef EN_SPDIF
    spdif_init(96000);
    xTaskCreate(i2s_spdif_task, "i2s_spdif_task", 4096, NULL, 2, NULL); // 2048
#endif
#endif
}
