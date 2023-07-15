# ESPNOW_AUDIO
Wireless audio using ESPNOW between two ESP32

Working @48k/32b 
ESP32 WROOM 32D devkit ->ESPNOW->ESP32-S3 DevkitC SPDIF output with modified SPDIF.c from esp_a2dp_sink_spdif 
max ~0.22A for esp32(SOURCE) and ~0.12A for ESP32-S3(SINK)

AMPDU off

Very slight delay between source and sink side playback. 
Noticeable when source and sink playing at the same time.

ToDO:
cleanup code, change wifi menuconfig settings for rx/tx.
try get 96KHz to work 
maybe try MCS7 again
maybe dedicated threads for send | receive
watchdog / timer / try detect silence/disconnection and go sleep 
try reduce power consumption
move parts of code to different files


MCS7/54MB is too susceptible to interference and packet loss, useless for this function. Version with external antenna may work better?
Very weak to microwave.


Done:
Change to Ring Buffer
Working with Source side I2S_receive at 48KHz/32b 64 frame_num and 2 desc_num  
Use of semaphores to avoid glitches in audio
