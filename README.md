# ESPNOW_AUDIO
Wireless audio using ESPNOW between two ESP32

Working ~@48k/32b~ 96KHz/32bit  
ESP32 WROOM 32D devkit ->ESPNOW->ESP32-S3 DevkitC SPDIF output with modified SPDIF.c from esp_a2dp_sink_spdif   
max ~0.22A for esp32(SOURCE) and ~0.12A for ESP32-S3(SINK)  

AMPDU off  

Very slight delay between source and sink side playback.   
Noticeable when source and sink playing at the same time.  

ToDO:
change to broadcast based so there is no wait for an ack, make different data packet formatting for different purposes

cleanup code, change wifi menuconfig settings for rx/tx.  
~maybe try MCS7 again~  
~maybe dedicated threads for send | receive~  
fix problems pairing at startup, needs reset several times to pair and start sending data  
watchdog / timer / try detect silence/disconnection and go sleep   
try reduce power consumption  
move parts of code to different files  


Done:  
Change to Ring Buffer  
Working with Source side I2S_receive at 48KHz/32b 64 frame_num and 2 desc_num  
96KHz/32Bit  Frame=256 Desc_num = 2
Dedicated thread for tx/rx  

