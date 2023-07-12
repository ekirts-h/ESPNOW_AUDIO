# ESPNOW_AUDIO
Wireless audio using ESPNOW

Mostly working @48k/32b 
ESP32 WROOM 32D devkit ->ESPNOW->ESP32-S3 DevkitC SPDIF output with modified SPDIF.c from esp_a2dp_sink_spdif 
max ~0.22A for esp32(SOURCE) and ~0.12A for ESP32-S3(SINK)

Slight delay between source and sink side playback. 

ToDO cleanup code, change wifi menuconfig settings for rx/tx.
try get 96KHz to work 

MCS7/54MB is too susceptible to interference and packet loss, useless for this function. Version with external antenna may work better?
Very weak to microwave.
