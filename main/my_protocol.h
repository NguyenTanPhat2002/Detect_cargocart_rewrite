/* Follow:
1. After connect success, sensor send "SENSOR_STATUS(ONLINE)"
2. If disconnect, sensor auto reconnect after 5 second
3. if sensor don't receive ACK about 2 second, sensor resend message, max 3 times
4. Don't send 2 message too close, minimun interval of 10 ms
5. Big Endian
6. Checksum: Sensor ID (XOR) Timestamp

*/


#ifndef MY_PROTOCOL_H
#define MY_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define HEADER_BYTE 0xAA55
#define TAIL_BYTE   0x55AA

#define MSG_SENSOR_DATA 0x01
#define MSG_HEARTBEAT   0x02
#define MSG_SENSOR_STATUS   0x03
#define MSG_SENSOR_ALARM    0x04
#define MSG_ACK    0x80
#define MSG_CONFIG  0x81
#define MSG_RESET   0x82
#define MSG_ERROR   0xFF

#define DATA_DIGITAL    0x01
#define DATA_ANALOG     0x02
#define DATA_FLOAT      0x03

#define DATA_QUALITY_GOOD    0x00
#define DATA_QUALITY_UNCERTAIN  0x01
#define DATA_QUALITY_BAD    0x02

#define SENSOR_STATUS_OFF   0x00
#define SENSOR_STATUS_ON    0x01
#define SENSOR_STATUS_ERR   0x02
#define SENSOR_STATUS_CALI  0x03
#define SENSOR_STATUS_MAIN  0x04

#define ACK_OK  0x00
#define ACK_CHECKSUM_FAIL 0x01
#define ACK_FORMAT_FAIL 0x02

#define CONFIG_SIGNAL_DELAY_MS  0x01
#define CONFIG_WIFI_SSID        0x02
#define CONFIG_WIFI_PASS        0x03
#define CONFIG_SERVER_IP        0x04
#define CONFIG_SERVER_PORT      0x05

#define HEARTBEAT_PERIOD_MS 3000
#define BATTERY_WIRED   0xFF

#define TX_FRAME_MAX_SIZE 256

bool my_protocol_build_sensor_data_digital(uint16_t sensor_id, bool detected, 
                                            uint32_t timestamp, uint8_t *out_buf, 
                                            size_t out_buf_size, size_t *out_len);

bool my_protocol_build_status(uint16_t sensor_id, uint8_t status,
                                uint32_t timestamp, uint8_t *out_buf,
                                size_t out_buf_size, size_t *out_len);

bool my_protocol_build_heartbeat(uint16_t sensor_id, uint32_t uptime_sec, 
                                uint8_t battery_level,
                                uint32_t timestamp, uint8_t *out_buf,
                                size_t out_buf_size, size_t *out_len);
#endif