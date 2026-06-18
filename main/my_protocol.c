#include "my_protocol.h"
#include <string.h>

static void put_u16_be(uint8_t *buf, uint16_t value)
{
    buf[0] = (value >> 8) &0xFF;
    buf[1] = value & 0xFF;
}

static void put_32_be(uint8_t *buf, uint32_t value)
{
    buf[0] = (value >> 24) & 0xFF;
    buf[1] = (value >> 16) & 0xFF;
    buf[2] = (value >> 8)  & 0xFF;
    buf[3] = value & 0xFF;
}

static uint8_t protocol_check_sum(const uint8_t *buff, size_t len)
{
    uint8_t check_sum =0;

    for(size_t i=0 ; i < len ; i++)
    {
        check_sum ^= buff[i];
    }

    return check_sum;
}


static bool protocol_build_frame(uint16_t sensor_id, uint8_t msg_type,
                                    uint8_t *data, uint16_t data_len, 
                                    uint32_t timestamp, uint8_t *out_buf, 
                                    size_t out_buf_size, size_t *out_len)
{
    if(out_buf == NULL || out_len == NULL)
    {
        return false;
    }

    if(data_len > 0 && data == NULL)
    {
        return false;
    }

    size_t frame_len = 14U + data_len;

    if(frame_len > out_buf_size || frame_len > TX_FRAME_MAX_SIZE)
    {
        return false;
    }

    size_t idx = 0;

    put_u16_be(&out_buf[idx], HEADER_BYTE);
    idx += 2;

    put_u16_be(&out_buf[idx], sensor_id);
    idx += 2;

    out_buf[idx++] = msg_type;

    put_u16_be(&out_buf[idx], data_len);
    idx += 2;

    if(data_len > 0)
    {
        memcpy(&out_buf[idx], data, data_len);
        idx += data_len;
    }

    put_32_be(&out_buf[idx], timestamp);
    idx +=4;

    uint8_t check_sum = protocol_check_sum(&out_buf[2], 2 + 1 + 2 + data_len + 4);
    out_buf[idx++] = check_sum;

    put_u16_be(&out_buf[idx], TAIL_BYTE);
    idx+=2;

    *out_len = idx;
    
    return true;
}

bool my_protocol_build_sensor_data_digital(uint16_t sensor_id, bool detected, 
                                            uint32_t timestamp, uint8_t *out_buf, 
                                            size_t out_buf_size, size_t *out_len)
{
    uint8_t data[6];

    data[0] = DATA_DIGITAL;

    uint32_t value = detected ? 1:0 ;
    put_32_be(&data[1], value);

    data[5] = DATA_QUALITY_GOOD;

    return protocol_build_frame(sensor_id, MSG_SENSOR_DATA, 
                                data, sizeof(data),
                                timestamp, out_buf,
                                out_buf_size, out_len);

}

bool my_protocol_build_status(uint16_t sensor_id, uint8_t status,
                                uint32_t timestamp, uint8_t *out_buf,
                                size_t out_buf_size, size_t *out_len)
{
    uint8_t data[1];

    data[0] = status;

    return protocol_build_frame(sensor_id, MSG_SENSOR_STATUS,
                                data, sizeof(data),
                                timestamp, out_buf,
                                out_buf_size, out_len);
}

bool my_protocol_build_heartbeat(uint16_t sensor_id, uint32_t uptime_sec, 
                                    uint8_t battery_level,  
                                    uint32_t timestamp, uint8_t *out_buf,
                                    size_t out_buf_size, size_t *out_len)
{
    uint8_t data[5];

    put_32_be(&data[0], uptime_sec);

    data[4] = battery_level;

    return protocol_build_frame(sensor_id, MSG_HEARTBEAT, 
                                    data, sizeof(data),
                                    timestamp, out_buf,
                                    out_buf_size, out_len);
}