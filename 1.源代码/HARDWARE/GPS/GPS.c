#include "GPS.h"
#include "usart2.h"

// 全局变量初始化（默认清零，避免野值）
GPS_DataTypedef gps_data = {0};
float gps_lat_decimal = 0.0f;
float gps_lon_decimal = 0.0f;

/******************************************************************
 * 函数名：GPS_Init
 * 功能：GPS模块初始化（清空缓冲区、重置所有状态标志）
 * 优化点：初始化更彻底，状态统一置false
 ******************************************************************/
void GPS_Init(void) {
    memset(&gps_data, 0, sizeof(GPS_DataTypedef));
    gps_data.is_data_ready = false;
    gps_data.is_parsed = false;
    gps_data.is_valid = false;
    gps_lat_decimal = 0.0f;
    gps_lon_decimal = 0.0f;
}

/******************************************************************
 * 函数名：GPS_ErrorLog
 * 功能：GPS错误日志打印（内部使用，不阻塞程序）
 * 优化点：日志更详细，支持自定义错误信息
 ******************************************************************/
static void GPS_ErrorLog(const char* err_info) {
    printf("[GPS Error] %s\r\n", err_info);
    GPS_ClearBuffer(); // 清空缓冲区，避免重复解析错误数据
}

/******************************************************************
 * 函数名：NMEA2Decimal
 * 功能：NMEA度分格式（ddmm.mmmm/dddmm.mmmm）→ 十进制度
 * 参数：nmea_str-度分字符串；is_longitude-是否为经度（true=经度，false=纬度）
 * 优化点：提取为独立函数，减少冗余；增强容错（空指针、无小数点）
 ******************************************************************/
static float NMEA2Decimal(const char* nmea_str, bool is_longitude) {
    if (nmea_str == NULL || strlen(nmea_str) < 5) { // 容错：字符串为空或过短
        GPS_ErrorLog("NMEA string invalid");
        return 0.0f;
    }

    char* dot_ptr = strchr(nmea_str, '.'); // 查找小数点
    if (dot_ptr == NULL) { // 容错：无小数点（非法格式）
        GPS_ErrorLog("No decimal point in NMEA string");
        return 0.0f;
    }

    uint8_t deg_digits = is_longitude ? 3 : 2; // 经度前3位是度（ddd），纬度前2位是度（dd）
    if ((dot_ptr - nmea_str) < deg_digits + 1) { // 容错：度/分位数不足
        GPS_ErrorLog("NMEA degree/minute invalid");
        return 0.0f;
    }

    // 提取度（整数部分）
    char deg_str[4] = {0};
    strncpy(deg_str, nmea_str, deg_digits);
    for (uint8_t i = 0; i < deg_digits; i++) { // 容错：非数字字符
        if (!isdigit(deg_str[i])) {
            GPS_ErrorLog("Degree is not digit");
            return 0.0f;
        }
    }
    float deg = atof(deg_str);

    // 提取分（整数+小数部分）
    char min_str[8] = {0};
    strncpy(min_str, nmea_str + deg_digits, dot_ptr - (nmea_str + deg_digits) + 4); // 保留4位小数
    float min = atof(min_str);

    return deg + (min / 60.0f); // 度 + 分/60 = 十进制度
}

/******************************************************************
 * 函数名：GPS_ParseNMEA
 * 功能：解析NMEA协议核心语句（$GPGGA），提取关键信息
 * 优化点：
 * 1. 缓存字符串长度，减少strlen重复调用（提升效率）
 * 2. 增强容错（空缓冲区、非GGA语句、字段缺失）
 * 3. 标准化字段提取，避免内存越界
 ******************************************************************/
void GPS_ParseNMEA(void) {
    // 容错1：数据未准备好或缓冲区为空
    if (!gps_data.is_data_ready || strlen(gps_data.buf) < 30) {
        GPS_ClearBuffer();
        return;
    }

    // 容错2：仅解析$GPGGA语句（NMEA核心定位语句）
    if (strstr(gps_data.buf, "$GPGGA") == NULL) {
        GPS_ErrorLog("Not GPGGA sentence, skip");
        GPS_ClearBuffer();
        return;
    }

    gps_data.is_data_ready = false; // 重置数据准备标志
    printf("[GPS Raw] %s\r\n", gps_data.buf);

    char* sub_str = NULL;
    char* sub_str_next = NULL;
    uint8_t field_idx = 0; // 字段索引（0=$GPGGA, 1=UTC时间, 2=定位有效性, ...）
    char valid_buf[2] = {0};

    // 按逗号分割字段（解析核心6个字段：UTC时间→东西半球）
    for (field_idx = 0; field_idx <= 6; field_idx++) {
    if (field_idx == 0) {
        sub_str = strstr(gps_data.buf, ",");
        if (sub_str == NULL) {
            GPS_ErrorLog("No separator found");
            return;
        }
    } else {
        sub_str++;
        sub_str_next = strstr(sub_str, ",");
        if (sub_str_next == NULL) {
            GPS_ErrorLog("Field missing");
            return;
        }

        uint8_t field_len = MIN(sub_str_next - sub_str, 10);
        switch (field_idx) {
            case 1: // UTC时间（hhmmss.sss）
                strncpy(gps_data.utc_time, sub_str, MIN(field_len, UTC_TIME_MAX_LEN - 1));
                gps_data.utc_time[UTC_TIME_MAX_LEN - 1] = '\0';
                break;
            case 2: // 纬度（ddmm.mmmm）
                strncpy(gps_data.latitude, sub_str, MIN(field_len, LATITUDE_MAX_LEN - 1));
                gps_data.latitude[LATITUDE_MAX_LEN - 1] = '\0';
                break;
            case 3: // 南北半球（N/S）
                strncpy(gps_data.ns_hemisphere, sub_str, MIN(field_len, HEMISPHERE_MAX_LEN - 1));
                gps_data.ns_hemisphere[HEMISPHERE_MAX_LEN - 1] = '\0';
                break;
            case 4: // 经度（dddmm.mmmm）
                strncpy(gps_data.longitude, sub_str, MIN(field_len, LONGITUDE_MAX_LEN - 1));
                gps_data.longitude[LONGITUDE_MAX_LEN - 1] = '\0';
                break;
            case 5: // 东西半球（E/W）
                strncpy(gps_data.ew_hemisphere, sub_str, MIN(field_len, HEMISPHERE_MAX_LEN - 1));
                gps_data.ew_hemisphere[HEMISPHERE_MAX_LEN - 1] = '\0';
                break;
            case 6: // 定位有效性（1=有效，0=无效）→ 关键修正！
                strncpy(valid_buf, sub_str, MIN(field_len, 1));
                valid_buf[1] = '\0';
                // GPGGA的有效性是数字（1=有定位，0=无定位），不是'A/V'！
                gps_data.is_valid = (valid_buf[0] == '1') ? true : false;
                break;
            default:
                break;
        }
        sub_str = sub_str_next;
        gps_data.is_parsed = true;
    }
}
		}

/******************************************************************
 * 函数名：GPS_DisplayAndSend
 * 功能：1. 经纬度格式转换；2. OLED显示；3. 串口打印+蓝牙发送
 * 优化点：
 * 1. 移除冗余代码，调用NMEA2Decimal函数简化转换逻辑
 * 2. 用snprintf替代sprintf，避免缓冲区溢出
 * 3. 统一显示格式，增强可读性
 ******************************************************************/
void GPS_DisplayAndSend(void) {
    if (!gps_data.is_parsed) {
        return;
    }

    gps_data.is_parsed = false;
    printf("\r\n[GPS Parsed] ----------\r\n");

    // 1. UTC时间转换（可选：若需显示北京时间，增加容错）
    if (strlen(gps_data.utc_time) >= 6) {
        uint8_t hour = ((gps_data.utc_time[0] - '0') * 10 + (gps_data.utc_time[1] - '0')) + 8;
        hour = (hour >= 24) ? (hour - 24) : hour;
        uint8_t min = (gps_data.utc_time[2] - '0') * 10 + (gps_data.utc_time[3] - '0');
        uint8_t sec = (gps_data.utc_time[4] - '0') * 10 + (gps_data.utc_time[5] - '0');
        printf("UTC Time: %s → Beijing Time: %02d:%02d:%02d\r\n", gps_data.utc_time, hour, min, sec);
    } else {
        GPS_ErrorLog("UTC time invalid");
    }

    // 2. 有效定位时：转换经纬度+显示+发送
    if (gps_data.is_valid) {
        // 度分→十进制度转换（调用独立函数）
        gps_lat_decimal = NMEA2Decimal(gps_data.latitude, false); // 纬度：false
        gps_lon_decimal = NMEA2Decimal(gps_data.longitude, true);  // 经度：true

        // 容错：经纬度超出合理范围（纬度：-90~90，经度：-180~180）
        if ((gps_lat_decimal < -90.0f || gps_lat_decimal > 90.0f) || 
            (gps_lon_decimal < -180.0f || gps_lon_decimal > 180.0f)) {
            GPS_ErrorLog("Lat/Lon out of range");
            gps_data.is_valid = false;
            return;
        }

        // 格式化经纬度字符串（保留5位小数，避免缓冲区溢出）
        char lat_str[12] = {0};
        char lon_str[12] = {0};
        snprintf(lat_str, sizeof(lat_str), "%.5f", gps_lat_decimal);
        snprintf(lon_str, sizeof(lon_str), "%.5f", gps_lon_decimal);

        // OLED显示（位置固定，格式统一）
        OLED_ShowString(45, 16, (uint8_t*)lon_str, 16, 1); // 经度
        OLED_ShowChar(45 + strlen(lon_str)*8, 16, gps_data.ew_hemisphere[0], 16, 1);
        OLED_ShowString(45, 32, (uint8_t*)lat_str, 16, 1); // 纬度
        OLED_ShowChar(45 + strlen(lat_str)*8, 32, gps_data.ns_hemisphere[0], 16, 1);

        // 串口打印
        printf("Latitude: %s %s (Decimal: %.5f)\r\n", gps_data.latitude, gps_data.ns_hemisphere, gps_lat_decimal);
        printf("Longitude: %s %s (Decimal: %.5f)\r\n", gps_data.longitude, gps_data.ew_hemisphere, gps_lon_decimal);
        printf("----------------------------------------\r\n");

        // 蓝牙发送（USART2）
        GPS_SendLatLonViaUSART2();
    } else {
        // 无效定位：OLED显示提示，串口打印日志
        GPS_ErrorLog("Position invalid (check GPS signal)");
        OLED_ShowString(45, 16, (uint8_t*)"No Signal ", 16, 1);
        OLED_ShowString(45, 32, (uint8_t*)"No Signal ", 16, 1);
    }
}

/******************************************************************
 * 函数名：GPS_SendLatLonViaUSART2
 * 功能：通过USART2（蓝牙）发送经纬度数据（格式标准化）
 * 优化点：格式统一，便于上位机解析；用snprintf避免溢出
 ******************************************************************/
void GPS_SendLatLonViaUSART2(void) {
    char bluetooth_buf[64] = {0};
    // 格式：GPS,LAT:30.12345N,LON:120.12345E\r\n（便于上位机解析）
    snprintf(bluetooth_buf, sizeof(bluetooth_buf), 
             "纬度:%.5f%s,经度:%.5f%s\r\n",
             gps_lat_decimal, gps_data.ns_hemisphere,
             gps_lon_decimal, gps_data.ew_hemisphere);
    USART2_SendString((const char*)bluetooth_buf);
}

/******************************************************************
 * 函数名：GPS_ClearBuffer
 * 功能：彻底清空GPS缓冲区和状态标志（避免残留数据影响）
 * 优化点：清空更彻底，状态标志统一置false
 ******************************************************************/
void GPS_ClearBuffer(void) {
    memset(gps_data.buf, 0, sizeof(gps_data.buf));
    memset(gps_data.utc_time, 0, sizeof(gps_data.utc_time));
    memset(gps_data.latitude, 0, sizeof(gps_data.latitude));
    memset(gps_data.longitude, 0, sizeof(gps_data.longitude));
    memset(gps_data.ns_hemisphere, 0, sizeof(gps_data.ns_hemisphere));
    memset(gps_data.ew_hemisphere, 0, sizeof(gps_data.ew_hemisphere));
    gps_data.is_data_ready = false;
    gps_data.is_parsed = false;
    gps_data.is_valid = false;
}



