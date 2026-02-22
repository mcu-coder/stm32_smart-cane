#include "stm32f10x.h"
#include "led.h"        // LED驱动头文件
#include "beep.h"       // 蜂鸣器驱动头文件
#include "usart.h"      // 串口1（调试/打印）驱动头文件
#include "delay.h"      // 延时函数头文件
#include "oled.h"       // OLED显示屏驱动头文件
#include "key.h"        // 按键驱动头文件
#include "Modules.h"    // 模块相关结构体定义头文件
#include "adcx.h"       // ADC采集（光照等）头文件
#include "flash.h"      // FLASH存储（阈值/时间）头文件
#include "usart2.h"     // 串口2（蓝牙通信）驱动头文件
#include "usart3.h"     // 串口3（语音模块）驱动头文件
#include "MPU6050.h"    // MPU6050（姿态/跌倒检测）驱动头文件
#include "ultrasonic.h" // 超声波（距离检测）驱动头文件
#include "MyRTC.h"      // RTC实时时钟驱动头文件
#include "TIM2.h"       // 定时器2（定时中断）驱动头文件
#include "timer.h"      // 定时器相关头文件
#include "GPS.h"        // GPS模块（定位/经纬度）驱动头文件



#define KEY_Long1	11

#define KEY_1	1
#define KEY_2	2
#define KEY_3	3
#define KEY_4	4

#define FLASH_START_ADDR	0x0801f000	//写入的起始地址
#define RESET_FLAG_UNDONE	0x00			// 未执行过复位
#define RESET_FLAG_DONE		0xAA			// 已执行过复位

 SensorModules sensorData;								//声明传感器数据结构体变量
 SensorThresholdValue Sensorthreshold;		//声明传感器阈值结构体变量
 DriveModules driveData;									//声明驱动器状态结构体变量

uint8_t mode = 1;	//系统模式  1自动  2手动  3设置
u8 dakai;//串口3使用的传递变量
u8 Flag_dakai;//串口3接收标志位
uint8_t is_secondary_menu = 0;  // 0一级菜单，1二级菜单
uint8_t secondary_pos = 1;      // 二级菜单光标位置（1-3对应时/分/秒）
uint8_t secondary_type = 0;   // 二级菜单类型：0=RTC时间，1=定时开启，2=定时关闭



extern float gps_lat_decimal;
extern float gps_lon_decimal;
//系统静态变量
//static uint8_t count_a = 1;  //自动模式按键数（预留，当前未使用）
static uint8_t count_m = 1;  // 手动模式按键计数（1=灯光控制，2=蜂鸣器控制）
static uint8_t count_s = 1;	 // 设置模式按键计数（1=时间设置，2=距离阈值，3=光照阈值）
static uint8_t auto_page = 1;  // 自动模式页面切换变量（1=第一页：传感器数据，2=第二页：GPS数据）- 解决未定义错误
//static uint8_t last_mode = 0;      // 记录上一次的模式（预留，当前使用局部静态变量）
//static uint8_t last_count_s = 0;   // 记录设置模式内上一次的页面（预留，当前使用局部静态变量）



enum 
{
	AUTO_MODE = 1,
	MANUAL_MODE,
	SETTINGS_MODE
	
}MODE_PAGES;
  
/**
  * @brief  显示菜单1的固定内容
  * @param  无
  * @retval 显示页面标题、传感器数据项名称（时间、光照、距离、跌倒检测）
  */
void OLED_autoPage1(void)		//自动模式菜单第一页
{

	//显示”光照强度：  Lux
	OLED_ShowChinese(0,16,0,16,1); //光
	OLED_ShowChinese(16,16,1,16,1);//照
	OLED_ShowChinese(32,16,2,16,1);//强
	OLED_ShowChinese(48,16,3,16,1);//度
	OLED_ShowChar(64,16,':',16,1);
	 
	
	OLED_Refresh();
	
}
void OLED_autoPage2(void)   //自动模式菜单第二页
{
	OLED_ShowString(45, 0, "G P S", 16, 1);
	
//  OLED_ShowChinese(0,16,13,16,1);	
//	OLED_ShowChinese(16,16,14,16,1);	
//  OLED_ShowChar(32,16,':',16,1);
	
	 
}



/**
  * @brief  自动模式第一页传感器数据显示+蓝牙发送
  * @param  无
  * @retval 无
  * @note   1. 采集传感器数据（光照、距离、跌倒检测）
  *         2. 解析GPS数据（若有新数据），格式化经纬度
  *         3. 合并传感器数据+GPS数据，通过蓝牙发送
  *         4. 显示时间、光照、距离、跌倒检测结果到OLED
  */
void SensorDataDisplay1(void)		
{
    SensorScan();	// 获取传感器数据（光照、距离、MPU6050姿态）
    char all_data[256];  // 传感器数据缓存（足够容纳所有数据）
	  char gps_str[64] = {0};    // GPS数据临时缓存（格式化经纬度）
    
  
   
    // 跌倒检测逻辑（上电后前两次连续跌倒不显示“是”，第三次及以后显示+报警）
    static uint8_t mpu_continuous_count = 0;  // 连续检测到跌倒的次数（静态变量：函数调用间保留值）
    static uint8_t first_two_passed = 0;      // 前两次连续跌倒是否已过（0=未过，1=已过）

    if(sensorData.mpu_test == 1)  // 检测到跌倒（MPU6050触发）
    {
        mpu_continuous_count++;  // 连续次数+1（仅连续检测时累计）
        
        // 核心逻辑：仅前两次连续跌倒→不显示“是”，第三次及以后→显示“是”
        if(first_two_passed == 1)
        {
            driveData.MPU_Flag = 1;  // 跌倒标志置1（用于关联其他动作）
            OLED_ShowChinese(80,48,6,16,1);	// 显示“是”
            USART2_SendString("跌倒请注意！\r\n");  // 蓝牙发送跌倒报警
        }
        else
        {
            driveData.MPU_Flag = 0;  // 前两次跌倒，标志置0
            OLED_ShowChinese(80,48,7,16,1);	// 显示“否”（避免误提示）
        }
        
        // 连续检测到2次后，标记“前两次已过”（后续连续检测直接触发）
        if(mpu_continuous_count >= 2)
        {
            first_two_passed = 1;
        }
    }
    
}

/**
  * @brief  自动模式第二页GPS数据显示+蓝牙发送
  * @param  无
  * @retval 无
  * @note   1. 格式化光照、距离数据，通过蓝牙发送
  *         2. 解析GPS数据，显示经纬度到OLED+蓝牙发送
  */
void SensorDataDisplay2(void)		
{
	 char all_data[128];  // 传感器数据缓存
	// 格式化光照、距离数据
	sprintf(all_data, "光照: %d\r\n距离：%d CM\r\n",
    sensorData.lux,     // 光照强度
    sensorData.distance // 距离（超声波）
);
 
    // 通过串口2（蓝牙）发送传感器数据
    USART2_SendString(all_data);  
    GPS_ParseNMEA();    // 解析GPS原始数据（NMEA协议→经纬度）
    GPS_DisplayAndSend();// 显示经纬度到OLED+蓝牙发送
}

/**
  * @brief  显示手动模式设置界面1
  * @param  无
  * @retval 无
  */
void OLED_manualPage1(void)
{
	//显示“灯光”
	OLED_ShowChinese(16,0,15,16,1);	
	OLED_ShowChinese(32,0,16,16,1);	
	OLED_ShowChinese(48,0,17,16,1);	
	OLED_ShowChar(64,0,':',16,1);
 
	
}

/**
  * @brief  显示手动模式设置参数界面1
  * @param  无
  * @retval 无
  */
void ManualSettingsDisplay1(void)
{
	if(driveData.LED_Flag ==1)
	{
		OLED_ShowChinese(96,0,23,16,1); 	//开
	}
 
}

/**
  * @brief  显示系统阈值设置界面1
  * @param  无
  * @retval 无
  */
void OLED_settingsPage1(void)
{

	
	//显示“温度阈值”
	OLED_ShowChinese(16,0,27,16,1);	
	OLED_ShowChinese(32,0,28,16,1);	
	OLED_ShowChinese(48,0,13,16,1);	
	OLED_ShowChinese(64,0,14,16,1);
  OLED_ShowChar(80,0,':',16,1);
	OLED_ShowString(88,0,"xxx",16,1);

	 
	
	
}

void OLED_settingsPage2(void)//显示系统阈值设置界面2
{


}
void OLED_settingsPage3(void)//显示系统阈值设置界面3
{
	OLED_ShowChinese(30,0,27,16,1);	
	OLED_ShowChinese(46,0,28,16,1);	
	 
}

void SettingsThresholdDisplay1(void)//实际阈值1
{

	//显示光照阈值数值
	OLED_ShowNum(90, 32, Sensorthreshold.luxValue , 3,16,1);
	//显示光照阈值数值
	OLED_ShowNum(90, 16, Sensorthreshold.distanceValue , 3,16,1);
}

void SettingsThresholdDisplay2(void)//实际阈值2
{

}

void SettingsThresholdDisplay3(void)//实际阈值3
{
// 显示时:分:秒
    OLED_ShowNum(30,32,MyRTC_Time[3],2,16,1);
    OLED_ShowChar(46,32,':',16,1);
    OLED_ShowNum(62,32,MyRTC_Time[4],2,16,1);
    
}

/**
  * @brief  自动模式页面切换处理（按键2）
  * @param  无
  * @retval 当前自动模式页面（1=第一页，2=第二页）
  * @note   按下KEY2切换自动模式页面，切换时清屏并绘制对应页面固定内容
  */
uint8_t SetAuto(void)  
{
	  if(KeyNum == KEY_2)  // 检测到KEY2按下
    {
        KeyNum = 0;          // 清除按键标志（避免重复触发）
        auto_page = (auto_page == 1) ? 2 : 1;  // 页面切换（1?2）
        OLED_Clear();        // 清屏（避免页面内容重叠）
        delay_ms(5);         // 清屏后延迟，避免OLED绘制残留
        
        // 切换后绘制对应页面的固定内容
        if(auto_page == 1)
        {
            OLED_autoPage1();  // 第一页：传感器数据页面
        }
        
    }
    return auto_page;  // 返回当前页面
}

/**
  * @brief  手动模式控制项切换处理（按键2）
  * @param  无
  * @retval 当前手动模式控制项（1=灯光，2=蜂鸣器）
  * @note   按下KEY2切换控制项，超过2项则循环到1
  */
uint8_t SetManual(void)  
{
	if(KeyNum == KEY_2)  // 检测到KEY2按下
	{
		KeyNum = 0;  // 清除按键标志
		count_m++;   // 控制项计数+1
		if (count_m > 2)  // 控制项最大为2（灯光、蜂鸣器）
		{
			OLED_Clear();  // 循环时清屏
			count_m = 1;   // 重置计数为1（灯光）
		}
	}
	return count_m;  // 返回当前控制项
}

/**
  * @brief  设置模式控制项切换处理（按键2）
  * @param  无
  * @retval 当前设置模式控制项（1=时间，2=距离阈值，3=光照阈值）
  * @note   按下KEY2切换设置项，超过3项则循环到1
  */
uint8_t SetSelection(void)
{
    if(KeyNum == KEY_2 && is_secondary_menu == 0)  // KEY2按下且当前是一级菜单
    {
        KeyNum = 0;  // 清除按键标志
        count_s++;   // 设置项计数+1
        if (count_s > 3)  // 设置项最大为3（时间、距离、光照）
        {
            count_s = 1;  // 重置计数为1（时间）
        }
    }
    return count_s;  // 返回当前设置项
}

/**
  * @brief  显示手动模式界面的选择符号
  * @param  num 为显示的位置
  * @retval 无
  */
void OLED_manualOption(uint8_t num)
{
	switch(num)
	{
		case 1:	
			OLED_ShowChar(0, 0,'>',16,1);
			OLED_ShowChar(0,16,' ',16,1);
			OLED_ShowChar(0,32,' ',16,1);
			OLED_ShowChar(0,48,' ',16,1);
			break;
	 
	}
}

/**
  * @brief  显示阈值界面的选择符号
  * @param  num 为显示的位置
  * @retval 无
  */
void OLED_settingsOption(uint8_t num)
{
	static uint8_t prev_num = 1; // 记录上一次光标位置（避免重复绘制）

    // 清除上一次光标（仅操作光标位置，不影响数据）
    switch(prev_num)
    {
        case 1: OLED_ShowChar(0, 0, ' ', 16, 1); break; // 系统时间行
        case 2: OLED_ShowChar(0, 16, ' ', 16, 1); break; // 温度阈值行
        case 3: OLED_ShowChar(0, 32, ' ', 16, 1); break; // 湿度阈值行
        case 4: OLED_ShowChar(0, 48, ' ', 16, 1); break; // 光照阈值行
        default: break;
    }
	switch(num)
	{
		case 1:	
			OLED_ShowChar(0, 0,'>',16,1);
			OLED_ShowChar(0,16,' ',16,1);
			OLED_ShowChar(0,32,' ',16,1);
			OLED_ShowChar(0,48,' ',16,1);
			break;
		case 2:	
			OLED_ShowChar(0, 0,' ',16,1);
			OLED_ShowChar(0,16,'>',16,1);
			OLED_ShowChar(0,32,' ',16,1);
			OLED_ShowChar(0,48,' ',16,1);
			break;
	 

		default: break;
	}
	 prev_num = num;
    OLED_Refresh(); // 仅刷新光标，数据区域无变化
}

/**
  * @brief  自动模式控制函数
  * @param  无
  * @retval 无
  */
void AutoControl(void)//自动控制
{	
    //光照
		 if(sensorData.lux<Sensorthreshold.luxValue)//根据光照数据控制灯
		driveData.LED_Flag =1;
   else
		driveData.LED_Flag =0; 

	 
	 //距离
	  static uint8_t beep_trigger_count = 0; // 静态计数器：记录触发次数（函数调用间保留值）
if (sensorData.distance < Sensorthreshold.distanceValue || sensorData.mpu_test == 1)
{
	
      beep_trigger_count++; // 满足条件，计数+1
        // 仅当计数≥3时，才打开蜂鸣器
        driveData.BEEP_Flag = (beep_trigger_count >= 3) ? 1 : 0;
}
 
		 // 保存参数到FLASH：光照阈值、距离阈值、RTC时间（时/分/秒）
		FLASH_W(FLASH_START_ADDR, Sensorthreshold.luxValue, Sensorthreshold.distanceValue,MyRTC_Time[3], MyRTC_Time[4], MyRTC_Time[5]);
}

/**
  * @brief  手动模式控制函数
  * @param  无
  * @retval 无
  */
void ManualControl(uint8_t num)
{
	switch(num)
	{
		case 1:  
            if(KeyNum == KEY_3)
            {
                driveData.LED_Flag = 1;  
                KeyNum = 0;  
                printf("[按键] KEY3按下，LED_Flag置1\n");  
            }
            if(KeyNum == KEY_4)
            {
                driveData.LED_Flag = 0; 
                KeyNum = 0;  
                printf("[按键] KEY4按下，LED_Flag置0\n"); 
            }
            break;

	 
		

		default: break;
	}
}

/**
  * @brief  控制函数
  * @param  无
  * @retval 无
  */
void Control_Manager(void)
{
    if(driveData.LED_Flag )
    {	
        LED_On(); 
    }
    else 
    {
        LED_Off();
    }
		
		 
}

/**
  * @brief  阈值设置函数
  * @param  无
  * @retval 无
  */
void ThresholdSettings(uint8_t num)
{
	
	switch (num)
	{
		//温度

			//光照
			case 3:
			if (KeyNum == KEY_3)
			{
				KeyNum = 0;
				Sensorthreshold.luxValue += 10;
				if (Sensorthreshold.luxValue > 500)
				{
					Sensorthreshold.luxValue = 80;
				}
			}
		 
        break;	
			//距离
		case 2:
			if (KeyNum == KEY_3)
			{
				KeyNum = 0;
				Sensorthreshold.distanceValue += 1;
				if (Sensorthreshold.distanceValue > 50)
				{
					Sensorthreshold.distanceValue = 1;
				}
			}
			 
		
        default: break;
	}   
}

/**
  * @brief  上电复位检测：仅首次上电（POR）延迟1秒复位，软件复位后跳过
  * @param  无
  * @retval 无
  */
void PowerOnResetCheck(void)
{
    // 读取RCC复位状态寄存器，判断复位类型
    uint32_t reset_flag = RCC_GetFlagStatus(RCC_FLAG_PORRST); // 上电复位标志
    
    if(reset_flag == SET) // 首次上电（检测到上电复位标志）
    {
        delay_ms(1000); // 上电后延迟1秒
        
        // 清除所有复位标志（避免影响后续判断）
        RCC_ClearFlag();
        
        // 执行软件复位
        NVIC_SystemReset();
    }
    
}

//flash读取
void FLASH_ReadThreshold()
{
    Sensorthreshold.luxValue = FLASH_R(FLASH_START_ADDR );       // +4
    Sensorthreshold.distanceValue = FLASH_R(FLASH_START_ADDR + 2);  // +6
		MyRTC_Time[3]= FLASH_R(FLASH_START_ADDR+4);	
    MyRTC_Time[4]= FLASH_R(FLASH_START_ADDR+6);	
    MyRTC_Time[5]= FLASH_R(FLASH_START_ADDR+8);
}
/**
  * @brief  主函数（程序入口）
  * @param  无
  * @retval int：返回值（实际未使用）
  * @note   1. 初始化所有硬件模块
  *         2. 读取FLASH保存的参数（阈值、时间）
  *         3. 主循环：处理按键、模式切换、传感器数据、设备控制、显示刷新
  */
int main(void)
{ 
    SystemInit();// 配置系统时钟为72MHz	
    delay_init(72);  // 延时函数初始化（基于72MHz系统时钟）
    ADCX_Init();     // ADC初始化（用于光照强度采集）
    LED_Init();      // LED初始化（GPIO配置）
    BEEP_Init();     // 蜂鸣器初始化（GPIO配置）
    
    
    // 延时100ms，等待硬件稳定后读取FLASH参数
    delay_ms(100);
    FLASH_ReadThreshold(); // 从FLASH读取阈值、时间参数
	  MyRTC_Init();	    // RTC初始化（实时时钟配置）
	  MyRTC_SetTime();    // RTC时间设置（使用FLASH读取的时间参数）
	  OLED_Clear();// OLED清屏（初始化后清屏，避免乱码）
   
    // 状态管理静态变量（主循环内保留值）
    static uint8_t last_mode = 0;  // 记录上一次系统模式（用于模式切换检测）
    static uint32_t last_sensor_time = 0; // 传感器扫描时间戳（控制扫描频率）
    static uint32_t last_display_time = 0; // 显示刷新时间戳（控制刷新频率）
     
    
    TIM2_Init(72-1, 1000-1);  // 定时器2初始化（定时2ms中断：72MHz时钟，分频72→1MHz，计数1000→2ms）
    printf("Start \n"); // 串口1打印启动信息（调试用）
	
    
    while (1)
    {	
        
        // ==================== 获取当前系统时间 ====================
        uint32_t current_time = delay_get_tick();  // 获取当前系统滴答计数（毫秒级时间戳，用于定时控制）
        
        // ==================== 优化传感器扫描频率 ====================
        if(current_time - last_sensor_time > 100) // 每200ms扫描一次传感器 (100 * 2ms = 200ms)
        {
					
            SensorScan(); 	//获取传感器数据
            last_sensor_time = current_time;// 更新时间戳

        }
        
        // ==================== 立即处理按键 ====================
        uint8_t current_key_num = KeyNum; // 保存当前按键值（避免按键标志被多次处理）
        
        // 模式切换按键立即处理（KEY1=模式切换，KEY_Long1=自动→设置）
        if(current_key_num != 0)
        {
            switch(mode)
            {
                case AUTO_MODE: // 当前是自动模式
                    if(current_key_num == KEY_1) // KEY1=自动→手动
                    {
                        mode = MANUAL_MODE;
                        count_m = 1; // 手动模式默认光标指向灯光
                        driveData.LED_Flag = 0; // 切换时关闭LED
                        driveData.BEEP_Flag = 0; // 切换时关闭蜂鸣器
                        KeyNum = 0; // 清除按键标志
                    }
                    else if(current_key_num == KEY_Long1) // 长按KEY1=自动→设置
                    {
                        mode = SETTINGS_MODE;
                        count_s = 1; // 设置模式默认光标指向时间
                        KeyNum = 0; // 清除按键标志
                    }
                    break;
                    
                
                    
                case SETTINGS_MODE: // 当前是设置模式（按键在模式内部处理）
                    break;
            }
        }
        
        // 模式切换检测：若当前模式与上一次不同，清屏并绘制新模式固定内容
        if(last_mode != mode)
        {
            OLED_Clear(); // 清屏（避免模式间内容重叠）
            last_mode = mode; // 更新上一次模式
            
            // 绘制新模式的固定内容
            switch(mode)
            {
                case AUTO_MODE:
                    OLED_autoPage1(); // 自动模式默认第一页
                    break;
                case MANUAL_MODE:
                    OLED_manualPage1(); // 手动模式页面
                    break;
                case SETTINGS_MODE:
                    OLED_settingsPage1(); // 设置模式第一页
                    break;
            }
            OLED_Refresh(); // 立即刷新显示
        }
        
        // 按当前模式执行对应逻辑
        switch(mode)
        {
            case AUTO_MODE: // 自动模式
            {
                // 获取当前自动模式页面（处理KEY2切换）
                uint8_t curr_auto_page = SetAuto();
                if(curr_auto_page == 1)
                {
                    SensorDataDisplay1();	// 第一页：显示传感器数据+蓝牙发送
                }
                else
                {
                    SensorDataDisplay2();	// 第二页：显示GPS数据+蓝牙发送
                }
                
                AutoControl(); // 自动控制逻辑（LED/蜂鸣器）
                Control_Manager(); // 执行设备控制（LED/蜂鸣器开关）
                break;
            }    
            
            case MANUAL_MODE: // 手动模式
            {
                // 手动模式状态管理静态变量
                static uint8_t manual_page_initialized = 0; // 页面初始化标志
                static uint8_t last_manual_count = 0; // 上一次控制项计数
                static uint8_t last_LED_Flag = 0; // 上一次LED状态
                static uint8_t last_BEEP_Flag = 0; // 上一次蜂鸣器状态
                static uint8_t force_refresh = 0;  // 强制刷新标志
               
                
                // 获取当前控制项（处理KEY2切换）
                uint8_t current_manual_count = SetManual();
                
                // 检测设备状态是否变化（变化则需要刷新显示）
                uint8_t need_refresh = 0;
                if(driveData.LED_Flag != last_LED_Flag || driveData.BEEP_Flag != last_BEEP_Flag)
                {
                    need_refresh = 1;
                    last_LED_Flag = driveData.LED_Flag;
                    last_BEEP_Flag = driveData.BEEP_Flag;
                }
                
                // 页面未初始化、控制项变化、设备状态变化或强制刷新时，重新绘制页面
                if(!manual_page_initialized || current_manual_count != last_manual_count || need_refresh || force_refresh)
                {
                    OLED_manualPage1();          // 绘制固定内容（灯光、蜂鸣器名称）
                    OLED_manualOption(current_manual_count); // 绘制光标
                    ManualSettingsDisplay1();    // 绘制设备状态（开/关）
                    manual_page_initialized = 1; // 标记页面已初始化
                    last_manual_count = current_manual_count; // 更新控制项计数
                    force_refresh = 0;  // 清除强制刷新标志
                    OLED_Refresh(); // 刷新显示
                }
                
                
                
                Control_Manager(); // 执行设备控制（LED/蜂鸣器开关）
                break;
            }
            
            case SETTINGS_MODE: // 设置模式
            {
                // 设置模式状态管理静态变量
                static uint8_t is_threshold_page_inited = 0;    // 一级菜单初始化标志
                static uint8_t is_back_from_secondary = 0;      // 从二级菜单返回标志
                uint8_t curr_count_s = SetSelection(); // 获取当前设置项（处理KEY2切换）
                uint8_t refresh_needed = 0;                     // 刷新标志
                static uint8_t is_secondary_inited = 0;         // 二级菜单初始化标志

                // 二级菜单逻辑（时间设置）
                if (is_secondary_menu == 1)
                {
                    // 进入二级菜单时初始化（仅一次）
                    if (is_secondary_inited == 0)
                    {
                        OLED_Clear();          // 清屏
                        delay_ms(8);           // 延迟避免残留
                        OLED_settingsPage3();  // 绘制时间设置标题
                        is_secondary_inited = 1; // 标记二级菜单已初始化
                        is_back_from_secondary = 0;
                        refresh_needed = 1;    // 需要刷新显示
                    }

                    
                    // KEY4：数值-1（时/分/秒递减，循环范围）
                    else if (KeyNum == KEY_4)
                    {
                        KeyNum = 0;
                        switch(secondary_pos)
                        {
                            case 1: MyRTC_Time[3] = (MyRTC_Time[3]+23)%24; break; // 时（0-23）
                            case 2: MyRTC_Time[4] = (MyRTC_Time[4]+59)%60; break; // 分（0-59）
                            case 3: MyRTC_Time[5] = (MyRTC_Time[5]+59)%60; break; // 秒（0-59）
                        }
                        MyRTC_SetTime(); // 更新RTC时间
                        refresh_needed = 1;
                    }

                    

                    // 需要刷新时，更新时间显示和光标
                    if (refresh_needed)
                    {
                        SettingsThresholdDisplay3(); // 显示当前时间
                        // 绘制当前光标位置（'v'表示选中）
                        switch(secondary_pos)
                        {
                            case 1: OLED_ShowChar(30, 16, 'v', 16, 1); break; // 时
                            case 2: OLED_ShowChar(62, 16, 'v', 16, 1); break; // 分
                            case 3: OLED_ShowChar(94, 16, 'v', 16, 1); break; // 秒
                        }
                        OLED_Refresh(); // 刷新显示
                        refresh_needed = 0;
                    }
                }

                // 一级菜单逻辑（阈值设置）
                else
                {
                    is_secondary_inited = 0; // 重置二级菜单初始化标志

                    // 一级菜单未初始化时（首次进入设置模式）
                    if (is_threshold_page_inited == 0)
                    {
                        OLED_Clear();          // 清屏
                        OLED_settingsPage1();  // 绘制固定内容（阈值项名称）
                        SettingsThresholdDisplay1(); // 绘制当前阈值
                        OLED_settingsOption(curr_count_s); // 绘制光标
                        OLED_Refresh();        // 刷新显示
                        is_threshold_page_inited = 1; // 标记一级菜单已初始化
                        is_back_from_secondary = 0;
                    }

                    // 从二级菜单返回一级菜单时
                    if (is_back_from_secondary == 1)
                    {
                        OLED_settingsPage1();  // 绘制固定内容
                        SettingsThresholdDisplay1(); // 绘制阈值
                        OLED_settingsOption(curr_count_s); // 绘制光标
                        OLED_Refresh(); // 刷新显示
                        is_back_from_secondary = 0;
                    }

                    

                    // 一级菜单光标移动（设置项变化时）
                    static uint8_t last_curr_count_s = 0;
                    if (curr_count_s != last_curr_count_s)
                    {
                        OLED_settingsOption(curr_count_s); // 更新光标
                        OLED_Refresh(); // 刷新显示
                        last_curr_count_s = curr_count_s; // 更新上一次设置项
                    }
                }
                break;
            }
        }
        
        // 显示刷新频率控制：每50ms刷新一次（避免频繁刷新占用资源）
        if(current_time - last_display_time > 25)
        {
            OLED_Refresh(); // 刷新OLED显示
            last_display_time = current_time; // 更新时间戳
        }
    }
}

 

