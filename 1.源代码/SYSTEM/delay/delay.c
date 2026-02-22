#include "delay.h"
#include "misc.h"
#include "stm32f10x.h"

static u8  fac_us = 0;       // us延时倍乘数
static u16 fac_ms = 0;       // ms延时倍乘数
static volatile uint32_t system_tick = 0; // 系统滴答计数器（毫秒级）

/**
 * @brief  初始化延迟函数和系统滴答计时器
 * @param  SYSCLK: 系统时钟频率（单位：MHz，如72）
 * @note   同时支持阻塞式延时（delay_ms/delay_us）和系统时间计数（delay_get_tick）
 */
void delay_init(u8 SYSCLK)
{
    // 1. 配置SysTick时钟源为HCLK/8（与原有阻塞延时兼容）
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
    
    // 2. 计算延时倍数（用于阻塞式延时）
    fac_us = SYSCLK / 8;       // 72MHz时，fac_us=9（72/8=9）
    fac_ms = (u16)fac_us * 1000; // 72MHz时，fac_ms=9000
    
    // 3. 配置SysTick周期性中断（每1ms触发一次，用于更新system_tick）
    // 加载值 = 1ms内的SysTick时钟周期数 = (HCLK/8) * 1ms
    // HCLK=72MHz时，HCLK/8=9MHz，1ms=9000个周期
    SysTick->LOAD = 9000 - 1;  // 72MHz下，每1ms触发一次中断
    SysTick->VAL = 0;          // 清空计数器
    
    // 4. 开启SysTick中断，并启动计数器
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk; // 使能中断
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;  // 启动计数器
    
    // 5. 初始化系统滴答计数器
    system_tick = 0;
}

/**
 * @brief  SysTick中断服务程序（每1ms触发一次）
 * @note   必须在启动文件中注册，否则无法触发
 */
void SysTick_Handler(void)
{
    delay_inc_tick(); // 每1ms递增一次system_tick
}

/**
 * @brief  递增系统滴答计数器（由中断服务程序调用）
 */
void delay_inc_tick(void)
{
    system_tick++;
}

/**
 * @brief  获取当前系统时间（单位：ms）
 * @retval 系统启动后的毫秒数
 */
uint32_t delay_get_tick(void)
{
    return system_tick;
}

/**
 * @brief  阻塞式毫秒延时
 * @param  nms: 延时毫秒数（最大1864ms@72MHz）
 */
void delay_ms(u16 nms)
{
    u32 temp;
    // 保存当前SysTick配置（避免与周期性中断冲突）
    u32 ctrl_backup = SysTick->CTRL;
    
    // 关闭SysTick中断（防止延时过程中触发中断干扰）
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
    
    // 加载延时计数
    SysTick->LOAD = (u32)nms * fac_ms;
    SysTick->VAL = 0x00;       // 清空计数器
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; // 开始计数
    
    // 等待计数完成
    do {
        temp = SysTick->CTRL;
    } while (temp & 0x01 && !(temp & (1 << 16)));
    
    // 恢复SysTick配置（重新开启中断和计数器）
    SysTick->CTRL = ctrl_backup;
    SysTick->VAL = 0x00;       // 清空计数器
}

/**
 * @brief  阻塞式微秒延时
 * @param  nus: 延时微秒数
 */
void delay_us(u32 nus)
{
    u32 temp;
    u32 ctrl_backup = SysTick->CTRL; // 保存当前配置
    
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk; // 关闭中断
    
    SysTick->LOAD = nus * fac_us;    // 加载计数
    SysTick->VAL = 0x00;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
    
    do {
        temp = SysTick->CTRL;
    } while (temp & 0x01 && !(temp & (1 << 16)));
    
    SysTick->CTRL = ctrl_backup;     // 恢复配置
    SysTick->VAL = 0x00;
}

