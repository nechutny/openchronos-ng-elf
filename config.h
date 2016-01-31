// !!!! DO NOT EDIT !!!, use: make config

#ifndef _CONFIG_H_
#define _CONFIG_H_

#define CONFIG_DEBUG
#define USE_LCD_CHARGE_PUMP
#define USE_WATCHDOG
#define CONFIG_DEBUG_EASY_RESET
#define CONFIG_RTC_IRQ
#define CONFIG_RTC_DST
#define CONFIG_RTC_DST_ZONE 4
#define CONFIG_TIMER_4S_IRQ
#define CONFIG_TIMER_20HZ_IRQ
#ifndef CONFIG_BUTTONS_LONG_PRESS_TIME
#define CONFIG_BUTTONS_LONG_PRESS_TIME 6
#endif // CONFIG_BUTTONS_LONG_PRESS_TIME
#ifndef CONFIG_BUTTONS_SHORT_PRESS_TIME
#define CONFIG_BUTTONS_SHORT_PRESS_TIME 1
#endif // CONFIG_BUTTONS_SHORT_PRESS_TIME
#define CONFIG_BATTERY_MONITOR
#define CONFIG_BATTERY_DISABLE_FILTER
#ifndef CONFIG_TEMPERATURE_OFFSET
#define CONFIG_TEMPERATURE_OFFSET -260
#endif // CONFIG_TEMPERATURE_OFFSET
#define CONFIG_TEMPERATURE_METRIC
// CONFIG_MOD_OTP is not set
#define CONFIG_MOD_OTP_KEY ""
#define CONFIG_MOD_OTP_OFFSET 0
// CONFIG_MOD_TIDE is not set
#define CONFIG_MOD_CLOCK
#define CONFIG_MOD_CLOCK_BLINKCOL
// CONFIG_MOD_CLOCK_AMPM is not set
// CONFIG_MOD_CLOCK_MONTH_FIRST is not set
#define CONFIG_MOD_RESET
#define CONFIG_MOD_ALARM
#define CONFIG_MOD_STOPWATCH
// CONFIG_MOD_ACCELEROMETER is not set
#define CONFIG_MOD_TEMPERATURE
#define CONFIG_MOD_BATTERY
#define CONFIG_MOD_BATTERY_SHOW_VOLTAGE
#define CONFIG_MOD_MUSIC

#endif // _CONFIG_H_
