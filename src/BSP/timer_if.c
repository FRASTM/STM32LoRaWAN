/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    timer_if.c
  * @author  MCD Application Team
  * @brief   Configure RTC Alarm (B), Tick and Calendar manager
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                       opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include "timer_if.h"
#include "main.h" /*for STM32CubeMX generated RTC_N_PREDIV_S and RTC_N_PREDIV_A*/
#include "rtc.h"
// #include "stm32_lpm.h"
// #include "utilities_def.h"
#include "stm32wlxx_ll_rtc.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/

extern RTC_HandleTypeDef RtcHandle;

/* HAL MSP function used for RTC_Init */
void HAL_RTC_MspInit(RTC_HandleTypeDef* rtcHandle)
{

  if(rtcHandle->Instance==RTC)
  {
    if (HAL_RTCEx_SetSSRU_IT(rtcHandle) != HAL_OK)
    {
        Error_Handler();
    }

    /* RTC interrupt Init */
    HAL_NVIC_SetPriority(TAMP_STAMP_LSECSS_SSRU_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TAMP_STAMP_LSECSS_SSRU_IRQn);

    HAL_NVIC_SetPriority(RTC_Alarm_IRQn, RTC_IRQ_PRIO, RTC_IRQ_SUBPRIO);
    HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);
  }
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef* rtcHandle)
{

  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspDeInit 0 */

  /* USER CODE END RTC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_RTC_DISABLE();
    __HAL_RCC_RTCAPB_CLK_DISABLE();

    /* RTC interrupt Deinit */
    HAL_NVIC_DisableIRQ(TAMP_STAMP_LSECSS_SSRU_IRQn);
    HAL_NVIC_DisableIRQ(RTC_Alarm_IRQn);
  /* USER CODE BEGIN RTC_MspDeInit 1 */

  /* USER CODE END RTC_MspDeInit 1 */
  }
}

/**
  * @brief Timer driver callbacks handler
  */
const UTIL_TIMER_Driver_s UTIL_TimerDriver =
{
  TIMER_IF_Init,
  NULL,

  TIMER_IF_StartTimer,
  TIMER_IF_StopTimer,

  TIMER_IF_SetTimerContext,
  TIMER_IF_GetTimerContext,

  TIMER_IF_GetTimerElapsedTime,
  TIMER_IF_GetTimerValue,
  TIMER_IF_GetMinimumTimeout,

  TIMER_IF_Convert_ms2Tick,
  TIMER_IF_Convert_Tick2ms,
};

/**
  * @brief SysTime driver callbacks handler
  */
const UTIL_SYSTIM_Driver_s UTIL_SYSTIMDriver =
{
  TIMER_IF_BkUp_Write_Seconds,
  TIMER_IF_BkUp_Read_Seconds,
  TIMER_IF_BkUp_Write_SubSeconds,
  TIMER_IF_BkUp_Read_SubSeconds,
  TIMER_IF_GetTime,
};

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/**
  * @brief Minimum timeout delay of Alarm in ticks
  */
#define MIN_ALARM_DELAY    3

/**
  * @brief Backup seconds register
  */
#define RTC_BKP_SECONDS    RTC_BKP_DR0

/**
  * @brief Backup subseconds register
  */
#define RTC_BKP_SUBSECONDS RTC_BKP_DR1

/**
  * @brief Backup msbticks register
  */
#define RTC_BKP_MSBTICKS   RTC_BKP_DR2

/* #define RTIF_DEBUG */

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
#ifdef RTIF_DEBUG
#include "sys_app.h" /*for app_log*/
/**
  * @brief Post the RTC log string format to the circular queue for printing in using the polling mode
  */
#define TIMER_IF_DBG_PRINTF(...) do{ {UTIL_ADV_TRACE_COND_FSend(VLEVEL_ALWAYS, T_REG_OFF, TS_OFF, __VA_ARGS__);} }while(0);
#else
/**
  * @brief not used
  */
#define TIMER_IF_DBG_PRINTF(...)
#endif /* RTIF_DEBUG */

/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
  * @brief Indicates if the RTC is already Initialized or not
  */
static bool RTC_Initialized = false;

/**
  * @brief RtcTimerContext
  */
static uint32_t RtcTimerContext = 0;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief Get rtc timer Value in rtc tick
  * @return val the rtc timer value (upcounting)
  */
static inline uint32_t GetTimerTicks(void);

/**
  * @brief define if function is leap or note
  * @param year
  * @return true if year is leap else false
  */
static inline bool isLeapYear(uint8_t year2k);

/**
  * @brief Writes MSBticks to backup register
  * Absolute RTC time in tick is (MSBticks)<<32 + (32bits binary counter)
  * @note MSBticks incremented every time the 32bits RTC timer wraps around (~44days)
  * @param[in] MSBticks
  */
static void TIMER_IF_BkUp_Write_MSBticks(uint32_t MSBticks);

/**
  * @brief Reads MSBticks from backup register
  * Absolute RTC time in tick is (MSBticks)<<32 + (32bits binary counter)
  * @note MSBticks incremented every time the 32bits RTC timer wraps around (~44days)
  * @retval MSBticks
  */
static uint32_t TIMER_IF_BkUp_Read_MSBticks(void);

/* USER CODE BEGIN PFP */

/* Function to attach to the RTC IRQ as a callback */
void UTIL_TIMER_IRQ_MAP_PROCESS(void *data)
{
    UNUSED(data);

    UTIL_TIMER_IRQ_Handler();
}

/* USER CODE END PFP */

/* Exported functions ---------------------------------------------------------*/
UTIL_TIMER_Status_t TIMER_IF_Init(void)
{
  UTIL_TIMER_Status_t ret = UTIL_TIMER_OK;
  /* USER CODE BEGIN TIMER_IF_Init */

  /* USER CODE END TIMER_IF_Init */
  if (RTC_Initialized == false)
  {
    /* RTC is already Initialized by the LoRaWan::begin */
    RtcHandle.IsEnabled.RtcFeatures = UINT32_MAX;

    /** Enable the Alarm B just after the HAL_RTC_Init */
    RTC_StartAlarm(RTC_ALARM_B, 0, 0, 0, 0, 0, RTC_HOURFORMAT12_PM, ALL_MSK);

    /*Stop Timer */
    TIMER_IF_StopTimer();

    /*overload RTC feature enable*/
    RtcHandle.IsEnabled.RtcFeatures = UINT32_MAX;

    /*Initialize MSB ticks*/
    TIMER_IF_BkUp_Write_MSBticks(0);

    TIMER_IF_SetTimerContext();

    /* Register a task to associate to UTIL_TIMER_Irq() interrupt */
    UTIL_TIMER_IRQ_MAP_INIT();

    RTC_Initialized = true;
  }

  /* USER CODE BEGIN TIMER_IF_Init_Last */

  /* USER CODE END TIMER_IF_Init_Last */
  return ret;
}

/* Timeout is expressed in ms (not null and greater than guard time) */
UTIL_TIMER_Status_t TIMER_IF_StartTimer(uint32_t timeout)
{
  UTIL_TIMER_Status_t ret = UTIL_TIMER_OK;
  /* USER CODE BEGIN TIMER_IF_StartTimer */

  /* USER CODE END TIMER_IF_StartTimer */
  if (timeout > TIMEOUT_GUARD_TIME) {
    /*Stop timer if one is already started*/
    RTC_StopAlarm(RTC_ALARM_B);

    TIMER_IF_DBG_PRINTF("Start timer for %d ms\n\r",  timeout);

    /* Convert the the timeout value to a BCD calendar value */
    uint16_t subSecondsToAdd = timeout % 1000;

    timeout = timeout / 1000;
    uint8_t daysToAdd = timeout / 86400;
    uint8_t hoursToAdd = (timeout - daysToAdd * 86400) / 3600;
    uint8_t minutesToAdd = (timeout - daysToAdd * 86400 - hoursToAdd * 3600) / 60;
    uint8_t secondsToAdd = (timeout - daysToAdd * 86400 - hoursToAdd * 3600 - minutesToAdd * 60);
    hourAM_PM_t period; /* AM or 24h format */
    uint8_t hrCurrent, minCurrent, secCurrent;
    uint32_t subSecondsCurrent;
    uint8_t weekDay, currentDay, currentMonth, currentYear;
    /* Retrieve the current calendar*/
    RTC_GetDate(&currentYear, &currentMonth, &currentDay, &weekDay);
    RTC_GetTime(&hrCurrent, &minCurrent, &secCurrent, &subSecondsCurrent, &period);

    /* Define the calendar value to set from current date + timeout */
    uint32_t ss = subSecondsCurrent + subSecondsToAdd;
    if (ss >= 1000) {
      ss -= 1000;
      secondsToAdd++;
    }

    if (secondsToAdd >= 60) {
      secondsToAdd = 0;
      minutesToAdd++;
    }
    uint8_t s = secCurrent + secondsToAdd;
    if (s >= 60) {
      s -= 60;
      minutesToAdd++;
    }

    if (minutesToAdd >= 60) {
      minutesToAdd -= 60;
      hoursToAdd++;
    }
    uint8_t m = minCurrent + minutesToAdd;
    if (m >= 60) {
      m -= 60;
      hoursToAdd++;
    }

    if (hoursToAdd >= 24) {
      hoursToAdd -= 24;
      daysToAdd++;
    }

    /* format is HOUR_FORMAT_24 as init by the Lorawan */
    uint8_t h = hrCurrent + hoursToAdd;
    if (RTC_GetFormat() == HOUR_FORMAT_12) {
      if (h >= 24) {
        h -= 24;
        daysToAdd++;
    } else if (h >= 12) {
        if (period == HOUR_AM) {
          period = HOUR_PM;
        } else {
          period = HOUR_AM;
          daysToAdd++;
        }

        if (h > 12) {
          h -= 12;
        }
      }
    } else if (h >= 24) {
      h -= 24;
      daysToAdd++;
    }

    // numbers of days in each month (february is calculated based on leap year)
    static uint8_t daysInMonths[] = {31, 0, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    uint8_t endDay;
    if (currentMonth == 2) {
      endDay = isLeapYear(currentYear) ? 29 : 28;
    } else {
      endDay = daysInMonths[currentMonth - 1];
    }

    uint8_t d = currentDay + daysToAdd;
    if (d > endDay) {
      d -= endDay;
    }
    // month-year overflow isn't handled because its not supported by RTC's alarm

    /* Program ALARM B on subsecond, mask is ALL for calendar */
    RTC_StartAlarm(RTC_ALARM_B, d, h, m, s, ss, period, ALL_MSK);

  /* USER CODE BEGIN TIMER_IF_StartTimer_Last */

  /* USER CODE END TIMER_IF_StartTimer_Last */
  } else {
    ret = UTIL_TIMER_INVALID_PARAM;
  }
  return ret;
}

UTIL_TIMER_Status_t TIMER_IF_StopTimer(void)
{
  UTIL_TIMER_Status_t ret = UTIL_TIMER_OK;
  /* USER CODE BEGIN TIMER_IF_StopTimer */

  /* USER CODE END TIMER_IF_StopTimer */

  /* Disable the Alarm B interrupt */
  RTC_StopAlarm(RTC_ALARM_B);

  /*overload RTC feature enable*/
  RtcHandle.IsEnabled.RtcFeatures = UINT32_MAX;
  /* USER CODE BEGIN TIMER_IF_StopTimer_Last */

  /* USER CODE END TIMER_IF_StopTimer_Last */
  return ret;
}

uint32_t TIMER_IF_SetTimerContext(void)
{
  /*store time context*/
  RtcTimerContext = GetTimerTicks();

  /* USER CODE BEGIN TIMER_IF_SetTimerContext */

  /* USER CODE END TIMER_IF_SetTimerContext */

  TIMER_IF_DBG_PRINTF("TIMER_IF_SetTimerContext=%d\n\r", RtcTimerContext);
  /*return time context*/
  return RtcTimerContext;
}

uint32_t TIMER_IF_GetTimerContext(void)
{
  /* USER CODE BEGIN TIMER_IF_GetTimerContext */

  /* USER CODE END TIMER_IF_GetTimerContext */

  TIMER_IF_DBG_PRINTF("TIMER_IF_GetTimerContext=%d\n\r", RtcTimerContext);
  /*return time context*/
  return RtcTimerContext;
}

uint32_t TIMER_IF_GetTimerElapsedTime(void)
{
  uint32_t ret = 0;
  /* USER CODE BEGIN TIMER_IF_GetTimerElapsedTime */

  /* USER CODE END TIMER_IF_GetTimerElapsedTime */
  ret = ((uint32_t)(GetTimerTicks() - RtcTimerContext));
  /* USER CODE BEGIN TIMER_IF_GetTimerElapsedTime_Last */

  /* USER CODE END TIMER_IF_GetTimerElapsedTime_Last */
  return ret;
}

uint32_t TIMER_IF_GetTimerValue(void)
{
  uint32_t ret = 0;
  /* USER CODE BEGIN TIMER_IF_GetTimerValue */

  /* USER CODE END TIMER_IF_GetTimerValue */
  if (RTC_Initialized == true)
  {
    ret = GetTimerTicks();
  }
  /* USER CODE BEGIN TIMER_IF_GetTimerValue_Last */

  /* USER CODE END TIMER_IF_GetTimerValue_Last */
  return ret;
}

uint32_t TIMER_IF_GetMinimumTimeout(void)
{
  uint32_t ret = 0;
  /* USER CODE BEGIN TIMER_IF_GetMinimumTimeout */

  /* USER CODE END TIMER_IF_GetMinimumTimeout */
  ret = (MIN_ALARM_DELAY);
  /* USER CODE BEGIN TIMER_IF_GetMinimumTimeout_Last */

  /* USER CODE END TIMER_IF_GetMinimumTimeout_Last */
  return ret;
}

uint32_t TIMER_IF_Convert_ms2Tick(uint32_t timeMilliSec)
{
  uint32_t ret = 0;
  /* USER CODE BEGIN TIMER_IF_Convert_ms2Tick */

  /* USER CODE END TIMER_IF_Convert_ms2Tick */
  ret = ((uint32_t)((((uint64_t) timeMilliSec) << RTC_N_PREDIV_S) / 1000));
  /* USER CODE BEGIN TIMER_IF_Convert_ms2Tick_Last */

  /* USER CODE END TIMER_IF_Convert_ms2Tick_Last */
  return ret;
}

uint32_t TIMER_IF_Convert_Tick2ms(uint32_t tick)
{
  uint32_t ret = 0;
  /* USER CODE BEGIN TIMER_IF_Convert_Tick2ms */

  /* USER CODE END TIMER_IF_Convert_Tick2ms */
  ret = ((uint32_t)((((uint64_t)(tick)) * 1000) >> RTC_N_PREDIV_S));
  /* USER CODE BEGIN TIMER_IF_Convert_Tick2ms_Last */

  /* USER CODE END TIMER_IF_Convert_Tick2ms_Last */
  return ret;
}

void TIMER_IF_DelayMs(uint32_t delay)
{
  /* USER CODE BEGIN TIMER_IF_DelayMs */

  /* USER CODE END TIMER_IF_DelayMs */
  uint32_t delayTicks = TIMER_IF_Convert_ms2Tick(delay);
  uint32_t timeout = GetTimerTicks();

  /* Wait delay ms */
  while (((GetTimerTicks() - timeout)) < delayTicks)
  {
    __NOP();
  }
  /* USER CODE BEGIN TIMER_IF_DelayMs_Last */

  /* USER CODE END TIMER_IF_DelayMs_Last */
}

void HAL_RTCEx_SSRUEventCallback(RTC_HandleTypeDef *hrtc)
{
  (void)hrtc; // unused
  /* USER CODE BEGIN HAL_RTCEx_SSRUEventCallback */

  /* USER CODE END HAL_RTCEx_SSRUEventCallback */
  /*called every 48 days with 1024 ticks per seconds*/
  TIMER_IF_DBG_PRINTF(">>Handler SSRUnderflow at %d\n\r", GetTimerTicks());
  /*Increment MSBticks*/
  uint32_t MSB_ticks = TIMER_IF_BkUp_Read_MSBticks();
  TIMER_IF_BkUp_Write_MSBticks(MSB_ticks + 1);
  /* USER CODE BEGIN HAL_RTCEx_SSRUEventCallback_Last */

  /* USER CODE END HAL_RTCEx_SSRUEventCallback_Last */
}

uint32_t TIMER_IF_GetTime(uint16_t *mSeconds)
{
  uint32_t seconds = 0;
  /* USER CODE BEGIN TIMER_IF_GetTime */

  /* USER CODE END TIMER_IF_GetTime */
  uint64_t ticks;
  uint32_t timerValueLsb = GetTimerTicks();
  uint32_t timerValueMSB = TIMER_IF_BkUp_Read_MSBticks();

  ticks = (((uint64_t) timerValueMSB) << 32) + timerValueLsb;

  seconds = (uint32_t)(ticks >> RTC_N_PREDIV_S);

  ticks = (uint32_t) ticks & RTC_PREDIV_S;

  *mSeconds = TIMER_IF_Convert_Tick2ms(ticks);

  /* USER CODE BEGIN TIMER_IF_GetTime_Last */

  /* USER CODE END TIMER_IF_GetTime_Last */
  return seconds;
}

void TIMER_IF_BkUp_Write_Seconds(uint32_t Seconds)
{
  /* USER CODE BEGIN TIMER_IF_BkUp_Write_Seconds */

  /* USER CODE END TIMER_IF_BkUp_Write_Seconds */
  HAL_RTCEx_BKUPWrite(&RtcHandle, RTC_BKP_SECONDS, Seconds);
  /* USER CODE BEGIN TIMER_IF_BkUp_Write_Seconds_Last */

  /* USER CODE END TIMER_IF_BkUp_Write_Seconds_Last */
}

void TIMER_IF_BkUp_Write_SubSeconds(uint32_t SubSeconds)
{
  /* USER CODE BEGIN TIMER_IF_BkUp_Write_SubSeconds */

  /* USER CODE END TIMER_IF_BkUp_Write_SubSeconds */
  HAL_RTCEx_BKUPWrite(&RtcHandle, RTC_BKP_SUBSECONDS, SubSeconds);
  /* USER CODE BEGIN TIMER_IF_BkUp_Write_SubSeconds_Last */

  /* USER CODE END TIMER_IF_BkUp_Write_SubSeconds_Last */
}

uint32_t TIMER_IF_BkUp_Read_Seconds(void)
{
  uint32_t ret = 0;
  /* USER CODE BEGIN TIMER_IF_BkUp_Read_Seconds */

  /* USER CODE END TIMER_IF_BkUp_Read_Seconds */
  ret = HAL_RTCEx_BKUPRead(&RtcHandle, RTC_BKP_SECONDS);
  /* USER CODE BEGIN TIMER_IF_BkUp_Read_Seconds_Last */

  /* USER CODE END TIMER_IF_BkUp_Read_Seconds_Last */
  return ret;
}

uint32_t TIMER_IF_BkUp_Read_SubSeconds(void)
{
  uint32_t ret = 0;
  /* USER CODE BEGIN TIMER_IF_BkUp_Read_SubSeconds */

  /* USER CODE END TIMER_IF_BkUp_Read_SubSeconds */
  ret = HAL_RTCEx_BKUPRead(&RtcHandle, RTC_BKP_SUBSECONDS);
  /* USER CODE BEGIN TIMER_IF_BkUp_Read_SubSeconds_Last */

  /* USER CODE END TIMER_IF_BkUp_Read_SubSeconds_Last */
  return ret;
}

/* USER CODE BEGIN EF */

/* USER CODE END EF */

/* Private functions ---------------------------------------------------------*/
static void TIMER_IF_BkUp_Write_MSBticks(uint32_t MSBticks)
{
  /* USER CODE BEGIN TIMER_IF_BkUp_Write_MSBticks */

  /* USER CODE END TIMER_IF_BkUp_Write_MSBticks */
  HAL_RTCEx_BKUPWrite(&RtcHandle, RTC_BKP_MSBTICKS, MSBticks);
  /* USER CODE BEGIN TIMER_IF_BkUp_Write_MSBticks_Last */

  /* USER CODE END TIMER_IF_BkUp_Write_MSBticks_Last */
}

static uint32_t TIMER_IF_BkUp_Read_MSBticks(void)
{
  /* USER CODE BEGIN TIMER_IF_BkUp_Read_MSBticks */

  /* USER CODE END TIMER_IF_BkUp_Read_MSBticks */
  uint32_t MSBticks;
  MSBticks = HAL_RTCEx_BKUPRead(&RtcHandle, RTC_BKP_MSBTICKS);
  return MSBticks;
  /* USER CODE BEGIN TIMER_IF_BkUp_Read_MSBticks_Last */

  /* USER CODE END TIMER_IF_BkUp_Read_MSBticks_Last */
}

static inline uint32_t GetTimerTicks(void)
{
  /* USER CODE BEGIN GetTimerTicks */

  /* USER CODE END GetTimerTicks */
#if 0
  uint32_t ssr = LL_RTC_TIME_GetSubSecond(RTC);
  /* read twice to make sure value it valid*/
  while (ssr != LL_RTC_TIME_GetSubSecond(RTC))
  {
    ssr = LL_RTC_TIME_GetSubSecond(RTC);
  }
//  return UINT32_MAX - ssr;
   return ssr;

 #else
  uint32_t bcd_time = LL_RTC_TIME_Get(RTC); /* Format: 0x00HHMMSS */
  uint32_t binary_time = 0;
binary_time += (__LL_RTC_CONVERT_BCD2BIN((bcd_time >> 16) & 0xFF)) * 3600;
binary_time += (__LL_RTC_CONVERT_BCD2BIN((bcd_time >> 8) & 0xFF)) * 60;
binary_time += (__LL_RTC_CONVERT_BCD2BIN((bcd_time >> 0) & 0xFF)) * 1;
binary_time = (binary_time * 1000) + (LL_RTC_TIME_GetSubSecond(RTC) *1000)/1000;
return binary_time;
#endif
  /* USER CODE BEGIN GetTimerTicks_Last */

  /* USER CODE END GetTimerTicks_Last */
}

static inline bool isLeapYear(uint8_t year2k)
{
  int year = year2k + 2000;

  // if year not divisible by 4 - not a leap year
  // else if year divisible by 4 and not by 100 - a leap year
  // else if year divisible by 400 - a leap year
  return (year % 4 != 0) ? false : (year % 100 != 0) ? true : year % 400 == 0;
}

/* USER CODE BEGIN PrFD */

/* USER CODE END PrFD */
