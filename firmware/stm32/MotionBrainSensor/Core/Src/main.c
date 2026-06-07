/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_MODE_SENSOR_BRIDGE 0U
#define APP_MODE_TELEOP_REMOTE 1U
#define APP_MODE APP_MODE_TELEOP_REMOTE
#define SAFETY_TELEMETRY_ENABLED ((APP_MODE == APP_MODE_SENSOR_BRIDGE) || (APP_MODE == APP_MODE_TELEOP_REMOTE))
#define TELEOP_DIAGNOSTIC_BUTTON_SCAN 0U

#define MPU_SAMPLE_RATE_HZ 200U
#define SENSOR_TX_RATE_HZ 10U
#define TELEOP_TX_RATE_HZ 25U
#define MPU_CALIBRATION_SAMPLES 400U
#define MPU_FILTER_TAU_SEC 0.5f
#define HCSR04_ENABLED 0U
#define HCSR04_TRIGGER_INTERVAL_MS 100U
#define HCSR04_TIMEOUT_US 30000U
#define HCSR04_MAX_VALID_CM 400.0f
#define HCSR04_CM_PER_US 0.01715f

#define MPU_STATUS_NOT_PROBED 0U
#define MPU_STATUS_READY 1U
#define MPU_STATUS_PROBE_FAIL 2U
#define MPU_STATUS_WHOAMI_FAIL 3U
#define MPU_STATUS_INIT_FAIL 4U
#define MPU_STATUS_CALIB_FAIL 5U
#define MPU_STATUS_READ_FAIL 6U

// Handheld remote v1 provisional button map.
// B-F446E-96B01A에서 현재 바로 꽂기 쉬운 Arduino digital header 기준.
// 현재 사용 중인 D1(UART TX), D2/D3(HC-SR04), D14/D15(I2C2)를 피하고
// 완전히 비어 있는 D9/D10/D11/D13만 버튼 입력으로 사용한다.
#define TELEOP_DEADMAN_GPIO_Port GPIOE
#define TELEOP_DEADMAN_Pin GPIO_PIN_4   // Arduino D10
#define TELEOP_LED_GPIO_Port GPIOB
#define TELEOP_LED_Pin GPIO_PIN_4       // Arduino D9
#define TELEOP_LED_ALT_ENABLED 0U
#define TELEOP_LED_ALT_GPIO_Port GPIOD
#define TELEOP_LED_ALT_Pin GPIO_PIN_15  // Arduino D5 fallback
#define TELEOP_GRIP_OPEN_GPIO_Port GPIOE
#define TELEOP_GRIP_OPEN_Pin GPIO_PIN_2 // Arduino D13
#define TELEOP_GRIP_OPEN_ALT_ENABLED 0U
#define TELEOP_GRIP_OPEN_ALT_GPIO_Port GPIOD
#define TELEOP_GRIP_OPEN_ALT_Pin GPIO_PIN_2 // Arduino D4 fallback
#define TELEOP_GRIP_CLOSE_GPIO_Port GPIOE
#define TELEOP_GRIP_CLOSE_Pin GPIO_PIN_6 // Arduino D11

#define DIAG_G1_D4_GPIO_Port GPIOD
#define DIAG_G1_D4_Pin GPIO_PIN_2
#define DIAG_G1_D5_GPIO_Port GPIOD
#define DIAG_G1_D5_Pin GPIO_PIN_15
#define DIAG_G1_D6_GPIO_Port GPIOD
#define DIAG_G1_D6_Pin GPIO_PIN_14
#define DIAG_G1_D7_GPIO_Port GPIOD
#define DIAG_G1_D7_Pin GPIO_PIN_13
#define DIAG_G1_D8_GPIO_Port GPIOE
#define DIAG_G1_D8_Pin GPIO_PIN_3
#define DIAG_G1_D9_GPIO_Port GPIOB
#define DIAG_G1_D9_Pin GPIO_PIN_4
#define DIAG_G1_D10_GPIO_Port GPIOE
#define DIAG_G1_D10_Pin GPIO_PIN_4
#define DIAG_G1_D11_GPIO_Port GPIOE
#define DIAG_G1_D11_Pin GPIO_PIN_6

#define DIAG_G2_D12_GPIO_Port GPIOE
#define DIAG_G2_D12_Pin GPIO_PIN_5
#define DIAG_G2_D13_GPIO_Port GPIOE
#define DIAG_G2_D13_Pin GPIO_PIN_2
#define DIAG_G2_A0_GPIO_Port GPIOA
#define DIAG_G2_A0_Pin GPIO_PIN_1
#define DIAG_G2_A1_GPIO_Port GPIOA
#define DIAG_G2_A1_Pin GPIO_PIN_2
#define DIAG_G2_A2_GPIO_Port GPIOC
#define DIAG_G2_A2_Pin GPIO_PIN_3
#define DIAG_G2_A3_GPIO_Port GPIOC
#define DIAG_G2_A3_Pin GPIO_PIN_2
#define DIAG_G2_A4_GPIO_Port GPIOB
#define DIAG_G2_A4_Pin GPIO_PIN_1
#define DIAG_G2_A5_GPIO_Port GPIOC
#define DIAG_G2_A5_Pin GPIO_PIN_0

#define DIAG_G3_PA0_GPIO_Port GPIOA
#define DIAG_G3_PA0_Pin GPIO_PIN_0
#define DIAG_G3_PA4_GPIO_Port GPIOA
#define DIAG_G3_PA4_Pin GPIO_PIN_4
#define DIAG_G3_PB0_GPIO_Port GPIOB
#define DIAG_G3_PB0_Pin GPIO_PIN_0

#define TELEOP_BUTTON_ACTIVE_STATE GPIO_PIN_RESET
#define TELEOP_BUTTON_PULL GPIO_PULLUP

#define TELEOP_REACH_SIGN 1.0f
#define TELEOP_LIFT_SIGN 1.0f
#define TELEOP_TWIST_SIGN 1.0f

#define TELEOP_ANGLE_DEADZONE_DEG 6.0f
#define TELEOP_ANGLE_FULL_SCALE_DEG 45.0f
#define TELEOP_TWIST_RATE_DEADZONE_DPS 2.0f
#define TELEOP_TWIST_ANGLE_DEADZONE_DEG 6.0f
#define TELEOP_TWIST_ANGLE_FULL_SCALE_DEG 35.0f
#define TELEOP_TWIST_ANGLE_LIMIT_DEG 60.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint16_t g_mpu_addr = 0;
static uint8_t g_mpu_ready = 0;
static uint8_t g_imu_ok = 0;
static uint32_t g_mpu_status = MPU_STATUS_NOT_PROBED;
static uint32_t g_mpu_error = 0;
static volatile uint32_t g_sample_due_count = 0;
static float g_gyro_bias_x_dps = 0.0f;
static float g_gyro_bias_y_dps = 0.0f;
static float g_gyro_bias_z_dps = 0.0f;
static float g_roll_deg = 0.0f;
static float g_pitch_deg = 0.0f;
static float g_gyro_x_dps = 0.0f;
static float g_gyro_y_dps = 0.0f;
static float g_gyro_z_dps = 0.0f;
static float g_vibe = 0.0f;
static uint8_t g_attitude_ready = 0;
#if APP_MODE == APP_MODE_SENSOR_BRIDGE
static uint32_t g_tx_divider = 0;
#endif
#if APP_MODE == APP_MODE_TELEOP_REMOTE
static uint32_t g_last_teleop_tx_ms = 0;
#endif
#if SAFETY_TELEMETRY_ENABLED
static float g_distance_cm = 0.0f;
static uint8_t g_range_ok = HCSR04_ENABLED ? 0U : 1U;
static uint32_t g_last_hcsr04_trigger_ms = 0;
#if HCSR04_ENABLED
static uint32_t g_hcsr04_trigger_started_us = 0;
static volatile uint32_t g_hcsr04_rise_us = 0;
static volatile uint32_t g_hcsr04_echo_width_us = 0;
static volatile uint8_t g_hcsr04_waiting_for_fall = 0;
static volatile uint8_t g_hcsr04_measurement_ready = 0;
#endif
static volatile uint8_t g_hcsr04_echo_pending = 0;
#endif
static uint32_t g_teleop_session = 0;
static uint32_t g_teleop_sequence = 0;
static uint32_t g_teleop_led_toggle_seq = 0;
static uint8_t g_deadman_active = 0;
static uint8_t g_prev_deadman_pressed = 0;
static uint8_t g_prev_led_button_pressed = 0;
static float g_teleop_neutral_roll_deg = 0.0f;
static float g_teleop_neutral_pitch_deg = 0.0f;
static float g_teleop_twist_angle_deg = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// ITM SWV printf 리다이렉트 (SWO 핀 -> STM32CubeIDE SWV Console)
int _write(int file, char *ptr, int len)
{
    (void)file;
    for (int i = 0; i < len; i++) {
        ITM_SendChar((uint32_t)ptr[i]);
    }
    return len;
}

static void EnableCycleCounter(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

#if SAFETY_TELEMETRY_ENABLED && HCSR04_ENABLED
static uint32_t Micros(void)
{
    return DWT->CYCCNT / (HAL_RCC_GetHCLKFreq() / 1000000UL);
}

static void DelayUs(uint32_t delay_us)
{
    uint32_t start = Micros();
    while ((Micros() - start) < delay_us) {
    }
}
#endif

static void RecoverI2c2Bus(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_Delay(2);

    for (uint32_t i = 0; i < 18U && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == GPIO_PIN_RESET; ++i) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        for (volatile uint32_t delay = 0; delay < 120U; ++delay) {
        }
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        for (volatile uint32_t delay = 0; delay < 120U; ++delay) {
        }
    }

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
    for (volatile uint32_t delay = 0; delay < 120U; ++delay) {
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    for (volatile uint32_t delay = 0; delay < 120U; ++delay) {
    }
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_Delay(2);
}

static void ProbeMpu6050(I2C_HandleTypeDef *hi2c)
{
    const uint16_t addresses[] = {0x68 << 1, 0x69 << 1};
    g_mpu_status = MPU_STATUS_PROBE_FAIL;
    g_mpu_error = 0;

    for (size_t i = 0; i < (sizeof(addresses) / sizeof(addresses[0])); ++i) {
        uint16_t addr = addresses[i];
        HAL_StatusTypeDef ready = HAL_I2C_IsDeviceReady(hi2c, addr, 3, 100);
        uint32_t err = HAL_I2C_GetError(hi2c);
        g_mpu_addr = addr;
        g_mpu_error = err;

        printf("Probe 0x%02X: ready=%d err=0x%08lX\r\n",
               addr >> 1,
               ready,
               (unsigned long)err);

        if (ready != HAL_OK) {
            continue;
        }

        uint8_t who = 0;
        HAL_StatusTypeDef check = MPU6050_Check(hi2c, addr, &who);
        g_mpu_error = HAL_I2C_GetError(hi2c);
        printf("WHO_AM_I@0x%02X: ret=%d who=0x%02X err=0x%08lX\r\n",
               addr >> 1,
               check,
               who,
               (unsigned long)HAL_I2C_GetError(hi2c));

        if (check != HAL_OK) {
            g_mpu_status = MPU_STATUS_WHOAMI_FAIL;
            continue;
        }

        HAL_StatusTypeDef init = MPU6050_InitAt(hi2c, addr);
        g_mpu_error = HAL_I2C_GetError(hi2c);
        printf("Init 0x%02X: ret=%d err=0x%08lX\r\n",
               addr >> 1,
               init,
               (unsigned long)HAL_I2C_GetError(hi2c));

        if (init == HAL_OK) {
            g_mpu_addr = addr;
            g_mpu_ready = 1;
            g_mpu_status = MPU_STATUS_READY;
            return;
        }
        g_mpu_status = MPU_STATUS_INIT_FAIL;
    }
}

static void PrintFixed3(const char *label, float value)
{
    long scaled = lroundf(value * 1000.0f);
    unsigned long abs_scaled;

    if (scaled < 0) {
        abs_scaled = (unsigned long)(-scaled);
        printf("%s=-%lu.%03lu", label, abs_scaled / 1000UL, abs_scaled % 1000UL);
    } else {
        abs_scaled = (unsigned long)scaled;
        printf("%s=%lu.%03lu", label, abs_scaled / 1000UL, abs_scaled % 1000UL);
    }
}

static void FormatFixedValue(char *buffer, size_t size, float value, uint32_t scale, uint8_t decimals)
{
    long scaled = lroundf(value * (float)scale);
    unsigned long abs_scaled = (scaled < 0) ? (unsigned long)(-scaled) : (unsigned long)scaled;
    unsigned long whole = abs_scaled / scale;
    unsigned long fraction = abs_scaled % scale;
    const char *sign = (scaled < 0) ? "-" : "";

    if (decimals == 1U) {
        snprintf(buffer, size, "%s%lu.%01lu", sign, whole, fraction);
    } else if (decimals == 2U) {
        snprintf(buffer, size, "%s%lu.%02lu", sign, whole, fraction);
    } else {
        snprintf(buffer, size, "%s%lu.%03lu", sign, whole, fraction);
    }
}

static void TransmitFormattedPacket(char *buffer, size_t size, int length)
{
    if (length <= 0 || size == 0U) {
        return;
    }
    if ((size_t)length >= size) {
        return;
    }
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, (uint16_t)length, 50U);
}

#if SAFETY_TELEMETRY_ENABLED
static void TriggerHcsr04(void)
{
#if HCSR04_ENABLED
    HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_RESET);
    DelayUs(2U);
    HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_SET);
    DelayUs(10U);
    HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_RESET);

    g_hcsr04_trigger_started_us = Micros();
    g_hcsr04_echo_pending = 1U;
    g_hcsr04_waiting_for_fall = 0U;
#else
    g_distance_cm = 0.0f;
    g_range_ok = 1U;
#endif
}

static void UpdateRangeMeasurement(void)
{
#if HCSR04_ENABLED
    uint8_t measurement_ready = 0U;
    uint32_t pulse_width_us = 0U;

    __disable_irq();
    if (g_hcsr04_measurement_ready) {
        measurement_ready = 1U;
        pulse_width_us = g_hcsr04_echo_width_us;
        g_hcsr04_measurement_ready = 0U;
    }
    __enable_irq();

    if (measurement_ready) {
        g_distance_cm = (float)pulse_width_us * HCSR04_CM_PER_US;
        g_range_ok = (g_distance_cm > 0.0f && g_distance_cm <= HCSR04_MAX_VALID_CM) ? 1U : 0U;
        return;
    }

    if (g_hcsr04_echo_pending && (Micros() - g_hcsr04_trigger_started_us) > HCSR04_TIMEOUT_US) {
        g_hcsr04_echo_pending = 0U;
        g_hcsr04_waiting_for_fall = 0U;
        g_range_ok = 0U;
        g_distance_cm = 0.0f;
        printf("HC-SR04 timeout\r\n");
    }
#else
    g_distance_cm = 0.0f;
    g_range_ok = 1U;
#endif
}
#endif

static float ClampUnit(float value)
{
    if (value > 1.0f) {
        return 1.0f;
    }
    if (value < -1.0f) {
        return -1.0f;
    }
    return value;
}

static uint8_t ReadButton(GPIO_TypeDef *port, uint16_t pin)
{
    GPIO_PinState state = HAL_GPIO_ReadPin(port, pin);
    return (state == TELEOP_BUTTON_ACTIVE_STATE) ? 1U : 0U;
}

#if TELEOP_DIAGNOSTIC_BUTTON_SCAN
static uint8_t ReadDiagnosticGroup1Mask(void)
{
    uint8_t mask = 0U;
    if (ReadButton(DIAG_G1_D4_GPIO_Port, DIAG_G1_D4_Pin)) mask |= 0x01U;
    if (ReadButton(DIAG_G1_D5_GPIO_Port, DIAG_G1_D5_Pin)) mask |= 0x02U;
    if (ReadButton(DIAG_G1_D6_GPIO_Port, DIAG_G1_D6_Pin)) mask |= 0x04U;
    if (ReadButton(DIAG_G1_D7_GPIO_Port, DIAG_G1_D7_Pin)) mask |= 0x08U;
    if (ReadButton(DIAG_G1_D8_GPIO_Port, DIAG_G1_D8_Pin)) mask |= 0x10U;
    if (ReadButton(DIAG_G1_D9_GPIO_Port, DIAG_G1_D9_Pin)) mask |= 0x20U;
    if (ReadButton(DIAG_G1_D10_GPIO_Port, DIAG_G1_D10_Pin)) mask |= 0x40U;
    if (ReadButton(DIAG_G1_D11_GPIO_Port, DIAG_G1_D11_Pin)) mask |= 0x80U;
    return mask;
}

static uint8_t ReadDiagnosticGroup2Mask(void)
{
    uint8_t mask = 0U;
    if (ReadButton(DIAG_G2_D12_GPIO_Port, DIAG_G2_D12_Pin)) mask |= 0x01U;
    if (ReadButton(DIAG_G2_D13_GPIO_Port, DIAG_G2_D13_Pin)) mask |= 0x02U;
    if (ReadButton(DIAG_G2_A0_GPIO_Port, DIAG_G2_A0_Pin)) mask |= 0x04U;
    if (ReadButton(DIAG_G2_A1_GPIO_Port, DIAG_G2_A1_Pin)) mask |= 0x08U;
    if (ReadButton(DIAG_G2_A2_GPIO_Port, DIAG_G2_A2_Pin)) mask |= 0x10U;
    if (ReadButton(DIAG_G2_A3_GPIO_Port, DIAG_G2_A3_Pin)) mask |= 0x20U;
    if (ReadButton(DIAG_G2_A4_GPIO_Port, DIAG_G2_A4_Pin)) mask |= 0x40U;
    if (ReadButton(DIAG_G2_A5_GPIO_Port, DIAG_G2_A5_Pin)) mask |= 0x80U;
    return mask;
}

static uint8_t ReadDiagnosticGroup3Mask(void)
{
    uint8_t mask = 0U;
    if (ReadButton(DIAG_G3_PA0_GPIO_Port, DIAG_G3_PA0_Pin)) mask |= 0x01U;
    if (ReadButton(DIAG_G3_PA4_GPIO_Port, DIAG_G3_PA4_Pin)) mask |= 0x02U;
    if (ReadButton(DIAG_G3_PB0_GPIO_Port, DIAG_G3_PB0_Pin)) mask |= 0x04U;
    return mask;
}
#endif

static uint8_t ReadEitherButton(GPIO_TypeDef *primaryPort, uint16_t primaryPin,
                                GPIO_TypeDef *alternatePort, uint16_t alternatePin)
{
    return (ReadButton(primaryPort, primaryPin) || ReadButton(alternatePort, alternatePin)) ? 1U : 0U;
}

static uint8_t ReadButtonWithOptionalAlternate(GPIO_TypeDef *primaryPort, uint16_t primaryPin,
                                               uint8_t alternateEnabled,
                                               GPIO_TypeDef *alternatePort, uint16_t alternatePin)
{
    if (!alternateEnabled) {
        return ReadButton(primaryPort, primaryPin);
    }
    return ReadEitherButton(primaryPort, primaryPin, alternatePort, alternatePin);
}

static void InitTeleopInputs(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = TELEOP_BUTTON_PULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = TELEOP_DEADMAN_Pin;
    HAL_GPIO_Init(TELEOP_DEADMAN_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = TELEOP_LED_Pin;
    HAL_GPIO_Init(TELEOP_LED_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = TELEOP_LED_ALT_Pin;
    HAL_GPIO_Init(TELEOP_LED_ALT_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = TELEOP_GRIP_OPEN_Pin;
    HAL_GPIO_Init(TELEOP_GRIP_OPEN_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = TELEOP_GRIP_OPEN_ALT_Pin;
    HAL_GPIO_Init(TELEOP_GRIP_OPEN_ALT_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = TELEOP_GRIP_CLOSE_Pin;
    HAL_GPIO_Init(TELEOP_GRIP_CLOSE_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DIAG_G1_D4_Pin | DIAG_G1_D5_Pin | DIAG_G1_D6_Pin | DIAG_G1_D7_Pin;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DIAG_G1_D8_Pin | DIAG_G1_D10_Pin | DIAG_G1_D11_Pin |
                          DIAG_G2_D12_Pin | DIAG_G2_D13_Pin;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DIAG_G1_D9_Pin | DIAG_G2_A4_Pin | DIAG_G3_PB0_Pin;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DIAG_G2_A0_Pin | DIAG_G2_A1_Pin | DIAG_G3_PA0_Pin | DIAG_G3_PA4_Pin;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DIAG_G2_A2_Pin | DIAG_G2_A3_Pin | DIAG_G2_A5_Pin;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

static float NormalizeAxis(float value, float deadzone, float fullScale)
{
    float magnitude;
    float normalized;

    if (fullScale <= deadzone) {
        return 0.0f;
    }

    magnitude = fabsf(value);
    if (magnitude <= deadzone) {
        return 0.0f;
    }

    normalized = (magnitude - deadzone) / (fullScale - deadzone);
    if (normalized > 1.0f) {
        normalized = 1.0f;
    }

    return (value >= 0.0f) ? normalized : -normalized;
}

static void UpdateTeleopTwistAngle(float dt_sec)
{
#if APP_MODE == APP_MODE_TELEOP_REMOTE
    if (!g_deadman_active || !g_imu_ok) {
        if (!g_deadman_active) {
            g_teleop_twist_angle_deg = 0.0f;
        }
        return;
    }

    float twist_rate_dps = g_gyro_z_dps * TELEOP_TWIST_SIGN;
    if (fabsf(twist_rate_dps) <= TELEOP_TWIST_RATE_DEADZONE_DPS) {
        return;
    }

    g_teleop_twist_angle_deg += twist_rate_dps * dt_sec;
    if (g_teleop_twist_angle_deg > TELEOP_TWIST_ANGLE_LIMIT_DEG) {
        g_teleop_twist_angle_deg = TELEOP_TWIST_ANGLE_LIMIT_DEG;
    } else if (g_teleop_twist_angle_deg < -TELEOP_TWIST_ANGLE_LIMIT_DEG) {
        g_teleop_twist_angle_deg = -TELEOP_TWIST_ANGLE_LIMIT_DEG;
    }
#else
    (void)dt_sec;
#endif
}

static float GetTeleopTwistCommand(void)
{
    return NormalizeAxis(g_teleop_twist_angle_deg,
                         TELEOP_TWIST_ANGLE_DEADZONE_DEG,
                         TELEOP_TWIST_ANGLE_FULL_SCALE_DEG);
}

static void UpdateTeleopState(void)
{
#if TELEOP_DIAGNOSTIC_BUTTON_SCAN
    g_deadman_active = 0U;
    g_prev_deadman_pressed = 0U;
    g_prev_led_button_pressed = 0U;
    return;
#endif

    uint8_t deadman_pressed = ReadButton(TELEOP_DEADMAN_GPIO_Port, TELEOP_DEADMAN_Pin);
    uint8_t led_pressed = ReadButtonWithOptionalAlternate(TELEOP_LED_GPIO_Port, TELEOP_LED_Pin,
                                                          TELEOP_LED_ALT_ENABLED,
                                                          TELEOP_LED_ALT_GPIO_Port, TELEOP_LED_ALT_Pin);

    if (deadman_pressed && !g_prev_deadman_pressed) {
        g_teleop_session++;
        g_teleop_neutral_roll_deg = g_roll_deg;
        g_teleop_neutral_pitch_deg = g_pitch_deg;
        g_teleop_twist_angle_deg = 0.0f;
        g_deadman_active = 1U;
        printf("Teleop session=%lu ", (unsigned long)g_teleop_session);
        PrintFixed3("neutral_roll", g_teleop_neutral_roll_deg);
        printf(" ");
        PrintFixed3("neutral_pitch", g_teleop_neutral_pitch_deg);
        printf("\r\n");
    } else if (!deadman_pressed) {
        g_deadman_active = 0U;
        g_teleop_twist_angle_deg = 0.0f;
    }

    if (led_pressed && !g_prev_led_button_pressed) {
        g_teleop_led_toggle_seq++;
    }

    g_prev_deadman_pressed = deadman_pressed;
    g_prev_led_button_pressed = led_pressed;
}

static void SendTeleopPacket(void)
{
    char reach_str[24];
    char lift_str[24];
    char twist_str[24];
    char roll_str[24];
    char pitch_str[24];
    char gyro_x_str[24];
    char gyro_y_str[24];
    char gyro_z_str[24];
    char vibe_str[24];
    char dist_str[24];
    char tx_buffer[512];
    int length;
    float reach = 0.0f;
    float lift = 0.0f;
    float twist = 0.0f;
    uint32_t frame_sequence = g_teleop_sequence;
    uint32_t frame_session = g_teleop_session;
    uint32_t frame_led_toggle_seq = g_teleop_led_toggle_seq;
    uint8_t grip_open = ReadButtonWithOptionalAlternate(TELEOP_GRIP_OPEN_GPIO_Port, TELEOP_GRIP_OPEN_Pin,
                                                        TELEOP_GRIP_OPEN_ALT_ENABLED,
                                                        TELEOP_GRIP_OPEN_ALT_GPIO_Port, TELEOP_GRIP_OPEN_ALT_Pin);
    uint8_t grip_close = ReadButton(TELEOP_GRIP_CLOSE_GPIO_Port, TELEOP_GRIP_CLOSE_Pin);

    UpdateTeleopState();

#if TELEOP_DIAGNOSTIC_BUTTON_SCAN
    frame_session = ReadDiagnosticGroup1Mask();
    frame_sequence = ReadDiagnosticGroup2Mask();
    frame_led_toggle_seq = ReadDiagnosticGroup3Mask();
    grip_open = 0U;
    grip_close = 0U;
#else
    if (g_deadman_active && g_imu_ok) {
        reach = NormalizeAxis((g_pitch_deg - g_teleop_neutral_pitch_deg) * TELEOP_REACH_SIGN,
                              TELEOP_ANGLE_DEADZONE_DEG,
                              TELEOP_ANGLE_FULL_SCALE_DEG);
        lift = NormalizeAxis((g_roll_deg - g_teleop_neutral_roll_deg) * TELEOP_LIFT_SIGN,
                             TELEOP_ANGLE_DEADZONE_DEG,
                             TELEOP_ANGLE_FULL_SCALE_DEG);
        twist = GetTeleopTwistCommand();
    }
#endif

    reach = ClampUnit(reach);
    lift = ClampUnit(lift);
    twist = ClampUnit(twist);

    FormatFixedValue(reach_str, sizeof(reach_str), reach, 1000U, 3U);
    FormatFixedValue(lift_str, sizeof(lift_str), lift, 1000U, 3U);
    FormatFixedValue(twist_str, sizeof(twist_str), twist, 1000U, 3U);
    FormatFixedValue(roll_str, sizeof(roll_str), g_roll_deg, 1000U, 3U);
    FormatFixedValue(pitch_str, sizeof(pitch_str), g_pitch_deg, 1000U, 3U);
    FormatFixedValue(gyro_x_str, sizeof(gyro_x_str), g_gyro_x_dps, 1000U, 3U);
    FormatFixedValue(gyro_y_str, sizeof(gyro_y_str), g_gyro_y_dps, 1000U, 3U);
    FormatFixedValue(gyro_z_str, sizeof(gyro_z_str), g_gyro_z_dps, 1000U, 3U);
    FormatFixedValue(vibe_str, sizeof(vibe_str), g_vibe, 100U, 2U);
    FormatFixedValue(dist_str, sizeof(dist_str), g_distance_cm, 10U, 1U);

    if (!TELEOP_DIAGNOSTIC_BUTTON_SCAN) {
        g_teleop_sequence++;
        frame_sequence = g_teleop_sequence;
        frame_session = g_teleop_session;
        frame_led_toggle_seq = g_teleop_led_toggle_seq;
    }
    length = snprintf(tx_buffer,
                      sizeof(tx_buffer),
                      "{\"type\":\"teleop\",\"ts_ms\":%lu,\"seq\":%lu,\"session\":%lu,"
                      "\"deadman\":%s,\"reach\":%s,\"lift\":%s,\"twist\":%s,"
                      "\"grip_open\":%s,\"grip_close\":%s,\"led_toggle_seq\":%lu,"
                      "\"imu_ok\":%s,\"range_ok\":%s,\"roll\":%s,\"pitch\":%s,"
                      "\"gyro_x\":%s,\"gyro_y\":%s,\"gyro_z\":%s,\"vibe\":%s,\"dist_cm\":%s,"
                      "\"imu_status\":%lu,\"imu_addr\":%lu,\"imu_error\":%lu,"
                      "\"i2c_scl\":%s,\"i2c_sda\":%s}\r\n",
                      (unsigned long)HAL_GetTick(),
                      (unsigned long)frame_sequence,
                      (unsigned long)frame_session,
                      g_deadman_active ? "true" : "false",
                      reach_str,
                      lift_str,
                      twist_str,
                      grip_open ? "true" : "false",
                      grip_close ? "true" : "false",
                      (unsigned long)frame_led_toggle_seq,
                      g_imu_ok ? "true" : "false",
                      g_range_ok ? "true" : "false",
                      roll_str,
                      pitch_str,
                      gyro_x_str,
                      gyro_y_str,
                      gyro_z_str,
                      vibe_str,
                      dist_str,
                      (unsigned long)g_mpu_status,
                      (unsigned long)(g_mpu_addr >> 1),
                      (unsigned long)g_mpu_error,
                      HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET ? "true" : "false",
                      HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == GPIO_PIN_SET ? "true" : "false");

    TransmitFormattedPacket(tx_buffer, sizeof(tx_buffer), length);
}

static void PrintTeleopToSwv(void)
{
    float reach = 0.0f;
    float lift = 0.0f;
    float twist = 0.0f;

#if TELEOP_DIAGNOSTIC_BUTTON_SCAN
    reach = 0.0f;
    lift = 0.0f;
    twist = 0.0f;
#else
    if (g_deadman_active && g_imu_ok) {
        reach = NormalizeAxis((g_pitch_deg - g_teleop_neutral_pitch_deg) * TELEOP_REACH_SIGN,
                              TELEOP_ANGLE_DEADZONE_DEG,
                              TELEOP_ANGLE_FULL_SCALE_DEG);
        lift = NormalizeAxis((g_roll_deg - g_teleop_neutral_roll_deg) * TELEOP_LIFT_SIGN,
                             TELEOP_ANGLE_DEADZONE_DEG,
                             TELEOP_ANGLE_FULL_SCALE_DEG);
        twist = GetTeleopTwistCommand();
    }
#endif

    printf("teleop session=%lu deadman=%d ",
           (unsigned long)g_teleop_session,
           g_deadman_active);
    PrintFixed3("reach", reach);
    printf(" ");
    PrintFixed3("lift", lift);
    printf(" ");
    PrintFixed3("twist", twist);
    printf(" grip_open=%d grip_close=%d led_seq=%lu\r\n",
           ReadButtonWithOptionalAlternate(TELEOP_GRIP_OPEN_GPIO_Port, TELEOP_GRIP_OPEN_Pin,
                                           TELEOP_GRIP_OPEN_ALT_ENABLED,
                                           TELEOP_GRIP_OPEN_ALT_GPIO_Port, TELEOP_GRIP_OPEN_ALT_Pin),
           ReadButton(TELEOP_GRIP_CLOSE_GPIO_Port, TELEOP_GRIP_CLOSE_Pin),
           (unsigned long)g_teleop_led_toggle_seq);
}

#if APP_MODE == APP_MODE_SENSOR_BRIDGE
static void SendSensorPacket(void)
{
    char roll_str[24];
    char pitch_str[24];
    char gyro_x_str[24];
    char gyro_y_str[24];
    char gyro_z_str[24];
    char vibe_str[24];
    char dist_str[24];
    char tx_buffer[256];
    int length;

    FormatFixedValue(roll_str, sizeof(roll_str), g_roll_deg, 1000U, 3U);
    FormatFixedValue(pitch_str, sizeof(pitch_str), g_pitch_deg, 1000U, 3U);
    FormatFixedValue(gyro_x_str, sizeof(gyro_x_str), g_gyro_x_dps, 1000U, 3U);
    FormatFixedValue(gyro_y_str, sizeof(gyro_y_str), g_gyro_y_dps, 1000U, 3U);
    FormatFixedValue(gyro_z_str, sizeof(gyro_z_str), g_gyro_z_dps, 1000U, 3U);
    FormatFixedValue(vibe_str, sizeof(vibe_str), g_vibe, 100U, 2U);
    FormatFixedValue(dist_str, sizeof(dist_str), g_distance_cm, 10U, 1U);

    length = snprintf(tx_buffer,
                      sizeof(tx_buffer),
                      "{\"type\":\"sensor\",\"ts_ms\":%lu,\"imu_ok\":%s,\"range_ok\":%s,"
                      "\"roll\":%s,\"pitch\":%s,\"gyro_x\":%s,\"gyro_y\":%s,\"gyro_z\":%s,"
                      "\"vibe\":%s,\"dist_cm\":%s}\r\n",
                      (unsigned long)HAL_GetTick(),
                      g_imu_ok ? "true" : "false",
                      g_range_ok ? "true" : "false",
                      roll_str,
                      pitch_str,
                      gyro_x_str,
                      gyro_y_str,
                      gyro_z_str,
                      vibe_str,
                      dist_str);

    TransmitFormattedPacket(tx_buffer, sizeof(tx_buffer), length);
}

static void PrintTelemetryToSwv(void)
{
    printf("MPU 0x%02X ", g_mpu_addr >> 1);
    PrintFixed3("roll", g_roll_deg);
    printf(" ");
    PrintFixed3("pitch", g_pitch_deg);
    printf(" ");
    PrintFixed3("gyro_x", g_gyro_x_dps);
    printf(" ");
    PrintFixed3("gyro_y", g_gyro_y_dps);
    printf(" ");
    PrintFixed3("gyro_z", g_gyro_z_dps);
    printf(" ");
    PrintFixed3("vibe", g_vibe);
    printf(" ");
    PrintFixed3("dist_cm", g_distance_cm);
    printf(" range_ok=%d\r\n", g_range_ok);
}
#endif

static HAL_StatusTypeDef CalibrateMpu6050(I2C_HandleTypeDef *hi2c)
{
    float sum_gx = 0.0f;
    float sum_gy = 0.0f;
    float sum_gz = 0.0f;
    float sum_roll_acc = 0.0f;
    float sum_pitch_acc = 0.0f;

    printf("Calibrating gyro: keep sensor still...\r\n");

    for (uint32_t i = 0; i < MPU_CALIBRATION_SAMPLES; ++i) {
        MPU6050_Sample sample = {0};
        HAL_StatusTypeDef ret = MPU6050_ReadSampleAt(hi2c, g_mpu_addr, &sample);
        if (ret != HAL_OK) {
            printf("Calibration read failed: ret=%d err=0x%08lX\r\n",
                   ret,
                   (unsigned long)HAL_I2C_GetError(hi2c));
            return ret;
        }

        sum_gx += sample.gx_dps;
        sum_gy += sample.gy_dps;
        sum_gz += sample.gz_dps;
        sum_roll_acc += sample.roll_acc;
        sum_pitch_acc += sample.pitch_acc;
        HAL_Delay(1000U / MPU_SAMPLE_RATE_HZ);
    }

    g_gyro_bias_x_dps = sum_gx / (float)MPU_CALIBRATION_SAMPLES;
    g_gyro_bias_y_dps = sum_gy / (float)MPU_CALIBRATION_SAMPLES;
    g_gyro_bias_z_dps = sum_gz / (float)MPU_CALIBRATION_SAMPLES;
    g_roll_deg = sum_roll_acc / (float)MPU_CALIBRATION_SAMPLES;
    g_pitch_deg = sum_pitch_acc / (float)MPU_CALIBRATION_SAMPLES;
    g_attitude_ready = 1;
    g_imu_ok = 1;

    PrintFixed3("gyro_bias_x_dps", g_gyro_bias_x_dps);
    printf(" ");
    PrintFixed3("gyro_bias_y_dps", g_gyro_bias_y_dps);
    printf(" ");
    PrintFixed3("gyro_bias_z_dps", g_gyro_bias_z_dps);
    printf("\r\n");
    PrintFixed3("initial_roll", g_roll_deg);
    printf(" ");
    PrintFixed3("initial_pitch", g_pitch_deg);
    printf("\r\n");

    return HAL_OK;
}

static HAL_StatusTypeDef ProcessMpu6050Sample(I2C_HandleTypeDef *hi2c)
{
    const float dt_sec = 1.0f / (float)MPU_SAMPLE_RATE_HZ;
    const float alpha = MPU_FILTER_TAU_SEC / (MPU_FILTER_TAU_SEC + dt_sec);
    MPU6050_Sample sample = {0};
    HAL_StatusTypeDef ret = MPU6050_ReadSampleAt(hi2c, g_mpu_addr, &sample);

    if (ret != HAL_OK) {
        g_imu_ok = 0;
        g_mpu_status = MPU_STATUS_READ_FAIL;
        g_mpu_error = HAL_I2C_GetError(hi2c);
        return ret;
    }

    g_imu_ok = 1;
    g_mpu_status = MPU_STATUS_READY;
    g_mpu_error = 0;
    g_gyro_x_dps = sample.gx_dps - g_gyro_bias_x_dps;
    g_gyro_y_dps = sample.gy_dps - g_gyro_bias_y_dps;
    g_gyro_z_dps = sample.gz_dps - g_gyro_bias_z_dps;
    g_vibe = sqrtf((g_gyro_x_dps * g_gyro_x_dps) +
                   (g_gyro_y_dps * g_gyro_y_dps) +
                   (g_gyro_z_dps * g_gyro_z_dps));

    UpdateTeleopTwistAngle(dt_sec);

    if (!g_attitude_ready) {
        g_roll_deg = sample.roll_acc;
        g_pitch_deg = sample.pitch_acc;
        g_attitude_ready = 1;
    } else {
        g_roll_deg = alpha * (g_roll_deg + g_gyro_x_dps * dt_sec) +
                     (1.0f - alpha) * sample.roll_acc;
        g_pitch_deg = alpha * (g_pitch_deg + g_gyro_y_dps * dt_sec) +
                      (1.0f - alpha) * sample.pitch_acc;
    }

#if APP_MODE == APP_MODE_TELEOP_REMOTE
    /* Teleop packets are emitted from the main loop so the UART heartbeat
       continues even when IMU init/readout is unhealthy. */
#else
    g_tx_divider++;
    if (g_tx_divider >= (MPU_SAMPLE_RATE_HZ / SENSOR_TX_RATE_HZ)) {
        g_tx_divider = 0U;
        SendSensorPacket();
        PrintTelemetryToSwv();
    }
#endif

    return HAL_OK;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  RecoverI2c2Bus();
  MX_I2C2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  EnableCycleCounter();
  HAL_Delay(200);

  printf("\r\nMotionBrainSensor boot\r\n");
#if APP_MODE == APP_MODE_TELEOP_REMOTE
  printf("Mode: handheld teleop remote\r\n");
  printf("UART teleop stream: USART2 TX=PD5 (Arduino D1) @ 115200\r\n");
  printf("Embedded safety telemetry: imu/range/vibe/dist fields included in teleop frames\r\n");
  printf("HC-SR04 mapping: TRIG=PD4 (Arduino D2), ECHO=PC8 (Arduino D3)\r\n");
#if TELEOP_DIAGNOSTIC_BUTTON_SCAN
  printf("Diagnostic scan mode: session=D4/D5/D6/D7/D8/D9/D10/D11, sequence=D12/D13/A0/A1/A2/A3/A4/A5, led_seq=PA0/PA4/PB0 bits\r\n");
#endif
  printf("Provisional button map: deadman=PE4(D10) led=PB4(D9) grip_open=PE2(D13) grip_close=PE6(D11)\r\n");
  InitTeleopInputs();
#else
  printf("Board mapping: D15=PB10 (I2C2_SCL), D14=PC12 (I2C2_SDA)\r\n");
  printf("UART sensor stream: USART2 TX=PD5 (Arduino D1) @ 115200\r\n");
  printf("HC-SR04 mapping: TRIG=PD4 (Arduino D2), ECHO=PC8 (Arduino D3)\r\n");
#endif

  ProbeMpu6050(&hi2c2);
  if (!g_mpu_ready) {
    printf("MPU-6050 not detected on I2C2.\r\n");
  }
  if (g_mpu_ready) {
    if (CalibrateMpu6050(&hi2c2) != HAL_OK) {
      g_mpu_ready = 0;
      g_imu_ok = 0;
      g_mpu_status = MPU_STATUS_CALIB_FAIL;
      g_mpu_error = HAL_I2C_GetError(&hi2c2);
      printf("MPU-6050 calibration failed.\r\n");
    }
  }

  printf("Probe done. Check SWV and USART2 stream.\r\n");
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if SAFETY_TELEMETRY_ENABLED
    if ((HAL_GetTick() - g_last_hcsr04_trigger_ms) >= HCSR04_TRIGGER_INTERVAL_MS && !g_hcsr04_echo_pending) {
      TriggerHcsr04();
      g_last_hcsr04_trigger_ms = HAL_GetTick();
    }

    UpdateRangeMeasurement();
#endif

    if (g_mpu_ready && g_sample_due_count > 0U) {
      uint32_t pending_samples;

      __disable_irq();
      pending_samples = g_sample_due_count;
      g_sample_due_count = 0U;
      __enable_irq();

      while (pending_samples-- > 0U) {
        HAL_StatusTypeDef ret = ProcessMpu6050Sample(&hi2c2);
        if (ret != HAL_OK) {
          printf("MPU 0x%02X read failed: ret=%d err=0x%08lX\r\n",
                 g_mpu_addr >> 1,
                 ret,
                 (unsigned long)HAL_I2C_GetError(&hi2c2));
          break;
        }
      }
    }

#if APP_MODE == APP_MODE_TELEOP_REMOTE
    if ((HAL_GetTick() - g_last_teleop_tx_ms) >= (1000U / TELEOP_TX_RATE_HZ)) {
      g_last_teleop_tx_ms = HAL_GetTick();
      SendTeleopPacket();
      PrintTeleopToSwv();
    }
#endif
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3) {
    g_sample_due_count++;
  }
}

#if SAFETY_TELEMETRY_ENABLED && HCSR04_ENABLED
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == HCSR04_ECHO_Pin) {
        uint32_t now_us = Micros();

        if (HAL_GPIO_ReadPin(HCSR04_ECHO_GPIO_Port, HCSR04_ECHO_Pin) == GPIO_PIN_SET) {
            g_hcsr04_rise_us = now_us;
            g_hcsr04_waiting_for_fall = 1U;
        } else if (g_hcsr04_waiting_for_fall) {
            g_hcsr04_echo_width_us = now_us - g_hcsr04_rise_us;
            g_hcsr04_waiting_for_fall = 0U;
            g_hcsr04_echo_pending = 0U;
            g_hcsr04_measurement_ready = 1U;
        }
    }
}
#endif

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
