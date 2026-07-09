/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define RMII_REF_CLK_Pin GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port GPIOA
#define RMII_MDIO_Pin GPIO_PIN_2
#define RMII_MDIO_GPIO_Port GPIOA
#define RMII_CRS_DV_Pin GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define RMII_TXD1_Pin GPIO_PIN_13
#define RMII_TXD1_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG
#define RMII_TXD0_Pin GPIO_PIN_13
#define RMII_TXD0_GPIO_Port GPIOG
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* maneuver 값 */
#define MANEUVER_STRAIGHT          0U
#define MANEUVER_RIGHT_TURN        1U
#define MANEUVER_LEFT_TURN_UNPROT  2U
#define MANEUVER_LEFT_TURN_PROT    3U


/* candidateVehicle.type 값 */
#define CAND_NONE                  0U

#define CAND_RT_LEFT_STRAIGHT      1U  /* 자차 우회전 vs 좌측 직진 */
#define CAND_RT_OPP_LEFT           2U  /* 자차 우회전 vs 대향 보호 좌회전 */

#define CAND_LT_OPP_STRAIGHT       4U  /* 자차 비보호 좌회전 vs 대향 직진 */
#define CAND_LT_OPP_RIGHT          8U  /* 자차 비보호 좌회전 vs 대향 우회전 */

typedef struct
{
    uint8_t msg_id;      // 4bit
    uint16_t timestamp;	 // 12bit
    uint8_t updateMask;  // 8bit
} CAN_Header_t;

// [ 어플리케이션 전역 상태 구조체 (Direct 사용) ]
typedef struct {
    uint16_t x;              // 10 bit
    uint16_t y;              // 11 bit
    uint8_t speed;           // 8 bit
    uint16_t heading;        // 9 bit
    uint16_t timestamp;      // 12bit
} EgoVehicle;

typedef struct {
    uint8_t type;
    uint16_t cz_x;
    uint16_t cz_y;
    uint16_t x;
    uint16_t y;
    uint8_t speed;
    uint64_t timestamp_ms;
    uint64_t received_timestamp;
} CandidateVehicle;

typedef struct {
    uint8_t color;
    uint8_t time_left;
    uint16_t cz_x;
    uint16_t cz_y;
} TrafficLight;

// ========================================================
// Global Extern Variables
// ========================================================
extern volatile uint8_t canRxFlag;
extern volatile uint16_t rx_id;
extern CAN_Header_t rx_header;

// 전역 변수로 관리되는 상태 객체들
extern EgoVehicle ego;
extern CandidateVehicle candidateVehicle;
extern TrafficLight tl;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
