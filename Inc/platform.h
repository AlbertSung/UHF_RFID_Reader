/**
  ******************************************************************************
  * @file    platform.h
  * @author  Albert
  * @version V1.0.0
  * @date    03-June-2017
  * @brief   Header for platform module
  ******************************************************************************
	*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __B2E_PLATFORM_H
#define __B2E_PLATFORM_H

#ifdef __cplusplus
 extern "C" {
#endif


// Compiler options
#define EEPROM_WRITE_DMA
#define EEPROM_READ_DMA

#define BUZZER_BY_PWM


/* Includes ------------------------------------------------------------------*/
#include "ams_types.h"
#include "stm32l1xx_hal.h"
#include "usbd_def.h"
#include "display_menu.h"
#include "oled_driver.h"


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
extern u16 ulTuneValue;

/* Exported functions --------------------------------------------------------*/
void UHF_EN(GPIO_PinState ioState);
void UHF_NCS(GPIO_PinState ioState);
void UHF_PAG(u8 bIsPowerOn);
void OLED_RST(GPIO_PinState ioState);

void delay_us(volatile u32 time);
	 
bool is_timeout_ms(uint32_t top, uint32_t lastcount);

s8 SPITxRx(const u8* pbTxData, u8* pbRxData, u16 uLength);

s8 um900SetRFTxPower(s8 cPower);

void DeviceInit(void);
void DeviceDeInit(void);
void LP_GPIO_Init(void);
void DefaultInit(void);
void ScreenInit(void);

u16 uart1WriteBytes(const u8* pbBuffer, u16 uBufferLength);
u16 uart2WriteBytes(const u8* pbBuffer, u16 uBufferLength);

void MX_RTC_Init(void);
void RTC_StartTimeSet(u8* pstartTime);
void RTC_TimeDateShow(uint8_t* showtime, uint8_t* showdate);
void RTC_TimeGet(RTC_TimeTypeDef* ptimestructureget);
void RTC_DateGet(RTC_DateTypeDef* pdatestructureget);

void OLED_WriteCmd(uint8_t bTxData);
void OLED_WriteData(uint8_t bTxData);

s8 EEPROM_ReadData(uint8_t* pbBuffer, uint16_t uMemoryAddress, uint16_t* puLength);
s8 EEPROM_WriteData(uint8_t* pbBuffer, uint16_t uMemoryAddress, uint16_t* puLength);

s8 FLASH_EEPROM_ReadBytes(uint8_t* pbBuffer, uint32_t ulOffsetAddress, uint16_t* puLength);
s8 FLASH_EEPROM_WriteBytes(uint8_t* pbBuffer, uint32_t ulOffsetAddress, uint16_t* puLength);
u8 FLASH_EEPROM_IsEmpty(void);

void ADC_DataCollect(void);
void ADC_DataReCollect(void);
u16 ADC_AverageGet(void);
u8 ADC_PercentGet(void);


/* Exported variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd;
extern I2C_HandleTypeDef I2c2Handle;
extern UART_HandleTypeDef Uart1Handle, Uart2Handle;
extern USBD_HandleTypeDef UsbdHandle;
extern ADC_HandleTypeDef 	Adc1Handle;
extern RTC_HandleTypeDef RtcHandle;
extern TIM_HandleTypeDef Tim3Handle;



/* Platform configuration ----------------------------------------------------*/
#define RTC_CLOCK_SOURCE_LSE


/* Definition for USARTx clock resources */
#define USART1_CLK_ENABLE()              __HAL_RCC_USART1_CLK_ENABLE();
#define USART2_CLK_ENABLE()              __HAL_RCC_USART2_CLK_ENABLE();
#define USART1_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define USART1_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define USART2_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USART2_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define USART1_FORCE_RESET()             __HAL_RCC_USART1_FORCE_RESET()
#define USART1_RELEASE_RESET()           __HAL_RCC_USART1_RELEASE_RESET()
#define USART2_FORCE_RESET()             __HAL_RCC_USART2_FORCE_RESET()
#define USART2_RELEASE_RESET()           __HAL_RCC_USART2_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USART1_TX_PIN                    GPIO_PIN_6
#define USART1_TX_GPIO_PORT              GPIOB
#define USART1_TX_AF                     GPIO_AF7_USART1
#define USART1_RX_PIN                    GPIO_PIN_7
#define USART1_RX_GPIO_PORT              GPIOB
#define USART1_RX_AF                     GPIO_AF7_USART1
#define USART2_TX_PIN                    GPIO_PIN_2
#define USART2_TX_GPIO_PORT              GPIOA
#define USART2_TX_AF                     GPIO_AF7_USART2
#define USART2_RX_PIN                    GPIO_PIN_3
#define USART2_RX_GPIO_PORT              GPIOA
#define USART2_RX_AF                     GPIO_AF7_USART2

/* Definition for SPIx clock resources */
#define SPI2_CLK_ENABLE()                __HAL_RCC_SPI2_CLK_ENABLE()
#define SPI2_SCK_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPI2_MISO_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPI2_MOSI_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()

#define SPI2_FORCE_RESET()               __HAL_RCC_SPI2_FORCE_RESET()
#define SPI2_RELEASE_RESET()             __HAL_RCC_SPI2_RELEASE_RESET()

/* Definition for SPIx Pins */
#define SPI2_SCK_PIN                     GPIO_PIN_13
#define SPI2_SCK_GPIO_PORT               GPIOB
#define SPI2_SCK_AF                      GPIO_AF5_SPI2
#define SPI2_MISO_PIN                    GPIO_PIN_14
#define SPI2_MISO_GPIO_PORT              GPIOB
#define SPI2_MISO_AF                     GPIO_AF5_SPI2
#define SPI2_MOSI_PIN                    GPIO_PIN_15
#define SPI2_MOSI_GPIO_PORT              GPIOB
#define SPI2_MOSI_AF                     GPIO_AF5_SPI2

/* Definition for I2Cx clock resources */
#define I2C2_CLK_ENABLE()               __HAL_RCC_I2C2_CLK_ENABLE()
#define I2C2_DMA_CLK_ENABLE()           __HAL_RCC_DMA1_CLK_ENABLE()
#define I2C2_SDA_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define I2C2_SCL_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()

#define I2C2_FORCE_RESET()              __HAL_RCC_I2C2_FORCE_RESET()
#define I2C2_RELEASE_RESET()            __HAL_RCC_I2C2_RELEASE_RESET()

/* Definition for I2Cx Pins */
#define I2C2_SCL_PIN                    GPIO_PIN_10
#define I2C2_SCL_GPIO_PORT              GPIOB
#define I2C2_SDA_PIN                    GPIO_PIN_11
#define I2C2_SDA_GPIO_PORT              GPIOB
#define I2C2_SCL_SDA_AF                 GPIO_AF4_I2C2

/* Definition for I2Cx's DMA */
#define I2C2_DMA_INSTANCE_TX            DMA1_Channel4
#define I2C2_DMA_INSTANCE_RX            DMA1_Channel5

/* Definition for I2Cx's NVIC */
#define I2C2_DMA_TX_IRQn                DMA1_Channel4_IRQn
#define I2C2_DMA_RX_IRQn                DMA1_Channel5_IRQn
#define I2C2_DMA_TX_IRQHandler          DMA1_Channel4_IRQHandler
#define I2C2_DMA_RX_IRQHandler          DMA1_Channel5_IRQHandler

/* Definition of ADCx clock resources */
#define ADC1_CLK_ENABLE()               __HAL_RCC_ADC1_CLK_ENABLE()
#define ADC1_CHANNELa_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()

#define ADC1_FORCE_RESET()              __HAL_RCC_ADC1_FORCE_RESET()
#define ADC1_RELEASE_RESET()            __HAL_RCC_ADC1_RELEASE_RESET()

/* Definition of ADCx channels pins */
#define ADC1_CHANNELa_PIN               GPIO_PIN_5
#define ADC1_CHANNELa_GPIO_PORT         GPIOA

/* Definition of RTC clock resource */
#ifdef RTC_CLOCK_SOURCE_LSI
#define RTC_ASYNCH_PREDIV    0x7F
#define RTC_SYNCH_PREDIV     0x0120
#endif

#ifdef RTC_CLOCK_SOURCE_LSE
#define RTC_ASYNCH_PREDIV  0x7F
#define RTC_SYNCH_PREDIV   0x00FF
#endif

/* Definition for TIMx clock resources */
#define TIM3_CLK_ENABLE()								__HAL_RCC_TIM3_CLK_ENABLE()

#define TIM3_FORCE_RESET()							__HAL_RCC_TIM3_FORCE_RESET()
#define TIM3_RELEASE_RESET()            __HAL_RCC_TIM3_RELEASE_RESET()

// Define this to 1 to enable AS3980 support.
#ifdef _UHF_CHIP_AS3980_
#define RUN_ON_AS3980                   1
#else
#define RUN_ON_AS3980                   0
#endif

// Define this to 1 to enable AS3993 support.
#if RUN_ON_AS3980
#define RUN_ON_AS3993                   0
#else
#define RUN_ON_AS3993                   1
#endif

// Define EXT_PA to use external PA
#define EXT_PA

// Definition for high and low
#define OUT_HIGH		GPIO_PIN_SET
#define OUT_LOW     GPIO_PIN_RESET

#define UHF_ENEXTIRQ()        //*_INT1IE = 1;	    // Macro for enable external IRQ
#define UHF_DISEXTIRQ()       //*_INT1IE = 0;  	  // Macro for disable external IRQ
#define UHF_CLREXTIRQ()       //*_INT1IF = 0;    	// Macro for clearing external IRQ flag

#define UHF_NCS_SELECT()			UHF_NCS(OUT_LOW)		// Macro for activating AS3993 for SPI communication
#define UHF_NCS_DESELECT()  	UHF_NCS(OUT_HIGH)		// Macro for deactivating AS3993 for SPI communication

#define delay_ms(Delay)				HAL_Delay((u32)Delay)

#define getSysTicks()         HAL_GetTick()
#define getSysMilliseconds()  HAL_GetTick()

#define IsTimeout_ms(TICKS, MILLISECONDS)               is_timeout_ms(TICKS, MILLISECONDS)


// 
#define AS3993_IRQ_PIN				GPIO_PIN_1
#define SYS_EN_PIN						GPIO_PIN_1
#define LEFT_KEY_PIN					GPIO_PIN_9
#define RIGHT_KEY_PIN					GPIO_PIN_10
#define TRIG_KEY_PIN					GPIO_PIN_15
#define USB_CHARGE_PIN				GPIO_PIN_0
#define BUZZ_OUT_PIN					GPIO_PIN_4
#define BARC_PWR_PIN					GPIO_PIN_6
#define VIBR_OUT_PIN					GPIO_PIN_7
#define LED1_GRN_PIN					GPIO_PIN_14
#define LED2_RED_PIN					GPIO_PIN_13


// 
#define DEV_BADURATE_115200BPS      0xFF

#define DEV_AUTO_MODE_AUTO          0x00
#define DEV_AUTO_MODE_COMMAND       0x01
#define DEV_AUTO_MODE_CHECK         0x02
#define DEV_AUTO_MODE_EVENT         0x04
#define DEV_AUTO_MODE_DEFAULT_AUTO  0xFF

#define DEV_INTERFACE_WIEGAND       0x00
#define DEV_INTERFACE_MULTI_RS485   0x01
#define DEV_INTERFACE_RS232         0xFF

#define DEV_SCAN_MODE_ALWAYS        0xFF

#define DEV_TIME_TO_REMOVE_NEVER    0xFF

#define DEV_REPEAT_TIME_NEVER       0xFF

#define PA_GAIN08_SEL                 0x01
#define PA_GAIN16_SEL                 0x02

#define RADIO_TEST_MODE_CONTINUOUS_WAVE             0x00
#define RADIO_TEST_MODE_MODULATION                  0x01
#define RADIO_TEST_MODE_HOPPING                     0x02
#define RADIO_TEST_MODE_NONE                        0x03
//#define RADIO_TEST_MODE_RX_MODE                   0x04

/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the I2C communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */   
#define I2C_XFER_TIMEOUT_MAX    		300//800
/* Maximum number of trials for HAL_I2C_IsDeviceReady() function */
#define EEPROM_MAX_TRIALS       		300//800

#define EEPROM_ADDRESS							0xA2			// 1010 0 A1 A0 R/W
#define EEPROM_RWBIT_SET						0x01			// Set R/W bit for Read operation
#define EEPROM_PAGESIZE							128				// RF EEPROM ANT7-M24LR used

#define MAX_ADC_BUF_LEN							10

#define VIBR_BOOT_TIME_MS						100



#ifdef __cplusplus
}
#endif

#endif /* __B2E_PLATFORM_H */


