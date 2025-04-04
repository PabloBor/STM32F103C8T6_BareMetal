/*
 *  stm32f103xx.h
 *
 *  Created on: 01 march 2023
 *      Author: pablo borboa
 */

 #ifndef INC_STM32F103XX_H_
 #define INC_STM32F103XX_H_
 
 #include <stddef.h>
 #include <stdint.h>
 
 #define __vo volatile
 
 #define __weak __attribute__((weak))
 
//  /**********************************START:Processor Specific Details **********************************/
//  /*
//   * ARM Cortex Mx Processor NVIC ISERx register Addresses
//   */
 
//  #define NVIC_ISER0          ( (__vo uint32_t*)0xE000E100 )
//  #define NVIC_ISER1          ( (__vo uint32_t*)0xE000E104 )
//  #define NVIC_ISER2          ( (__vo uint32_t*)0xE000E108 )
//  #define NVIC_ISER3          ( (__vo uint32_t*)0xE000E10c )
 
 
//  /*
//   * ARM Cortex Mx Processor NVIC ICERx register Addresses
//   */
//  #define NVIC_ICER0             ((__vo uint32_t*)0XE000E180)
//  #define NVIC_ICER1            ((__vo uint32_t*)0XE000E184)
//  #define NVIC_ICER2          ((__vo uint32_t*)0XE000E188)
//  #define NVIC_ICER3            ((__vo uint32_t*)0XE000E18C)
 
 
//  /*
//   * ARM Cortex Mx Processor Priority Register Address Calculation
//   */
//  #define NVIC_PR_BASE_ADDR     ((__vo uint32_t*)0xE000E400)
 
//  /*
//   * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
//   */
//  #define NO_PR_BITS_IMPLEMENTED  4
 
 
//  /*
//   *     base addresses of flash and SRAM memories
//   */
 
//  #define    FALSH_BASEADDR                    0X08000000U
//  #define    SRAM1_BASEADDR                    0X20000000U
//  #define    SRAM2_BASEADDR                    0x2001C000U
//  #define    ROM                                0x1FFF0000U
//  #define    SRAM                                SRAM_BASEADDR
 
 /*
  *     AHBx and APBx Bus Peripheral base addresses
  */
 
 #define    APB1_BASE       0X40000000U
 #define    APB2_BASE       0X40010000U
 #define    AHB_BASE        0X40018000U
 
 /*
  *     Base addresses of peripherals which are hanging on APB1 bus
  */

                                        /*APB1 BASE addr + Offset*/
 #define    TIM2_BASEADRR                (APB1_BASE +    0x0000)
 #define    TIM3_BASEADRR                (APB1_BASE +    0x0400)
 #define    TIM4_BASEADRR                (APB1_BASE +    0x0800)
//  #define    TIM5_CEC_BASEADRR            (APB1_BASE +    0x0C00)
//  #define    TIM6_BASEADRR                (APB1_BASE +    0x1000)
//  #define    TIM7_BASEADRR                (APB1_BASE +    0x1400)
//  #define    TIM12_BASEADRR               (APB1_BASE +    0x1800)
//  #define    TIM13_BASEADRR               (APB1_BASE +    0x1C00)
//  #define    TIM14_BASEADRR               (APB1_BASE +    0x2000)
 #define    RTC_BASEADRR                 (APB1_BASE +    0x2800)
 #define    WWDG_BASEADRR                (APB1_BASE +    0x2C00)
 #define    IWDG_BASEADRR                (APB1_BASE +    0x3000)
 #define    SPI2_I2S_BASEADRR            (APB1_BASE +    0x3800)
//  #define    SPI3_I2S_BASEADRR            (APB1_BASE +    0x3C00)
 #define    USART2_BASEADRR              (APB1_BASE +    0x4400)
 #define    USART3_BASEADRR              (APB1_BASE +    0x4800)
//  #define    UART4_BASEADRR               (APB1_BASE +    0x4C00)
//  #define    UART5_BASEADRR               (APB1_BASE +    0x5000)
 #define    I2C1_BASEADRR                (APB1_BASE +    0x5400)
 #define    I2C2_BASEADRR                (APB1_BASE +    0x5800)
 #define    USB_BASEADRR                 (APB1_BASE +    0x5C00)
 #define    USB_CAN_SRAM_BASEADRR        (APB1_BASE +    0x6000)
 #define    bxCAN2_BASEADRR              (APB1_BASE +    0x6800)
//  #define    bxCAN1_BASEADRR              (APB1_BASE +    0x6400)
 #define    BKP_BASEADRR                 (APB1_BASE +    0x6C00)
 #define    PWR_BASEADRR                 (APB1_BASE +    0x7000)
//  #define    DAC_BASEADRR                 (APB1_BASE +    0x7400)
 

 /*
  *     Base addresses of peripherals which are hanging on APB2 bus
  */
 
                                      /*APB2 start addr   Offset*/
 #define    AFIO_BASEADRR                (APB2_BASE +    0x0000)
 #define    EXTI_BASEADRR                (APB2_BASE +    0x0400)
 #define    GPIOA_BASEADRR               (APB2_BASE +    0x0800)
 #define    GPIOB_BASEADRR               (APB2_BASE +    0x0C00)
 #define    GPIOC_BASEADRR               (APB2_BASE +    0x1000)
 #define    GPIOD_BASEADRR               (APB2_BASE +    0x1400)
 #define    GPIOE_BASEADRR               (APB2_BASE +    0x1800)
//  #define    GPIOF_BASEADRR               (APB2_BASE +    0x1C00)
//  #define    GPIOG_BASEADRR               (APB2_BASE +    0x2000)
 #define    ADC1_BASEADRR                (APB2_BASE +    0x2400)
 #define    ADC2_BASEADRR                (APB2_BASE +    0x2800)
 #define    TIM1_BASEADRR                (APB2_BASE +    0x2C00)
 #define    SPI1_BASEADRR                (APB2_BASE +    0x3000)
//  #define    TIM8_BASEADRR                (APB2_BASE +    0x3400)
 #define    USART1_BASEADRR              (APB2_BASE +    0x3800)
//  #define    ADC3_BASEADRR                (APB2_BASE +    0x3C00)
//  #define    TIM9_BASEADRR                (APB2_BASE +    0x4C00)
//  #define    TIM10_BASEADRR               (APB2_BASE +    0x5000)
//  #define    TIM11_BASEADRR               (APB2_BASE +    0x5400)

 
 /*
  *     Base addresses of peripherals which are hanging on AHB bus
  */
 
                                         /*AHB1 start addr   Offset*/
//  #define    SDIO_BASEADRR                (AHB_BASE +    0x0000)
 #define    DMA1_BASEADRR                (AHB_BASE +    0x8000)
//  #define    DMA2_BASEADRR                (AHB_BASE +    0x8400)
 #define    RCC_BASEADRR                 (AHB_BASE +    0x9000)
 #define    FLASH_INTERF_REG_BASEADRR    (AHB_BASE +    0xA000)
 #define    CRC_BASEADRR                 (AHB_BASE +    0xB000)
//  #define    ETHERNET_BASEADRR            (AHB_BASE +    0x10000)
//  #define    USB_OTG_FS_BASEADRR          (AHB_BASE +    0xFFE8000)
//  #define    FSMC_BASEADRR                (AHB_BASE +    0X5FFE 8000)
 
 
 /****************************** Peripheral register definition structures ******************************/
 /*
  * Note: Registers of aa peripheral are specific to MCU
  * e.g: Number of Registers of SPI peripheral of STM324x family of MCUs maybe diferent(more or less)
  * Please check your Device RM
  */



/////////////////////////////////////////////////////////////////////
//      PERIPHERAL REGISTER definition structure for RCC
/////////////////////////////////////////////////////////////////////
 typedef struct
 {
   __vo uint32_t CR;       // 0x00  Clock control register
   __vo uint32_t CFGR;     // 0x04  Clock configuration register
   __vo uint32_t CIR;      // 0x08  Clock interrupt register
   __vo uint32_t APB2RSTR; // 0x0C  APB2 peripheral reset register
   __vo uint32_t APB1RSTR; // 0x10  APB1 peripheral reset register
   __vo uint32_t AHBENR;   // 0x14  AHB Peripheral Clock enable register
   __vo uint32_t APB2ENR;  // 0x18  APB2 peripheral clock enable register
   __vo uint32_t APB1ENR;  // 0x1C  APB1 peripheral clock enable register
   __vo uint32_t BDCR;     // 0x20  Backup domain control register
   __vo uint32_t CSR;      // 0x24  Control/status register
 } RCC_RegDef_t;
 

/////////////////////////////////////////////////////////////////////
//      PERIPHERAL REGISTER definition structure for GPIO
/////////////////////////////////////////////////////////////////////
  typedef struct
 {
     __vo uint32_t CRL;  // 0x00  Port configuration register low
     __vo uint32_t CRH;  // 0x04  Port configuration register high
     __vo uint32_t IDR;  // 0x08  port input data register
     __vo uint32_t ODR;  // 0x0C  port output data register
     __vo uint32_t BSRR; // 0x10  port bit set/reset register
     __vo uint32_t BRR;  // 0x14  Port bit reset register
     __vo uint32_t LCKR; // 0x18  port configuration lock register
 }GPIO_RegDef_t;

// TODO:  add the AFIO registers 

/////////////////////////////////////////////////////////////////////
//      PERIPHERAL REGISTER definition structure for EXTI
/////////////////////////////////////////////////////////////////////
 typedef struct
 {
     __vo uint32_t IMR;   // 0x00  Interrupt mask register
     __vo uint32_t EMR;   // 0x04  Event mask register
     __vo uint32_t RTSR;  // 0x08  Rising trigger selection register
     __vo uint32_t FTSR;  // 0x0C  Falling trigger selection register
     __vo uint32_t SWIER; // 0x10  Software interrupt event register
     __vo uint32_t PR;    // 0x14  Pending register
 
 }EXTI_RegDef_t;
 

/////////////////////////////////////////////////////////////////////
//      PERIPHERAL REGISTER definition structure for SPI/I2S
/////////////////////////////////////////////////////////////////////
 typedef struct
 {
     __vo uint32_t CR1;     // 0x00  SPI control register 1
     __vo uint32_t CR2;     // 0x04  SPI control register 2
     __vo uint32_t SR;      // 0x08  SPI status register
     __vo uint32_t DR;      // 0x0C  SPI data register
     __vo uint32_t CRCPR;   // 0x10  SPI CRC polynomial register
     __vo uint32_t RXCRCR;  // 0x14  SPI RX CRC register
     __vo uint32_t TXCRCR;  // 0x18  SPI TX CRC register
     __vo uint32_t I2SCFGR; // 0x1C  SPI_I2S configuration register
     __vo uint32_t I2SPR;   // 0x20  SPI_I2S prescaler register
 } SPI_RegDef_t;
 
 
//  /*
//   * PERIPHERAL REGISTER definition structure for SYSCFG
//   */
//  typedef struct
//  {
//      __vo uint32_t MEMRMP;        /*Address offset: 0x00      */
//      __vo uint32_t PMC;            /*Address offset: 0x04      */
//      __vo uint32_t EXTICR[4];    /*Address offset: 0x08-0x14 */
//      uint32_t      RESERVED1[2];    /*RESERVED          0x18-0x1C */
//      __vo uint32_t CMPCR;        /*Address offset: 0x20      */
//      uint32_t      RESERVED2[2];    /*RESERVED          0x24-0x28 */
//      __vo uint32_t CFGR;            /*Address offset: 0x2C       */
//  } SYSCFG_RegDef_t;
 
 
 /*
  * PERIPHERAL REGISTER definition structure for I2C
  */
 typedef struct
 {
   __vo uint32_t CR1;     // 0x00  
   __vo uint32_t CR2;     // 0x04  
   __vo uint32_t OAR1;    // 0x08  
   __vo uint32_t OAR2;    // 0x0C  
   __vo uint32_t DR;      // 0x10  
   __vo uint32_t SR1;     // 0x14  
   __vo uint32_t SR2;     // 0x18  
   __vo uint32_t CCR;     // 0x1C  
   __vo uint32_t TRISE;   // 0x20  
   __vo uint32_t FLTR;    // 0x24  
 }I2C_RegDef_t;
 
 /*
  * PERIPHERAL REGISTER definition structure for USART
  */
 typedef struct
 {
     __vo uint32_t SR;            /*Address offset: 0x00 */
     __vo uint32_t DR;            /*Address offset: 0x04 */
     __vo uint32_t BRR;            /*Address offset: 0x08 */
     __vo uint32_t CR1;            /*Address offset: 0x0C */
     __vo uint32_t CR2;            /*Address offset: 0x10 */
     __vo uint32_t CR3;            /*Address offset: 0x14 */
     __vo uint32_t GTPR;            /*Address offset: 0x18 */
 } USART_RegDef_t;
 
 
 /*
  *  SPECIFIC DEFINITIONS OF PERIPHERAL REGISTERS (Base addresses of peripherals assigned to xxx_RedDef_t )
  */
 
 #define GPIOA        ((GPIO_RegDef_t*) GPIOA_BASEADRR)
 #define GPIOB        ((GPIO_RegDef_t*) GPIOB_BASEADRR)
 #define GPIOC        ((GPIO_RegDef_t*) GPIOC_BASEADRR)
 #define GPIOD        ((GPIO_RegDef_t*) GPIOD_BASEADRR)
 #define GPIOE        ((GPIO_RegDef_t*) GPIOE_BASEADRR)
 #define GPIOF        ((GPIO_RegDef_t*) GPIOF_BASEADRR)
 #define GPIOG        ((GPIO_RegDef_t*) GPIOG_BASEADRR)
 #define GPIOH        ((GPIO_RegDef_t*) GPIOH_BASEADRR)
 
 #define RCC            ((RCC_RegDef_t*)    RCC_BASEADRR)
 #define EXTI        ((EXTI_RegDef_t*)    EXTI_BASEADRR)
 #define SYSCFG        ((SYSCFG_RegDef_t*)    SYSCFG_BASEADRR)
 
 #define SPI1          ((SPI_RegDef_t*) SPI1_BASEADRR)
 #define SPI2          ((SPI_RegDef_t*) SPI2_BASEADRR)
 #define SPI3          ((SPI_RegDef_t*) SPI3_BASEADRR)
 #define SPI4          ((SPI_RegDef_t*) SPI4_BASEADRR)
 
 #define I2C1          ((I2C_RegDef_t*) I2C1_BASEADRR)
 #define I2C2          ((I2C_RegDef_t*) I2C2_BASEADRR)
 #define I2C3          ((I2C_RegDef_t*) I2C3_BASEADRR)
 
 #define USART1      ((USART_RegDef_t*) USART1_BASEADRR)
 #define USART2      ((USART_RegDef_t*) USART2_BASEADRR)
 #define USART3      ((USART_RegDef_t*) USART3_BASEADRR)
 #define UART4          ((USART_RegDef_t*) UART4_BASEADRR)
 #define UART5          ((USART_RegDef_t*) UART5_BASEADRR)
 #define USART6      ((USART_RegDef_t*) USART6_BASEADRR)
 
 
 /*************************************************************
  *                  CLOCK ENABLE AND DISABLE MACROS
  *************************************************************/
 /*
  * Clock Enable and Disable Macros for GPIOx peripherals
  */
 
 #define GPIOA_PCLK_EN()    ( RCC->AHB1ENR |= (1 << 0) )
 #define GPIOB_PCLK_EN()    ( RCC->AHB1ENR |= (1 << 1) )
 #define GPIOC_PCLK_EN()    ( RCC->AHB1ENR |= (1 << 2) )
 #define GPIOD_PCLK_EN()    ( RCC->AHB1ENR |= (1 << 3) )
 #define GPIOE_PCLK_EN()    ( RCC->AHB1ENR |= (1 << 4) )
 #define GPIOF_PCLK_EN()    ( RCC->AHB1ENR |= (1 << 5) )
 #define GPIOG_PCLK_EN()    ( RCC->AHB1ENR |= (1 << 6) )
 #define GPIOH_PCLK_EN()    ( RCC->AHB1ENR |= (1 << 7) )
 
 #define GPIOA_PCLK_DI()    ( RCC->AHB1ENR &= ~(1 << 0) )
 #define GPIOB_PCLK_DI()    ( RCC->AHB1ENR &= ~(1 << 1) )
 #define GPIOC_PCLK_DI()    ( RCC->AHB1ENR &= ~(1 << 2) )
 #define GPIOD_PCLK_DI()    ( RCC->AHB1ENR &= ~(1 << 3) )
 #define GPIOE_PCLK_DI()    ( RCC->AHB1ENR &= ~(1 << 4) )
 #define GPIOF_PCLK_DI()    ( RCC->AHB1ENR &= ~(1 << 5) )
 #define GPIOG_PCLK_DI()    ( RCC->AHB1ENR &= ~(1 << 6) )
 #define GPIOH_PCLK_DI()    ( RCC->AHB1ENR &= ~(1 << 7) )
 
 
 /*
  * Clock Enable and Disable Macros for I2Cx peripherals
  */
 
 #define I2C1_PCLK_EN()    ( RCC->APB1ENR |= (1 << 21) )
 #define I2C2_PCLK_EN()    ( RCC->APB1ENR |= (1 << 22) )
 #define I2C3_PCLK_EN()    ( RCC->APB1ENR |= (1 << 23) )
 
 #define I2C1_PCLK_DI()    ( RCC->APB1ENR &= ~(1 << 21) )
 #define I2C2_PCLK_DI()    ( RCC->APB1ENR &= ~(1 << 22) )
 #define I2C3_PCLK_DI()    ( RCC->APB1ENR &= ~(1 << 23) )
 
 
 /*
  * Clock Enable and Disable Macros for SPIx peripherals
  */
 
 #define SPI1_PCLK_EN()    ( RCC->APB2ENR |= (1 << 12) )
 #define SPI2_PCLK_EN()    ( RCC->APB1ENR |= (1 << 14) )
 #define SPI3_PCLK_EN()    ( RCC->APB1ENR |= (1 << 15) )
 #define SPI4_PCLK_EN()    ( RCC->APB2ENR |= (1 << 13) )
 
 #define SPI1_PCLK_DI()    ( RCC->APB2ENR &= ~(1 << 12) )
 #define SPI2_PCLK_DI()    ( RCC->APB1ENR &= ~(1 << 14) )
 #define SPI3_PCLK_DI()    ( RCC->APB1ENR &= ~(1 << 15) )
 #define SPI4_PCLK_DI()    ( RCC->APB2ENR &= ~(1 << 13) )
 
 /*
  * Clock Enable and Disable Macros for USARTx peripheral
  */
 
 #define USART1_PCCK_EN()    ( RCC->APB2ENR |= (1 << 4)     )
 #define USART2_PCCK_EN()    ( RCC->APB1ENR |= (1 << 17) )
 #define USART3_PCCK_EN()    ( RCC->APB1ENR |= (1 << 18) )
 #define UART4_PCCK_EN()        ( RCC->APB1ENR |= (1 << 19) )
 #define UART5_PCCK_EN()        ( RCC->APB1ENR |= (1 << 20) )
 #define USART6_PCCK_EN()    ( RCC->APB2ENR |= (1 << 5)     )
 
 #define USART1_PCCK_DI()    ( RCC->APB2ENR &= ~(1 << 4)     )
 #define USART2_PCCK_DI()    ( RCC->APB1ENR &= ~(1 << 17) )
 #define USART3_PCCK_DI()    ( RCC->APB1ENR &= ~(1 << 18) )
 #define UART4_PCCK_DI()        ( RCC->APB1ENR &= ~(1 << 19) )
 #define UART5_PCCK_DI()        ( RCC->APB1ENR &= ~(1 << 20) )
 #define USART6_PCCK_DI()    ( RCC->APB2ENR &= ~(1 << 5)     )
 
 /*
  * Clock Enable and Disable Macros for SYSCFG peripheral
  */
 #define SYSCFG_PCLK_EN()    ( RCC->APB2ENR |= (1 << 14) )
 
 #define SYSCFG_PCLK_DI()    ( RCC->APB2ENR &= ~(1 << 14) )
 
 
 /*************************************************************
  *                     REGISTER RESET MACROS
  *************************************************************/
 
 /*
  *  Macros to reset GPIOx peripherals
  */
 #define GPIOA_REG_RESET()                do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
 #define GPIOB_REG_RESET()                do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
 #define GPIOC_REG_RESET()                do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
 #define GPIOD_REG_RESET()                do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
 #define GPIOE_REG_RESET()                do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
 #define GPIOF_REG_RESET()                do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
 #define GPIOG_REG_RESET()                do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
 #define GPIOH_REG_RESET()                do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
 #define GPIOI_REG_RESET()                do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); }while(0)
 
 /*
  *  Macros to reset SPIx peripherals
  */
 #define SPI1_REG_RESET()                do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
 #define SPI2_REG_RESET()                do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); }while(0)
 #define SPI3_REG_RESET()                do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); }while(0)
 #define SPI4_REG_RESET()                do{ (RCC->APB2RSTR |= (1 << 13)); (RCC->APB2RSTR &= ~(1 << 13)); }while(0)
 
 /*
  *  Macros to reset I2Cx peripherals
  */
 #define I2C1_REG_RESET()                do{ (RCC->APB1RSTR |= (1 << 21)); (RCC->APB1RSTR &= ~(1 << 21)); }while(0)
 #define I2C2_REG_RESET()                do{ (RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22)); }while(0)
 #define I2C3_REG_RESET()                do{ (RCC->APB1RSTR |= (1 << 23)); (RCC->APB1RSTR &= ~(1 << 23)); }while(0)
 
 
 /*************************************************************
  *                         RETURN MACROS
  *************************************************************/
 
 /*
  * This macro returns a code( between 0 to 7) for a given GPIO base address(x)
  */
 #define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA)?0:\
                                         (x == GPIOB)?1:\
                                         (x == GPIOC)?2:\
                                         (x == GPIOD)?3:\
                                         (x == GPIOE)?4:\
                                         (x == GPIOF)?5:\
                                         (x == GPIOG)?6:\
                                         (x == GPIOH)?7:0 )
 
 /*************************************************************
  *                     SOME GENERIC MACROS
  *************************************************************/
 
 #define ENABLE                 1
 #define DISABLE             0
 #define SET                 ENABLE
 #define RESET                 DISABLE
 #define GPIO_PIN_SET        SET
 #define GPIO_PIN_RESET      RESET
 #define FLAG_RESET         RESET
 #define FLAG_SET             SET
 
 /*
  * IRQ(Interrupt Request) Numbers of STM32F446xx MCU
  */
 
 #define IRQ_NO_EXTI0         6
 #define IRQ_NO_EXTI1         7
 #define IRQ_NO_EXTI2         8
 #define IRQ_NO_EXTI3         9
 #define IRQ_NO_EXTI4         10
 #define IRQ_NO_EXTI9_5         23
 #define IRQ_NO_EXTI15_10     40
 
 
 #define IRQ_NO_SPI1            35
 #define IRQ_NO_SPI2         36
 #define IRQ_NO_SPI3         51
 #define IRQ_NO_SPI4            84
 
 
 #define IRQ_NO_I2C1_EV        31
 #define IRQ_NO_I2C1_ER        32
 
 #define IRQ_NO_USART1        37
 #define IRQ_NO_USART2        38
 #define IRQ_NO_USART3        39
 #define IRQ_NO_UART4        52
 #define IRQ_NO_UART5        53
 #define IRQ_NO_USART6        71
 
 
 /*
  * macros for all the possible priority levels
  */
 #define NVIC_IRQ_PRI0        0
 #define NVIC_IRQ_PRI15        15
 /***********************************************
  *  Bit position definitions of SPI peripheral
  ***********************************************/
 /*
  * Bit position definitions SPI_CR1
  */
 #define SPI_CR1_CPHA             0
 #define SPI_CR1_CPOL              1
 #define SPI_CR1_MSTR             2
 #define SPI_CR1_BR               3
 #define SPI_CR1_SPE             6
 #define SPI_CR1_LSBFIRST           7
 #define SPI_CR1_SSI             8
 #define SPI_CR1_SSM              9
 #define SPI_CR1_RXONLY          10
 #define SPI_CR1_DFF             11
 #define SPI_CR1_CRCNEXT           12
 #define SPI_CR1_CRCEN           13
 #define SPI_CR1_BIDIOE             14
 #define SPI_CR1_BIDIMODE          15
 
 /*
  * Bit position definitions SPI_CR2
  */
 #define SPI_CR2_RXDMAEN             0
 #define SPI_CR2_TXDMAEN            1
 #define SPI_CR2_SSOE            2
 #define SPI_CR2_FRF                4
 #define SPI_CR2_ERRIE            5
 #define SPI_CR2_RXNEIE            6
 #define SPI_CR2_TXEIE            7
 
 
 /*
  * Bit position definitions SPI_SR
  */
 #define SPI_SR_RXNE                0
 #define SPI_SR_TXE                1
 #define SPI_SR_CHSIDE            2
 #define SPI_SR_UDR                3
 #define SPI_SR_CRCERR            4
 #define SPI_SR_MODF                5
 #define SPI_SR_OVR                6
 #define SPI_SR_BSY                7
 #define SPI_SR_FRE                8
 
 
 /**********************************************
  *Bit position definitions of I2C peripheral
  **********************************************/
 /*
  * Bit position definitions I2C_CR1
  */
 #define I2C_CR1_PE                        0
 #define I2C_CR1_NOSTRETCH                  7
 #define I2C_CR1_START                     8
 #define I2C_CR1_STOP                       9
 #define I2C_CR1_ACK                      10
 #define I2C_CR1_SWRST                       15
 
 /*
  * Bit position definitions I2C_CR2
  */
 #define I2C_CR2_FREQ                     0
 #define I2C_CR2_ITERREN                     8
 #define I2C_CR2_ITEVTEN                     9
 #define I2C_CR2_ITBUFEN                 10
 
 /*
  * Bit position definitions I2C_OAR1
  */
 #define I2C_OAR1_ADD0                     0
 #define I2C_OAR1_ADD71                       1
 #define I2C_OAR1_ADD98                    8
 #define I2C_OAR1_ADDMODE                    15
 
 /*
  * Bit position definitions I2C_SR1
  */
 
 #define I2C_SR1_SB                          0
 #define I2C_SR1_ADDR                      1
 #define I2C_SR1_BTF                     2
 #define I2C_SR1_ADD10                     3
 #define I2C_SR1_STOPF                     4
 #define I2C_SR1_RXNE                     6
 #define I2C_SR1_TXE                     7
 #define I2C_SR1_BERR                     8
 #define I2C_SR1_ARLO                     9
 #define I2C_SR1_AF                          10
 #define I2C_SR1_OVR                     11
 #define I2C_SR1_TIMEOUT                 14
 
 /*
  * Bit position definitions I2C_SR2
  */
 #define I2C_SR2_MSL                        0
 #define I2C_SR2_BUSY                     1
 #define I2C_SR2_TRA                     2
 #define I2C_SR2_GENCALL                 4
 #define I2C_SR2_DUALF                     7
 
 /*
  * Bit position definitions I2C_CCR
  */
 #define I2C_CCR_CCR                      0
 #define I2C_CCR_DUTY                     14
 #define I2C_CCR_FS                       15
 
 
 /**********************************************
  *Bit position definitions of USART peripheral
  **********************************************/
 
 /*
  * Bit position definitions USART_CR1
  */
 #define USART_CR1_SBK                    0
 #define USART_CR1_RWU                     1
 #define USART_CR1_RE                      2
 #define USART_CR1_TE                     3
 #define USART_CR1_IDLEIE                 4
 #define USART_CR1_RXNEIE                  5
 #define USART_CR1_TCIE                    6
 #define USART_CR1_TXEIE                    7
 #define USART_CR1_PEIE                     8
 #define USART_CR1_PS                     9
 #define USART_CR1_PCE                     10
 #define USART_CR1_WAKE                  11
 #define USART_CR1_M                     12
 #define USART_CR1_UE                     13
 #define USART_CR1_OVER8                  15
 
 
 
 /*
  * Bit position definitions USART_CR2
  */
 #define USART_CR2_ADD                   0
 #define USART_CR2_LBDL                   5
 #define USART_CR2_LBDIE                  6
 #define USART_CR2_LBCL                   8
 #define USART_CR2_CPHA                   9
 #define USART_CR2_CPOL                   10
 #define USART_CR2_STOP                   12
 #define USART_CR2_LINEN                   14
 
 
 /*
  * Bit position definitions USART_CR3
  */
 #define USART_CR3_EIE                   0
 #define USART_CR3_IREN                   1
 #define USART_CR3_IRLP                  2
 #define USART_CR3_HDSEL                   3
 #define USART_CR3_NACK                   4
 #define USART_CR3_SCEN                   5
 #define USART_CR3_DMAR                  6
 #define USART_CR3_DMAT                   7
 #define USART_CR3_RTSE                   8
 #define USART_CR3_CTSE                   9
 #define USART_CR3_CTSIE                   10
 #define USART_CR3_ONEBIT                   11
 
 /*
  * Bit position definitions USART_SR
  */
 
 #define USART_SR_PE                        0
 #define USART_SR_FE                        1
 #define USART_SR_NE                        2
 #define USART_SR_ORE                       3
 #define USART_SR_IDLE                   4
 #define USART_SR_RXNE                    5
 #define USART_SR_TC                        6
 #define USART_SR_TXE                    7
 #define USART_SR_LBD                    8
 #define USART_SR_CTS                    9
 
 
 
 
 #include <stm32f446xx_gpio_driver.h>
 #include <stm32f446xx_spi_driver.h>
 #include <stm32f446xx_i2c_driver.h>
 #include <stm32f446xx_rcc_driver.h>
 #include <stm32f446xx_usart_driver.h>
 
 
 #endif /* INC_STM32F103XX_H_ */