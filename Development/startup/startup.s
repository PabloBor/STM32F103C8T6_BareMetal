.section .vector_table, "a"
.align 4
.global __vector_table
.extern _stack_top

__vector_table:
    .word _stack_top                      /* Dirección inicial de la pila */
    .word  reset_handler                  // Vector de reset
    .word  NMI_Handler                    // NMI
    .word  HardFault_Handler              // HardFault
    .word  MemoryFault_Handler            // MemFault
    .word  BusFault_Handler               // BusFault
    .word  UsageFault_Handler             // UsageFault
    .word  0                              // Reservado
    .word  0                              // Reservado
    .word  0                              // Reservado
    .word  0                              // Reservado
    .word  SVCall_Handler                 // SVCall
    .word  DebugMonitor_Handler           // Debug Monitor
    .word  0                              // Reservado
    .word  PendSV_Handler                 // PendSV
    .word  SysTick_Handler                // SysTick

    // Interrupciones del STM32F103C8T6
    .word  WWDG_IRQHandler                // Window Watchdog
    .word  PVD_IRQHandler                 // PVD through EXTI Line detection
    .word  TAMPER_IRQHandler              // Tamper and TimeStamp
    .word  RTC_IRQHandler                 // RTC global interrupt
    .word  FLASH_IRQHandler               // FLASH global interrupt
    .word  RCC_IRQHandler                 // RCC global interrupt
    .word  EXTI0_IRQHandler               // EXTI Line 0 interrupt
    .word  EXTI1_IRQHandler               // EXTI Line 1 interrupt
    .word  EXTI2_IRQHandler               // EXTI Line 2 interrupt
    .word  EXTI3_IRQHandler               // EXTI Line 3 interrupt
    .word  EXTI4_IRQHandler               // EXTI Line 4 interrupt
    .word  DMA1_Channel1_IRQHandler       // DMA1 Channel 1 global interrupt
    .word  DMA1_Channel2_IRQHandler       // DMA1 Channel 2 global interrupt
    .word  DMA1_Channel3_IRQHandler       // DMA1 Channel 3 global interrupt
    .word  DMA1_Channel4_IRQHandler       // DMA1 Channel 4 global interrupt
    .word  DMA1_Channel5_IRQHandler       // DMA1 Channel 5 global interrupt
    .word  DMA1_Channel6_IRQHandler       // DMA1 Channel 6 global interrupt
    .word  DMA1_Channel7_IRQHandler       // DMA1 Channel 7 global interrupt
    .word  ADC1_2_IRQHandler              // ADC1 and ADC2 global interrupt
    .word  USB_HP_CAN1_TX_IRQHandler      // USB High Priority CAN1 TX interrupt
    .word  USB_LP_CAN1_RX0_IRQHandler     // USB Low Priority CAN1 RX0 interrupt
    .word  CAN1_RX1_IRQHandler            // CAN1 RX1 interrupt
    .word  CAN1_SCE_IRQHandler            // CAN1 SCE interrupt
    .word  EXTI9_5_IRQHandler             // EXTI Line 5 to 9 interrupts
    .word  TIM1_BRK_TIM15_IRQHandler      // TIM1 Break and TIM15 global interrupt
    .word  TIM1_UP_TIM16_IRQHandler       // TIM1 Update and TIM16 global interrupt
    .word  TIM1_TRG_COM_TIM17_IRQHandler  // TIM1 Trigger and Commutation, TIM17 global interrupt
    .word  TIM1_CC_IRQHandler             // TIM1 Capture Compare global interrupt
    .word  TIM2_IRQHandler                // TIM2 global interrupt
    .word  TIM3_IRQHandler                // TIM3 global interrupt
    .word  TIM4_IRQHandler                // TIM4 global interrupt
    .word  I2C1_EV_IRQHandler             // I2C1 event interrupt
    .word  I2C1_ER_IRQHandler             // I2C1 error interrupt
    .word  I2C2_EV_IRQHandler             // I2C2 event interrupt
    .word  I2C2_ER_IRQHandler             // I2C2 error interrupt
    .word  SPI1_IRQHandler                // SPI1 global interrupt
    .word  SPI2_IRQHandler                // SPI2 global interrupt
    .word  USART1_IRQHandler              // USART1 global interrupt
    .word  USART2_IRQHandler              // USART2 global interrupt
    .word  USART3_IRQHandler              // USART3 global interrupt
    .word  EXTI15_10_IRQHandler           // EXTI Line 10 to 15 interrupts
    .word  RTC_Alarm_IRQHandler           // RTC Alarm (A and B) interrupt
    .word  USBWakeUp_IRQHandler           // USB Wakeup from suspend interrupt
    .word  TIM8_BRKHandler                // 
    .word  TIM8_UPHandler                 // 
    .word  TIM8_TRG_COMHandler            // 
    .word  TIM8_CCHandler                 // 
    .word  ADC3Handler                    // 
    .word  FSMCHandler                    // 
    .word  SDIOHandler                    // 
    .word  TIM5Handler                    // 
    .word  SPI3Handler                    // 
    .word  UART4Handler                   // 
    .word  UART5Handler                   // 
    .word  TIM6Handler                    // 
    .word  TIM7Handler                    // 
    .word  DMA2_Channel1Handler           // 
    .word  DMA2_Channel2Handler           // 
    .word  DMA2_Channel3Handler           // 
    .word  DMA2_Channel4_5Handler         // 

    .section .text

    .global reset_handler
reset_handler:
    ldr r0, =_stack_top                // Carga la dirección de la pila
    mov sp, r0                         // Configura el puntero de la pila
    bl main                            // Llama a main()
    b .                                // Si main() termina, se queda en un loop infinito

    // Definición de manejadores de excepciones
    .global NMI_Handler
NMI_Handler:
    b Default_Handler                  // Llama al manejador por defecto

    .global HardFault_Handler
HardFault_Handler:
    b Default_Handler                  // Llama al manejador por defecto

    .global MemoryFault_Handler
MemoryFault_Handler:
    b Default_Handler                  // Llama al manejador por defecto

    .global BusFault_Handler
BusFault_Handler:
    b Default_Handler                  // Llama al manejador por defecto

    .global UsageFault_Handler
UsageFault_Handler:
    b Default_Handler                  // Llama al manejador por defecto

    .global SVCall_Handler
SVCall_Handler:
    b Default_Handler                  // Llama al manejador por defecto

    .global DebugMonitor_Handler
DebugMonitor_Handler:
    b Default_Handler                  // Llama al manejador por defecto

    .global PendSV_Handler
PendSV_Handler:
    b Default_Handler                  // Llama al manejador por defecto

    .global SysTick_Handler
SysTick_Handler:
    b Default_Handler                  // Llama al manejador por defecto

    // Interrupciones del STM32F103C8T6
    .global WWDG_IRQHandler
WWDG_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global PVD_IRQHandler
PVD_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global TAMPER_IRQHandler
TAMPER_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global RTC_IRQHandler
RTC_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global FLASH_IRQHandler
FLASH_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global RCC_IRQHandler
RCC_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global EXTI0_IRQHandler
EXTI0_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global EXTI1_IRQHandler
EXTI1_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global EXTI2_IRQHandler
EXTI2_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global EXTI3_IRQHandler
EXTI3_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global EXTI4_IRQHandler
EXTI4_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global DMA1_Channel1_IRQHandler
DMA1_Channel1_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global DMA1_Channel2_IRQHandler
DMA1_Channel2_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global DMA1_Channel3_IRQHandler
DMA1_Channel3_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global DMA1_Channel4_IRQHandler
DMA1_Channel4_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global DMA1_Channel5_IRQHandler
DMA1_Channel5_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global DMA1_Channel6_IRQHandler
DMA1_Channel6_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global DMA1_Channel7_IRQHandler
DMA1_Channel7_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global ADC1_2_IRQHandler
ADC1_2_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global USB_HP_CAN1_TX_IRQHandler
USB_HP_CAN1_TX_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global USB_LP_CAN1_RX0_IRQHandler
USB_LP_CAN1_RX0_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global CAN1_RX1_IRQHandler
CAN1_RX1_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global CAN1_SCE_IRQHandler
CAN1_SCE_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global EXTI9_5_IRQHandler
EXTI9_5_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global TIM1_BRK_TIM15_IRQHandler
TIM1_BRK_TIM15_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global TIM1_UP_TIM16_IRQHandler
TIM1_UP_TIM16_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global TIM1_TRG_COM_TIM17_IRQHandler
TIM1_TRG_COM_TIM17_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global TIM1_CC_IRQHandler
TIM1_CC_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global TIM2_IRQHandler
TIM2_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global TIM3_IRQHandler
TIM3_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global TIM4_IRQHandler
TIM4_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global I2C1_EV_IRQHandler
I2C1_EV_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global I2C1_ER_IRQHandler
I2C1_ER_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global I2C2_EV_IRQHandler
I2C2_EV_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global I2C2_ER_IRQHandler
I2C2_ER_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global SPI1_IRQHandler
SPI1_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global SPI2_IRQHandler
SPI2_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global USART1_IRQHandler
USART1_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global USART2_IRQHandler
USART2_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global USART3_IRQHandler
USART3_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global EXTI15_10_IRQHandler
EXTI15_10_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global RTC_Alarm_IRQHandler
RTC_Alarm_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global USBWakeUp_IRQHandler
USBWakeUp_IRQHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global TIM8_BRKHandler
TIM8_BRKHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global TIM8_UPHandler
TIM8_UPHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global TIM8_TRG_COMHandler
TIM8_TRG_COMHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global TIM8_CCHandler
TIM8_CCHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global ADC3Handler
ADC3Handler:
    b Default_Handler                  // Llama al manejador por defecto

    .global FSMCHandler
FSMCHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global SDIOHandler
SDIOHandler:
    b Default_Handler                  // Llama al manejador por defecto

    .global TIM5Handler
TIM5Handler:
    b Default_Handler                  // Llama al manejador por defecto

    .global SPI3Handler
SPI3Handler:
    b Default_Handler                  // Llama al manejador por defecto

    .global UART4Handler
UART4Handler:
    b Default_Handler                  // Llama al manejador por defecto

    .global UART5Handler
UART5Handler:
    b Default_Handler                  // Llama al manejador por defecto

    .global TIM6Handler
TIM6Handler:
    b Default_Handler                  // Llama al manejador por defecto

    .global TIM7Handler
TIM7Handler:
    b Default_Handler                  // Llama al manejador por defecto

    .global DMA2_Channel1Handler
DMA2_Channel1Handler:
    b Default_Handler                  // Llama al manejador por defecto

    .global DMA2_Channel2Handler
DMA2_Channel2Handler:
    b Default_Handler                  // Llama al manejador por defecto

    .global DMA2_Channel3Handler
DMA2_Channel3Handler:
    b Default_Handler                  // Llama al manejador por defecto

    .global DMA2_Channel4_5Handler
DMA2_Channel4_5Handler:
    b Default_Handler                  // Llama al manejador por defecto

    .global Default_Handler
Default_Handler:
    b .                                // Entra en un bucle infinito

    .size reset_handler, .-reset_handler
