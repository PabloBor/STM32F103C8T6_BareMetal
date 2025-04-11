/*
 * stm32f103x8_gpio_driver.h
 *
 *  Created on: Apr 4, 2025
 *      Author: Pablo Borboa
 */

#ifndef STM32F103X8_GPIO_DRIVER_H_
#define STM32F103X8_GPIO_DRIVER_H_

typedef enum {
    PinA,     // 0
    NibbleA,  // 1
    ByteA,    // 2
    PortA,    // 3
    
    PinB,     // 4
    NibbleB,  // 5
    ByteB,    // 6
    PortB,    // 7
    
    PinC,     // 8
    NibbleC,  // 9
    ByteC,    // 10
    PortC,    // 11
    
    PinD,     // 12
    NibbleD,  // 13
    ByteD,    // 14
    PortD,    // 15

    PinE,     // 16
    NibbleE,  // 17
    ByteE,    // 18
    PortE     // 19
} _GpioPortSection;

typedef enum {
    Output,
    Output_GenPurpose_Push_pull,
    Output_GenPurpose_Open_drain,
    Output_AltPurpose_Push_pull,
    Output_AltPurpose_Open_drain,

    Input,
    Input_Analog,
    Input_Input_floating,
    Input_Input_pull_down,
    Input_Input_pull_up,
} _GpioMode;

//                    input
//							Analog
//							Input floating
//							Input pull-down
//							Input pull-up
//
//                    output
//                          General purpose output
//                                                 Push-pull
//                                                 Open-drain
//                          Alternate Function output
//                                                 Push-pull
//                                                 Open-drain



/*****************************************************************************************************
 * 									APIs supported by this driver
 * 			For more information about the APIs check the function definitions
 *****************************************************************************************************/

/*
 *  Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDis);

/*
 * Init and De-Init
 */
void GPIO_Init(_GpioPortSection Section, int Position, _GpioMode Mode);

void GPIO_Init_Complex(_GpioPortSection Section, int Position, _GpioMode Mode, Alternative, );
        void GPIO_Alt_Funct(_GpioPortSection Section, int Position, _GpioMode Mode);









void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
//void GPIO_ToggleOutputPort(void);				// MAYBE THIS COULD BE A GOOD ADD TO THE COURSE

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);




#endif /* STM32F103X8_GPIO_DRIVER_H_ */