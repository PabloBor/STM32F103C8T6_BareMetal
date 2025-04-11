/*
 * stm32f103x8_gpio_driver.c
 *
 *  Created on: Apr 4, 2025
 *      Author: Pablo Borboa
 */

#include "../header/stm32f103x8_gpio_driver.h"

int GPIO_ValidateSectionPosition(_GpioPortSection Section, int Position) {
    int return_code = 0; // No error
    
    switch (Section) {
        case PinA: case PinB: case PinC: case PinD: case PinE:
            // Pin 0 to 15
            if (Position < 0 || Position > 15) {
                return_code = 1; // Invalid
            }
            break;
        case NibbleA: case NibbleB: case NibbleC: case NibbleD: case NibbleE:
            // Nibble 0, 1, 2 or 3
            if (Position < 0 || Position > 3) {
                return_code = 1; // Invalid
            }
            break;
        case ByteA: case ByteB: case ByteC: case ByteD: case ByteE:
            // Byte 0 or 1
            if (Position < 0 || Position > 1) {
                return_code = 1; // Invalid
            }
            break;
        case PortA: case PortB: case PortC: case PortD: case PortE:
            // Port only 1
            if (Position != 1) {
                return_code = 1; // Invalid
            }
            break;
        default:
            return_code = 2; // Invalid
    }

    return return_code;
}
 


int GPIO_ValidateMode(_GpioMode Mode) {
    return (Mode < Output || Mode > Input_Input_pull_up);
}


void GPIO_Init(_GpioPortSection Section, int Position, _GpioMode Mode)
{
    if (GPIO_ValidateSectionPosition(Section, Position)) {
        return;
    }
    if (GPIO_ValidateMode(Mode)) {
        return;
    }





}