#include <arch/board/board.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "chip.h"
#include "stm32l4_gpio.h"
//add by liushuhe 207.09.08
#include "nucleo-l476rg.h"



/* GPIO pins used by the GPIO Subsystem */

//add by liushuhe 2017.10.09
#define BOARD_NGPIOIN     0 /* Amount of GPIO Input pins */
#define BOARD_NGPIOOUT    0 /* Amount of GPIO Output pins */
#define BOARD_NGPIOINT    1 /* Amount of GPIO Input w/ Interruption pins */

//add by liushuhe 2017.10.09
#define GPIO_IN1          (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN7)
#define GPIO_OUT1         (GPIO_PORTB | GPIO_PIN7 | GPIO_OUTPUT_SET | GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz)                          
//add by liushuhe 2017.10.09
#define GPIO_INT1         (GPIO_INPUT | GPIO_PULLUP |GPIO_EXTI | GPIO_PORTC | GPIO_PIN13)

//add by liushuhe 2017.12.03
//#define GPIO_INT1         (GPIO_INPUT | GPIO_PULLUP |GPIO_EXTI | GPIO_PORTC | GPIO_PIN10)
//#define GPIO_INT2         (GPIO_INPUT | GPIO_PULLUP |GPIO_EXTI | GPIO_PORTC | GPIO_PIN10)
//#define GPIO_INT3         (GPIO_INPUT | GPIO_PULLUP |GPIO_EXTI | GPIO_PORTC | GPIO_PIN10)


//×¢²ágpioÇý¶¯Éè±¸
int stm32l4_gpiodev_initialize(void);




