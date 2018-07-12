#ifndef DEFINES_H_
#define DEFINES_H_

//sta³e do biblioteki TM
#define GPS_USART                USART6
#define GPS_USART_PINSPACK        TM_USART_PinsPack_1

//konfiguracja timerów PWM
#define PWM_PERIOD 999
#define PWM_PRESCALER 1439

//sta³e seRGB
#define INIT 0
#define R_RGB_INIT 0
#define G_RGB_INIT 0
#define B_RGB_INIT 999
#define OK 1
#define R_RGB_OK 0
#define G_RGB_OK 999
#define B_RGB_OK 0
#define FAULT 2
#define R_RGB_FAULT 999
#define G_RGB_FAULT 0
#define B_RGB_FAULT 0

//pozosta³e
#define HDOP_MIN 0.1
#define HDOP_MAX 1.4
#define HDOP_COUNT 10
#define EMPTY 1.1
#endif /* DEFINES_H_ */
