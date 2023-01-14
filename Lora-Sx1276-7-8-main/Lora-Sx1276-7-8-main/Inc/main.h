#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

void Error_Handler(const char* message);
void print(const char* message);
void printLine(const char* message);

#ifdef __cplusplus
}
#endif

#endif 
