#ifndef _IRQHANDLER_H
#define _IRQHANDLER_H

#include "globals.h"

void irqHandler(void *pvParameters);

void initIRQTask();
void notifyIrq(uint32_t uvalue);
void deleteIRQTask();
void setIrqMask(bool set);
void IRAM_ATTR RstButtonIRQ();
void IRAM_ATTR VibButtonIRQ();
#endif