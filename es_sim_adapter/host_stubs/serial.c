#include "serial.h"
#include <stdio.h>

int IsReceiveEmpty(void)
{
    return 1;
}

char GetChar(void)
{
    return '\0';
}

int IsTransmitEmpty(void)
{
    return 1;
}

void PutChar(char ch)
{
    fputc(ch, stdout);
}
