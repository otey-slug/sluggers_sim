#ifndef HOST_STUB_SERIAL_H
#define HOST_STUB_SERIAL_H

int IsReceiveEmpty(void);
char GetChar(void);
int IsTransmitEmpty(void);
void PutChar(char ch);

#endif
