#ifndef MYGLOBAL_GUARD
#define MYGLOBAL_GUARD
typedef struct {
    uint8_t symbol13: 1;
} symbol_status;
symbol_status symbols_found;
uint16_t currentLengthBuffer;
#define MAX_RX_BUFFER 512
#define STOP_SYMBOL   0x0D
uint8_t RXbuffer[MAX_RX_BUFFER];

#endif