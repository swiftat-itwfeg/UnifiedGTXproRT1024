#include "averyPrStatus.h"
static uint32_t sPrintState = SPS_LABEL_FEED_ERROR;

void initPrinterState( void ) { sPrintState = SPS_LABEL_FEED_ERROR; }
uint32_t getPrinterState( void ) { return sPrintState; }
void setPrinterState( PrinterStates_t state ) { sPrintState = state; }      
