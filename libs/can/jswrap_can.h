/*
 * ----------------------------------------------------------------------------
 * Contains JavaScript interface for Controller Area Network
 * ----------------------------------------------------------------------------
 */

#include "stm32f4xx_can.h"
#include "jsvar.h"

JsVar *jswrap_can_constructor(int id);
JsVar *jswrap_can_init(int i, int baud);
bool jswrap_can_send();
bool jswrap_can_read();

void txStructFromObject( JsVar *msg, CanTxMsg *tx );
JsVar *messageObjectFromRxStruct(CanRxMsg *rx);
void CANx_ReadAndDispatch(CAN_TypeDef *CANx, uint8_t FIFONumber);
