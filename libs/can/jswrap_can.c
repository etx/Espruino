
/*
 * ----------------------------------------------------------------------------
 * Contains JavaScript interface for Controller Area Network
 * ----------------------------------------------------------------------------
 *
 *  CAN for STM32F4 by etx313 at gmail
 */


#include "jswrap_can.h"
#include "jsinteractive.h"




CAN_InitTypeDef        CAN_InitStructure;
CAN_FilterInitTypeDef  CAN_FilterInitStructure;

JsVar *jsVarCan1;
JsVar *jsVarCan2;
JsVar *jsVarCan3;




/* Private function prototypes -----------------------------------------------*/
static void NVIC_Config(void);
static int CAN_Config(CAN_TypeDef* canx, int pin_rx, int pin_tx, int baud);
void Init_RxMes(CanRxMsg *RxMessage);
static uint16_t stmPin(Pin ipin);
static GPIO_TypeDef *stmPort(Pin pin);
static uint8_t stmPinSource(JsvPinInfoPin ipin);
/* Private functions ---------------------------------------------------------*/



// Setup JS Class
/*JSON{
  "type" : "class",
  "class" : "CAN"
}*/

/*JSON{
  "type" : "event",
  "class" : "CAN",
  "name" : "message",
  "params" : [
    ["data","JsVar","A CAN Message Object"]
  ]
}*/

/*JSON{
  "type" : "constructor",
  "class" : "CAN",
  "name" : "CAN",
  "generate" : "jswrap_can_constructor",
  "params" : [
    ["args","int","Bus ID"]
  ],
  "return" : ["JsVar","A CAN object"],
  "return_object" : "CAN"
}
Creates a object
 */
JsVar *jswrap_can_constructor(int id) {

  JsVar *obj = jspNewObject(0, "CAN");
  if (!obj) return 0; // out of memory
  return obj;
}

/*JSON{
  "type" : "idle",
  "generate" : "jswrap_can_idle"
}*/
bool jswrap_can_idle() {
  // if (mag_enabled && nrf_gpio_pin_read(MAG_INT)) {
  //   int16_t d[3];
  //   mag_read();
  //   JsVar *xyz = mag_to_xyz(mag_reading);
  //   JsVar *puck = jsvObjectGetChild(execInfo.root, "Puck", 0);
  //   if (jsvHasChildren(puck))
  //       jsiQueueObjectCallbacks(puck, JS_EVENT_PREFIX"mag", &xyz, 1);
  //   jsvUnLock2(puck, xyz);
  //   return true; // don't sleep - handle this now
  // }
  // return false;

  return false;

}




/*JSON{
  "type" : "staticmethod",
  "class" : "CAN",
  "name" : "init",
  "generate" : "jswrap_can_init",
  "params" : [
    ["i", "int","Bus ID to init"],
    ["baud","int","Bus speed"]
  ],
  "return" : ["JsVar","The new CANBus object"],
  "return_object" : "CAN"
}*/
JsVar *jswrap_can_init(int i, int baud) {

    int didInit;

    JsVar *obj = jspNewObject(0, "CAN");
    if (!obj) return 0; // out of memory

    baud = (baud == 0 ? 500 : baud);

    /* CAN configuration */
    switch(i){
      case 1:
        jsVarCan1 = obj;
        didInit = CAN_Config(CAN1, CAN1_RX, CAN1_TX, baud);
      break;
      case 2:
        jsVarCan2 = obj;
        didInit = CAN_Config(CAN2, CAN2_RX, CAN2_TX, baud);
      break;
      case 3:
        jsVarCan3 = obj;
        didInit = CAN_Config(CAN3, CAN3_RX, CAN3_TX, baud);
      break;
    }

    if( didInit == 0 ){
      jsExceptionHere(JSET_ERROR, "CAN failed to init.");
      return 0;
    }

    jsvObjectSetChild(obj, "busId", jsvNewFromInteger(didInit==1?i:0));
    jsvObjectSetChild(obj, "baud", jsvNewFromInteger(baud));
    jsvObjectSetChildAndUnLock(obj, "didInit", jsvNewFromString(didInit==1?"true":"false"));

    return obj;
}






/*JSON{
  "type" : "method",
  "class" : "CAN",
  "name" : "send",
  "generate" : "jswrap_can_send",
  "params" : [
    ["obj", "JsVar","Object"],
    ["msg", "JsVar","CAN Message Object"]
  ],
  "return" : ["bool",""]
}*/
bool jswrap_can_send(JsVar *obj, JsVar *msg) {

  JsVar *bus = jsvObjectGetChild(obj, "busId", 0);

  if (!jsvIsObject(msg)) {
    jsExceptionHere(JSET_ERROR, "Expecting message object but got %t", msg);
    return 0;
  }

  #ifdef STM32

  uint8_t busId = jsvGetInteger(bus);
  CAN_TypeDef *CANx;
  switch(busId){
    case 1: CANx = CAN1; break;
    case 2: CANx = CAN2; break;
    case 3: CANx = CAN3; break;
  }

  CanTxMsg tx;
  txStructFromObject(msg, &tx);
  CAN_Transmit(CANx, &tx);

  /* Wait until one of the mailboxes is empty */
  while((CAN_GetFlagStatus(CAN1, CAN_FLAG_RQCP0) !=RESET ) || \
        (CAN_GetFlagStatus(CAN1, CAN_FLAG_RQCP1) !=RESET ) || \
        (CAN_GetFlagStatus(CAN1, CAN_FLAG_RQCP2) !=RESET ));

  #endif

  // jsiConsolePrintf("Sent on CAN1! %d \r\n", bus);
  jsiConsolePrintf("Sent on CAN1!  %d \r\n", busId);


}




/*JSON{
  "type" : "method",
  "class" : "CAN",
  "name" : "read",
  "generate" : "jswrap_can_read",
  "return" : ["bool",""]
}*/
bool jswrap_can_read(JsVar *obj){

  JsVar *bus = jsvObjectGetChild(obj, "busId", 0);
  uint8_t busId = jsvGetInteger(bus);

  CAN_TypeDef *CANx;
  switch(busId){
    case 1: CANx = CAN1; break;
    case 2: CANx = CAN2; break;
    case 3: CANx = CAN3; break;
  }

	CanRxMsg RxMessage;
	uint32_t state = 0;
	uint8_t messagePending = CAN_MessagePending(CANx, CAN_FIFO0);
  //CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);

  jsiConsolePrintf("CAN_MessagePending = 0x%x\r\n", messagePending);

    state = CANx->MSR;
    // printf("MSR = 0x%x\r\n", state);
    jsiConsolePrintf("MSR = 0x%x\r\n", state);

    state = CANx->ESR;
    //printf("ESR = 0x%x\r\n", state);
    jsiConsolePrintf("ESR = 0x%x\r\n", state);

	while (messagePending > 0){
		CAN_Receive(CANx, CAN_FIFO0, &RxMessage);

    jsiConsolePrintf("MSG %x %x\r\n", RxMessage.StdId, RxMessage.Data[0]);

		// receivedMessage->stdid = (uint16_t)(0xFF & RxMessage.StdId); //message id
    //
		// for (int8_t i = 7; i>=0; i-=1){
		// 	receivedMessage->payload |= RxMessage.Data[i]; //fill in uint8_t payload to 8 byte can_message* buffer
		// 	if(i > 0)
		// 	{
		// 		receivedMessage->payload <<= 8;
		// 	}
		// }
		CAN_FIFORelease (CANx, CAN_FIFO0);
		messagePending -= 1;

		if(messagePending == 0){
			return true;
		}
	}
	return false;

}










/**
  * @brief  Configures the CAN.
  * @param  None
  * @retval None
  */
static int CAN_Config(CAN_TypeDef* canx, int pin_rx, int pin_tx, int baud)
{

  #ifdef STM32
  uint8_t InitStatus;
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* NVIC configuration */
  NVIC_Config();

  /* CAN GPIOs configuration **************************************************/

  /* Enable GPIO clock */
  // RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); // Espruino setup enables all clocks

  GPIO_TypeDef* Gpio_port = stmPort(pin_rx);

  uint8_t AF_port;
  canx == CAN1 ? AF_port = GPIO_AF_CAN1:0;
  canx == CAN2 ? AF_port = GPIO_AF_CAN2:0;
  canx == CAN3 ? AF_port = GPIO_AF_CAN3:0;

  #ifdef STM32F413xH
  // CAN1 on PB8 PB9 is AF8 not AF9! :/ make this work with the espruino build system? jshPinSetFunction()
  if (pin_rx == 24 && pin_tx == 25)
    AF_port = (uint8_t)0x08; // GPIO_AF_8
  #endif

  /* Connect CAN pins to Alternate Function */
  GPIO_PinAFConfig(Gpio_port, stmPinSource(pin_rx), AF_port);
  GPIO_PinAFConfig(Gpio_port, stmPinSource(pin_tx), AF_port);

  /* Configure CAN RX and TX pins */
  GPIO_InitStructure.GPIO_Pin   = stmPin(pin_rx) | stmPin(pin_tx);
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(Gpio_port, &GPIO_InitStructure);

  /* CAN configuration ********************************************************/
  /* Enable CAN clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 |
                        (canx == CAN2 ? RCC_APB1Periph_CAN2 : 0) |
                        (canx == CAN3 ? RCC_APB1Periph_CAN3 : 0)
                        , ENABLE);


  /* CAN register init */
  CAN_DeInit(canx);

  /* CAN cell init */
  CAN_InitStructure.CAN_TTCM = DISABLE;
  CAN_InitStructure.CAN_ABOM = DISABLE;
  CAN_InitStructure.CAN_AWUM = DISABLE;
  CAN_InitStructure.CAN_NART = DISABLE;
  CAN_InitStructure.CAN_RFLM = DISABLE;
  CAN_InitStructure.CAN_TXFP = DISABLE;
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1 = CAN_BS1_13tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;


  // Baud rate
	RCC_ClocksTypeDef     RCC_Clocks;
	RCC_GetClocksFreq(&RCC_Clocks);

  CAN_InitStructure.CAN_Prescaler = ((RCC_Clocks.PCLK1_Frequency/16) / 1000000) * (1000/baud);
  //CAN_InitStructure.CAN_Prescaler = 24; // Baud rate 125 Kbps
  //CAN_InitStructure.CAN_Prescaler = 12; // Baud rate 250 Kbps


  InitStatus = CAN_Init(canx, &CAN_InitStructure);

  /* CAN filter init */
  CAN_FilterInitStructure.CAN_FilterNumber = 0;
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0; // ?? Was 0
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);


  /* Enable FIFO 0 message pending Interrupt */
  CAN_ITConfig(canx, CAN_IT_FMP0, ENABLE);

  return !!InitStatus;
  #endif
}


/**
  * @brief  Configures the NVIC for CAN.
  * @param  None
  * @retval None
  */
static void NVIC_Config(void)
{
  NVIC_InitTypeDef  NVIC_InitStructure;

  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Initializes the Rx Message.
  * @param  RxMessage: pointer to the message to initialize
  * @retval None
  */
void Init_RxMes(CanRxMsg *RxMessage)
{
  jsiConsolePrintf("Init_RxMes\r\n");

  uint8_t ubCounter = 0;

  RxMessage->StdId = 0x00;
  RxMessage->ExtId = 0x00;
  RxMessage->IDE = CAN_ID_STD;
  RxMessage->DLC = 0;
  RxMessage->FMI = 0;
  for (ubCounter = 0; ubCounter < 8; ubCounter++)
  {
    RxMessage->Data[ubCounter] = 0x00;
  }
}


// Interrupt routines

/**
  * @brief  This function handles CAN1 RX0/RX1 Handlers.
  * @param  None
  * @retval None
  */
void CAN1_RX0_IRQHandler(void)
{
  CANx_ReadAndDispatch(CAN1, CAN_FIFO0);
}

void CAN1_RX1_IRQHandler(void)
{
  CANx_ReadAndDispatch(CAN1, CAN_FIFO1);
}


/**
  * @brief  This function handles CAN2 RX0/RX1 Handlers.
  * @param  None
  * @retval None
  */
void CAN2_RX0_IRQHandler(void)
{
  CANx_ReadAndDispatch(CAN2, CAN_FIFO0);
}

void CAN2_RX1_IRQHandler(void)
{
  CANx_ReadAndDispatch(CAN2, CAN_FIFO1);
}


/**
  * @brief  This function handles CAN3 RX0/RX1 Handlers.
  * @param  None
  * @retval None
  */
void CAN3_RX0_IRQHandler(void)
{
  CANx_ReadAndDispatch(CAN3, CAN_FIFO0);
}

void CAN3_RX1_IRQHandler(void)
{
  CANx_ReadAndDispatch(CAN3, CAN_FIFO1);
}



/**
  * @brief  Read CAN FIFO and dispatch JS event with the CAN Message data
  * @param  None
  * @retval None
  */
void CANx_ReadAndDispatch(CAN_TypeDef *CANx, uint8_t FIFONumber){

  CanRxMsg RxMessage;
	CAN_Receive(CANx, FIFONumber, &RxMessage);

  // Dispatch event
  JsVar *messageObj = messageObjectFromRxStruct(&RxMessage);
  jsiQueueObjectCallbacks(jsVarCan1, JS_EVENT_PREFIX"message", &messageObj, 1);
  jsvUnLock(messageObj);

}





/******************
* Helper functions
*/


/*
*   Build a TxMessage struct from JsVar Object
*/

void txStructFromObject( JsVar *msg, CanTxMsg *TxMessage ){

  JsVar *exid = jsvObjectGetChild(msg, "exid", 0);
  JsVar *id = jsvObjectGetChild(msg, "id", 0);
  JsVar *data = jsvObjectGetChild(msg, "data", 0);
  JsVar *length = jsvObjectGetChild(msg, "length", 0);

  /* Transmit Structure preparation */
  if( jsvGetInteger(exid) ){
    TxMessage->StdId = jsvGetInteger(id);
    TxMessage->ExtId = jsvGetInteger(exid);
    TxMessage->IDE = CAN_ID_EXT;
  }else if( jsvGetInteger(id) ){
    TxMessage->StdId = jsvGetInteger(id);
    TxMessage->IDE = CAN_ID_STD;
  }else{
    jsExceptionHere(JSET_ERROR, "Expecting id or exid in message object but got neither");
  }

  TxMessage->RTR = CAN_RTR_DATA;


  if (jsvIsArray(data)) {

    int i = 0;
    while(i < jsvGetArrayLength(data)){
      JsVar *item = jsvGetArrayItem(data, i);
      TxMessage->Data[i] = jsvGetInteger(item);
      i++;
    }

  }else{
    jsExceptionHere(JSET_ERROR, "Expecting data in message object but got %t", data);
  }

  if( length )
    TxMessage->DLC = jsvGetInteger(length);
  else
    TxMessage->DLC = jsvGetArrayLength(data);

}


/*
*   Build a JS Object from the CAN RX data
*/
JsVar *messageObjectFromRxStruct(CanRxMsg *rx){

  JsVar *rxObj = jsvNewObject();

  if(rx->IDE == CAN_ID_STD)
    jsvObjectSetChild(rxObj, "id", jsvNewFromInteger(rx->StdId));

  if(rx->IDE == CAN_ID_EXT)
    jsvObjectSetChild(rxObj, "exid", jsvNewFromInteger(rx->ExtId));

  JsVar *arr = jsvNewArray(NULL, 0);

  int i = 0;
  while(i<rx->DLC){
    jsvArrayPush(arr, jsvNewFromInteger(rx->Data[i]));
    i++;
  }

  jsvObjectSetChild(rxObj, "data", arr);
  jsvObjectSetChild(rxObj, "length", jsvNewFromInteger(rx->DLC));
  return rxObj;

}






static uint16_t stmPin(Pin ipin) {
  JsvPinInfoPin pin = pinInfo[ipin].pin;
  return (uint16_t)(1 << (pin-JSH_PIN0));
}

static GPIO_TypeDef *stmPort(Pin pin) {
  JsvPinInfoPort port = pinInfo[pin].port;
  return (GPIO_TypeDef *)((char*)GPIOA + (port-JSH_PORTA)*0x0400);
}

static uint8_t stmPinSource(JsvPinInfoPin ipin) {
  JsvPinInfoPin pin = pinInfo[ipin].pin;
  return (uint8_t)(pin-JSH_PIN0);
}
