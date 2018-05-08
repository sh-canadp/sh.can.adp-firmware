#ifndef __CAN_H__
#define __CAN_H__

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

#define LAWICEL_VERSION "1000"                      // Версия HHSS - возвращается V
#define LAWICEL_SERIAL_NUMBER "A000"                // Серийный номер - должен быть где то дальше во Flash или EEPROM

#define LENGTH_USB_IN_QUEUE 256                     // Длинна очереди

#define LW_CMD_MAX_LENGTH 40                        // Максимальная длинна команды LAWICEL, в байтах

#define LAWICEL_END  13                             // [cr] | \r
#define LAWICEL_OK  LAWICEL_END
#define LAWICEL_ERROR 7                             // [BELL] | \a

#define UC_STATE_OK  0                              // Сотояние все хорошо
#define UC_STATE_ERROR  1                           // Сотояние ошибка

// Status Flags (Mask)
#define STATUS_FLAGS_RX_OVERRUN_MASK    0b00001000  // Маска для бита RxOverrun
#define STATUS_FLAGS_ERROR_PASSIVE      0b00100000  // Маска для бита ErrorPassive
#define STATUS_FLAGS_BUS_OFF            0b10000000  // Маска для бита BusOff

typedef enum 
{
    USBCAN_CH_OPEN      = 1,                        // Состояние CAN - канал открыт
    USBCAN_CH_CLOSE     = 2
} USBCAN_StatusType;

typedef struct {
    uint8_t pBuffer[LW_CMD_MAX_LENGTH];
    uint8_t uLenght;
} LawicelMessage;

typedef enum {
    INMSG_USBIN               = 1,                        // Сообщение получено от USB
    INMSG_CANIN               = 2                         // Сообщение получено от CAN
} InMsgType;

typedef struct {
    void * PointerData;
    InMsgType type;
} StructInMsg;                                      // Структура, которая будет передаваться в очереди

typedef struct {                                    // Структура, содержит входящие сообщение по CAN и временную метку для него
    CanRxMsgTypeDef RxMsg;
    uint16_t TimeStamp;
} stRxMsgTimeStamp;

extern xSemaphoreHandle     xAccessTxCan;
extern uint8_t              StatusFlagsRegister;
extern USBCAN_StatusType    canStatus;
// extern CAN_HandleTypeDef    hCan; 

void usbcan_Init();
void usbReceiveCDC(uint8_t* Buf, uint32_t *Len);
void vUsbInProcess(void* pvParametrs);
void vSendUsbHost(void *pvParametrs);
void vCanTxSend(void *pvParametrs);
uint8_t usbcanWriteToBuffer(uint8_t *Buf, uint8_t Length);
void start_timer_time_stamp();
void stop_timer_time_stamp();

uint8_t can_switch_speed(uint8_t num);
HAL_StatusTypeDef can_open_channel(uint32_t CanMode);
uint32_t htol(uint8_t* buf, uint8_t len);
uint8_t ltoh(uint32_t l, uint8_t* buf, uint8_t len);

#endif