#include "stm32f3xx_hal.h"
#include "usbd_cdc_if.h"

#include "main.h"
#include "can.h"
#include "settings.h"

/* Определения */
#define TX_DATA_LENGTH_BUFFER  2048

extern USBD_HandleTypeDef hUsbDeviceFS;
extern TIM_HandleTypeDef htim6;
extern ADC_HandleTypeDef hadc1;
extern uint16_t uwTickTimer6;

xQueueHandle    xInMsgProcess;                  // Очередь для сырых сообщений, которые поступают из вне (USB, CAN)
xQueueHandle    xCanTxQueue;                    // Очередь для сообщений, которые нужно отправить по CAN

LawicelMessage *pCommandNext;                   // Указатель на строку (Нуль терминальная), со следующей командой 
                                                // Длинна текущей команды
uint8_t uStateRx;                               // Состояние получения команд от USB хост(ok, error)

uint8_t TxBuffer[TX_DATA_LENGTH_BUFFER];        // Буфер для отправки данных на хост (через USB или UART)
uint8_t TxBufferTwo[TX_DATA_LENGTH_BUFFER];     // Дублирующий буфер, для оптимизации
uint16_t TxBufferLength;                        // Колличество данных в буфере
xSemaphoreHandle xAccessTxBuffer;               // Мьютекс на доступ к буферу на отправку USB хост

// CAN_HandleTypeDef       hCan;                   // Хэндл для CAN (HAL)
extern CAN_HandleTypeDef hcan;
#define hCan    hcan

CanRxMsgTypeDef         sRxMsg;                 // Структура для хранения сообщения, которое было получено по CAN
CAN_FilterConfTypeDef   sFilterConfig;          // Фильтр CAN
USBCAN_StatusType       canStatus;              // Состояние can
xSemaphoreHandle        xAccessTxCan;           // Семафор, разрешает отправку сообщений по CAN (изначально свободны все mailbox = 3)

uint8_t                 uTimeStampState;        // Включена ли передача временной метки (1 - Вкл / 0 - Выкл)
/*  Содержит флаги состояния устройства
    Bit 0 -
    Bit 1 -
    Bit 2 -
    Bit 3 - RX overrun - RX очередь (программная, аппаратная или обе) была переполнена хотя бы раз с момента вызова команды F или с момента открытия канала.
    Bit 4 -
    Bit 5 - Error passive
    Bit 6 -
    Bit 7 - Bus off
*/
uint8_t                 StatusFlagsRegister;
FunctionalState         StateTerminalResistor;  // Состояние терминального резистора (ENABLE/DISABLE)
FunctionalState         StatePowerCANBusDevice; // Подати питание на CAN шину с утсройства

/* @breif: Обрабатывает входящие сообщения из USB
*/
void usbReceiveCDC(uint8_t* Buf, uint32_t *Len)
{
    BaseType_t xStatus;
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
    uint8_t uCmdTail;

    if (pCommandNext == NULL){
        pCommandNext = malloc(sizeof(LawicelMessage));
        pCommandNext->uLenght = 0;
    }

    for (uint32_t i=0; i < (*Len); i++){
        if (uStateRx == UC_STATE_ERROR){
            if (Buf[i] == LAWICEL_END){
                uStateRx = UC_STATE_OK;
                pCommandNext->uLenght = 0;
            }
            continue;
        }

        if (Buf[i] == LAWICEL_END){
            // Комадна получена целиком
            StructInMsg inMsgTmp;
            inMsgTmp.type = INMSG_USBIN;
            inMsgTmp.PointerData = pCommandNext;
            xStatus = xQueueSendToBackFromISR(xInMsgProcess, &inMsgTmp, &pxHigherPriorityTaskWoken);
            if (xStatus != pdTRUE){
                free(pCommandNext);
            }
            
            pCommandNext = malloc(sizeof(LawicelMessage));
            pCommandNext->uLenght = 0;
        }else{
            if ((LW_CMD_MAX_LENGTH - pCommandNext->uLenght) > 0) {
                pCommandNext->pBuffer[pCommandNext->uLenght] = Buf[i];
                pCommandNext->uLenght += 1;
            }else{
                uStateRx = UC_STATE_ERROR;
            }
        }
    }
};

/* @breif: Инициализация
*/
void usbcan_Init(){
    // xInUsbQueue = xQueueCreate(LENGTH_USB_IN_QUEUE, sizeof(LawicelMessage*));
    xCanTxQueue = xQueueCreate(100, sizeof(CanTxMsgTypeDef*));
    xInMsgProcess = xQueueCreate(40, sizeof(StructInMsg));
    xAccessTxBuffer = xSemaphoreCreateMutex();
    xAccessTxCan = xSemaphoreCreateCounting(3, 3);

    // В начале буфер пустой
    TxBufferLength = 0;
    
    // В начале ни куда не указыаает
    pCommandNext = NULL;
    uStateRx = UC_STATE_OK;

    // Инициализируем дескриптор CAN
    hCan.Instance = CAN;    
    hCan.Init.Prescaler = 9;
    hCan.Init.Mode = CAN_MODE_NORMAL;
    hCan.Init.SJW = CAN_SJW_3TQ;                    // Resynchronization Jump Width
    hCan.Init.BS1 = CAN_BS1_2TQ;
    hCan.Init.BS2 = CAN_BS2_1TQ;

    hCan.Init.TTCM = DISABLE;
    hCan.Init.ABOM = DISABLE;
    hCan.Init.AWUM = DISABLE;
    hCan.Init.NART = DISABLE;
    hCan.Init.RFLM = DISABLE;
    hCan.Init.TXFP = DISABLE;

    hCan.pRxMsg = &sRxMsg;

    // Настройка фильтра - Принимаем все пакеты
    sFilterConfig.FilterNumber = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = 0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.BankNumber = 14;

    //
    canStatus = USBCAN_CH_CLOSE;

    // Читаем настройки
    uTimeStampState = settings_get_TimeStampState();
    if (uTimeStampState == 1) {
        start_timer_time_stamp();
    }
    TerminalResistorControl( settings_get_StateTerminalResistor());
    PowerCANBusDeviceControl( settings_get_StatePowerCANBusDevice());

    StatusFlagsRegister = 0;
};

/* @breif: Задача по отправке сообщений на USB Host
*/
void vSendUsbHost(void *pvParametrs){
    USBD_CDC_HandleTypeDef *hcdc;

    for(;;){
        hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
        if (hcdc->TxState != 0){
            vTaskDelay(1/portTICK_PERIOD_MS);
            continue;
        }

        xSemaphoreTake(xAccessTxBuffer, portMAX_DELAY);
        
        if (TxBufferLength > 0){
            memcpy(TxBufferTwo, TxBuffer, TxBufferLength);

            USBD_CDC_SetTxBuffer(&hUsbDeviceFS, TxBufferTwo, TxBufferLength);
            USBD_CDC_TransmitPacket(&hUsbDeviceFS);

            TxBufferLength = 0;
        }
        
        xSemaphoreGive(xAccessTxBuffer);

        vTaskDelay(1/portTICK_PERIOD_MS);
    };

    vTaskDelete(NULL);
}

/* @breif: Задача по обработки сообщений поступивших от переферии (USB, CAN)
*/
void vUsbInProcess(void *pvParametrs){
    portBASE_TYPE xStatus;
    LawicelMessage *lMsg;
    stRxMsgTimeStamp* pRxMsgTSCan;
    StructInMsg inMsgMain;

    for(;;){
        xStatus = xQueueReceive(xInMsgProcess, &inMsgMain, portMAX_DELAY);
        if (xStatus){
            switch (inMsgMain.type){
                case INMSG_USBIN:
                    lMsg = inMsgMain.PointerData;
                    usbcanCmdProccess(lMsg->pBuffer, lMsg->uLenght);
                    free(lMsg);
                    break;
                case INMSG_CANIN:
                    pRxMsgTSCan = inMsgMain.PointerData;
                    usbcanRxCanProcess(pRxMsgTSCan);
                    free(pRxMsgTSCan);
                    break;
                default:
                    break;
            }
        }
    }

    vTaskDelete(NULL);
};

/* @breif: Задача по отправке сообщений в CAN шину
*/
void vCanTxSend(void *pvParametrs)
{
    portBASE_TYPE xStatus;
    CanTxMsgTypeDef* pCanTxMsg;

    for(;;){
        xStatus = xQueueReceive(xCanTxQueue, &pCanTxMsg, portMAX_DELAY);

        hCan.pTxMsg = pCanTxMsg;
        
        xSemaphoreTake(xAccessTxCan, portMAX_DELAY);

        HAL_CAN_Transmit_IT(&hCan);

        free(pCanTxMsg);
        hCan.pTxMsg = NULL;
    }

    vTaskDelete(NULL);
}

/* @breif: Записывает данные в буфер для отправки на USB хост
*/
uint8_t usbcanWriteToBuffer(uint8_t *Buf, uint8_t Length){
    xSemaphoreTake(xAccessTxBuffer, portMAX_DELAY);
    
    if ( (TxBufferLength + Length) > TX_DATA_LENGTH_BUFFER){
        return -1;
    }

    for(uint8_t i=0; i < Length; i++)
    {
        TxBuffer[TxBufferLength] = Buf[i];
        TxBufferLength += 1;
    }
    
    xSemaphoreGive(xAccessTxBuffer);
    
    return 0;
}

/* @breif: Обработка команды от USB хоста
*/
void usbcanCmdProccess(uint8_t *Buf, uint8_t Length)
{
    uint8_t tmp_buf[40];
    uint8_t tmp_len;
    uint8_t tmp, t2;
    uint32_t l1;
    CanTxMsgTypeDef* pCanTxMsg;
    BaseType_t xStatus;
    float ft2;

    uint32_t SerialNumber = (*(__IO uint32_t*) 0x0800F000);

    if (Length <= 0){
        return;
    }

    switch(Buf[0]) {
        case 'V':
            tmp = sprintf(tmp_buf, "V%s", LAWICEL_VERSION);
            tmp_len = strlen(tmp_buf);
            tmp_buf[tmp] = LAWICEL_END;
            usbcanWriteToBuffer(tmp_buf, tmp_len);
            break;
        
        case 'N':                                           // Read Serial Number
            tmp = sprintf(tmp_buf, "N%04x", SerialNumber);
            tmp_len = strlen(tmp_buf);
            tmp_buf[tmp] = LAWICEL_END;
            usbcanWriteToBuffer(tmp_buf, tmp_len);
            break;
        
        case 'O':                                           // Open Normal Mode
            if (canStatus != USBCAN_CH_CLOSE){
                tmp_buf[0] = LAWICEL_ERROR;
                usbcanWriteToBuffer(tmp_buf, 1);
                break;
            }
            
            if (can_open_channel(CAN_MODE_NORMAL) == HAL_OK )
            {
                canStatus = USBCAN_CH_OPEN;
                SetLed1Mode();
                tmp_buf[0] = LAWICEL_OK;
                usbcanWriteToBuffer(tmp_buf, 1);
            }else{
                tmp_buf[0] = LAWICEL_ERROR;
                usbcanWriteToBuffer(tmp_buf, 1);
            }
            break;
        
        case 'L':                                           // Open Silent Monde
            if (canStatus != USBCAN_CH_CLOSE){
                tmp_buf[0] = LAWICEL_ERROR;
                usbcanWriteToBuffer(tmp_buf, 1);
                break;
            }

            if (can_open_channel(CAN_MODE_SILENT) == HAL_OK )
            {
                canStatus = USBCAN_CH_OPEN;
                SetLed1Mode();
                tmp_buf[0] = LAWICEL_OK;
                usbcanWriteToBuffer(tmp_buf, 1);
            }else{
                tmp_buf[0] = LAWICEL_ERROR;
                usbcanWriteToBuffer(tmp_buf, 1);
            }
            break;
        
        case 'l':                                           // Open Loopback Mode
            if (canStatus != USBCAN_CH_CLOSE){
                tmp_buf[0] = LAWICEL_ERROR;
                usbcanWriteToBuffer(tmp_buf, 1);
                break;
            }

            if (can_open_channel(CAN_MODE_LOOPBACK) == HAL_OK )
            {
                canStatus = USBCAN_CH_OPEN;
                SetLed1Mode();
                tmp_buf[0] = LAWICEL_OK;
                usbcanWriteToBuffer(tmp_buf, 1);
            }else{
                tmp_buf[0] = LAWICEL_ERROR;
                usbcanWriteToBuffer(tmp_buf, 1);
            }
            break;
        
        case 'C':
            if (canStatus != USBCAN_CH_OPEN){
                tmp_buf[0] = LAWICEL_ERROR;
                usbcanWriteToBuffer(tmp_buf, 1);
                break;
            }
            if (HAL_CAN_DeInit(&hCan) == HAL_OK){
                canStatus = USBCAN_CH_CLOSE;
                SetLed1Mode();
                tmp_buf[0] = LAWICEL_OK;
                usbcanWriteToBuffer(tmp_buf, 1);
            }else{
                tmp_buf[0] = LAWICEL_ERROR;
                usbcanWriteToBuffer(tmp_buf, 1);
            }
            break;
        
        case 'S':
            if  ((Length != 2) 
                    || (canStatus != USBCAN_CH_CLOSE)
                )
            {
                tmp_buf[0] = LAWICEL_ERROR;
                usbcanWriteToBuffer(tmp_buf, 1);
                break;
            }
            if (can_switch_speed(Buf[1]) == 0){
                tmp_buf[0] = LAWICEL_OK;
                usbcanWriteToBuffer(tmp_buf, 1);
            }else{
                tmp_buf[0] = LAWICEL_ERROR;
                usbcanWriteToBuffer(tmp_buf, 1);
            }
            break;

        case 't':
            if  ((Length < 5)
                    || (canStatus != USBCAN_CH_OPEN)
                )
            {
                tmp_buf[0] = LAWICEL_ERROR;
                usbcanWriteToBuffer(tmp_buf, 1);
                break;
            }
            t2 = Buf[4] - 0x30;                          // DLC | 0x30 - ascii code of 0
            if  ( (t2 < 0) 
                    || (t2 > 8)
                    || (Length != (5 + (t2 * 2)))
                )
            {
                tmp_buf[0] = LAWICEL_ERROR;
                usbcanWriteToBuffer(tmp_buf, 1);
                break;
            }
            l1 = htol(&Buf[1], 3);                       // StdId
            
            if (l1 > 0x7FF) {
                tmp_buf[0] = LAWICEL_ERROR;
                usbcanWriteToBuffer(tmp_buf, 1);
                break;
            }

            pCanTxMsg = malloc(sizeof(CanTxMsgTypeDef));
            pCanTxMsg->StdId = l1;
            pCanTxMsg->ExtId = 0x01;
            pCanTxMsg->IDE = CAN_ID_STD;
            pCanTxMsg->RTR = CAN_RTR_DATA;
            pCanTxMsg->DLC = t2;
            for (uint8_t i = 0; i < t2; i++)
                pCanTxMsg->Data[i] = (uint8_t) htol(&Buf[5 + (i * 2)], 2);
            
            // Send to Queue
            xStatus = xQueueSendToBack(xCanTxQueue, &pCanTxMsg, portMAX_DELAY);
            if (xStatus != pdPASS) {                // Отправка в очередь не получилась
                free(pCanTxMsg);
                tmp_buf[0] = LAWICEL_ERROR;
                usbcanWriteToBuffer(tmp_buf, 1);
            }else{
                // Все хорошо
                tmp_buf[0] = 'z';
                tmp_buf[1] = LAWICEL_OK;
                usbcanWriteToBuffer(tmp_buf, 2);
            }

            break;

        case 'T':
            if  ((Length < 10)
                    || (canStatus != USBCAN_CH_OPEN)
                )
            {
                tmp_buf[0] = LAWICEL_ERROR;
                usbcanWriteToBuffer(tmp_buf, 1);
                break;
            }
            t2 = Buf[9] - 0x30;                          // Get DLC
            if  ( (t2 < 0) 
                    || (t2 > 8)
                    || (Length != (10 + (t2 * 2)))
                )
            {
                tmp_buf[0] = LAWICEL_ERROR;
                usbcanWriteToBuffer(tmp_buf, 1);
                break;
            }
            l1 = htol(&Buf[1], 8);                       // Get ExtId
            if (l1 > 0x1FFFFFFF) {
                tmp_buf[0] = LAWICEL_ERROR;
                usbcanWriteToBuffer(tmp_buf, 1);
                break;
            }

            pCanTxMsg = malloc(sizeof(CanTxMsgTypeDef));
            pCanTxMsg->StdId = 0x01;
            pCanTxMsg->ExtId = l1;
            pCanTxMsg->IDE = CAN_ID_EXT;
            pCanTxMsg->RTR = CAN_RTR_DATA;
            pCanTxMsg->DLC = t2;
            for (uint8_t i = 0; i < t2; i++)
                pCanTxMsg->Data[i] = (uint8_t) htol(&Buf[10 + (i * 2)], 2);
            
            // Send to Queue
            xStatus = xQueueSendToBack(xCanTxQueue, &pCanTxMsg, portMAX_DELAY);
            if (xStatus != pdPASS) {                // Отправка в очередь не получилась
                free(pCanTxMsg);
                tmp_buf[0] = LAWICEL_ERROR;
                usbcanWriteToBuffer(tmp_buf, 1);
            }else{
                // Все хорошо
                tmp_buf[0] = 'Z';
                tmp_buf[1] = LAWICEL_OK;
                usbcanWriteToBuffer(tmp_buf, 2);
            }

            break;
        
        case 'r':
            if  ((Length != 5)
                    || (canStatus != USBCAN_CH_OPEN)
                )
            {
                tmp_buf[0] = LAWICEL_ERROR;
                usbcanWriteToBuffer(tmp_buf, 1);
                break;
            }
            t2 = Buf[4] - 0x30;                          // DLC | 0x30 - ascii code of 0
            if  ( (t2 < 0) 
                    || (t2 > 8)
                )
            {
                tmp_buf[0] = LAWICEL_ERROR;
                usbcanWriteToBuffer(tmp_buf, 1);
                break;
            }
            l1 = htol(&Buf[1], 3);                       // StdId
            
            if (l1 > 0x7FF) {
                tmp_buf[0] = LAWICEL_ERROR;
                usbcanWriteToBuffer(tmp_buf, 1);
                break;
            }

            pCanTxMsg = malloc(sizeof(CanTxMsgTypeDef));
            pCanTxMsg->StdId = l1;
            pCanTxMsg->ExtId = 0x01;
            pCanTxMsg->IDE = CAN_ID_STD;
            pCanTxMsg->RTR = CAN_RTR_REMOTE;
            pCanTxMsg->DLC = t2;
            
            // Send to Queue
            xStatus = xQueueSendToBack(xCanTxQueue, &pCanTxMsg, portMAX_DELAY);
            if (xStatus != pdPASS) {                // Отправка в очередь не получилась
                free(pCanTxMsg);
                tmp_buf[0] = LAWICEL_ERROR;
                usbcanWriteToBuffer(tmp_buf, 1);
            }else{
                // Все хорошо
                tmp_buf[0] = 'z';
                tmp_buf[1] = LAWICEL_OK;
                usbcanWriteToBuffer(tmp_buf, 2);
            }

            break;
        
        case 'R':
            if  ((Length != 10)
                    || (canStatus != USBCAN_CH_OPEN)
                )
            {
                tmp_buf[0] = LAWICEL_ERROR;
                usbcanWriteToBuffer(tmp_buf, 1);
                break;
            }
            t2 = Buf[9] - 0x30;                          // Get DLC
            if  ( (t2 < 0) 
                    || (t2 > 8)
                )
            {
                tmp_buf[0] = LAWICEL_ERROR;
                usbcanWriteToBuffer(tmp_buf, 1);
                break;
            }
            l1 = htol(&Buf[1], 8);                       // Get ExtId
            if (l1 > 0x1FFFFFFF) {
                tmp_buf[0] = LAWICEL_ERROR;
                usbcanWriteToBuffer(tmp_buf, 1);
                break;
            }

            pCanTxMsg = malloc(sizeof(CanTxMsgTypeDef));
            pCanTxMsg->StdId = 0x01;
            pCanTxMsg->ExtId = l1;
            pCanTxMsg->IDE = CAN_ID_EXT;
            pCanTxMsg->RTR = CAN_RTR_REMOTE;
            pCanTxMsg->DLC = t2;
            
            // Send to Queue
            xStatus = xQueueSendToBack(xCanTxQueue, &pCanTxMsg, portMAX_DELAY);
            if (xStatus != pdPASS) {                // Отправка в очередь не получилась
                free(pCanTxMsg);
                tmp_buf[0] = LAWICEL_ERROR;
                usbcanWriteToBuffer(tmp_buf, 1);
            }else{
                // Все хорошо
                tmp_buf[0] = 'Z';
                tmp_buf[1] = LAWICEL_OK;
                usbcanWriteToBuffer(tmp_buf, 2);
            }

            break;
        
        case 'Z':
            if  ((Length != 2)
                    || (canStatus != USBCAN_CH_CLOSE)
                )
            {
                tmp_buf[0] = LAWICEL_ERROR;
                usbcanWriteToBuffer(tmp_buf, 1);
                break;
            }

            t2 = Buf[1] - 0x30;

            if ((t2 != 0) && (t2 != 1)) {
                tmp_buf[0] = LAWICEL_ERROR;
                usbcanWriteToBuffer(tmp_buf, 1);
                break;
            }

            if (t2 == 0){
                // Disable
                stop_timer_time_stamp();
                uTimeStampState = 0;
                settings_save();
            }
            if (t2 == 1){
                // Enable
                start_timer_time_stamp();
                uTimeStampState = 1;
                settings_save();
            }

            // Все хорошо
            tmp_buf[0] = LAWICEL_OK;
            usbcanWriteToBuffer(tmp_buf, 1);
            break;
        
        case 'F':
            if  (canStatus != USBCAN_CH_OPEN)
            {
                tmp_buf[0] = LAWICEL_ERROR;
                usbcanWriteToBuffer(tmp_buf, 1);
                break;
            }

            tmp = sprintf(tmp_buf, "F%02x", StatusFlagsRegister);
            tmp_buf[tmp] = LAWICEL_END;
            usbcanWriteToBuffer(tmp_buf, tmp);
            
            StatusFlagsRegister = 0;

            break;
        
        case 'm':                                           // Только для обратной совместимости, не работает
            tmp_buf[0] = LAWICEL_END;
            usbcanWriteToBuffer(tmp_buf, 1);
            break;
        
        case 'M':                                           // Только для обратной совместимости, не работает
            tmp_buf[0] = LAWICEL_END;
            usbcanWriteToBuffer(tmp_buf, 1);
            break;
        
        case 'f':                                           // Доп команды.
            if  ((Length > 3)
                    || (Length < 2)
                )
            {
                tmp_buf[0] = LAWICEL_ERROR;
                usbcanWriteToBuffer(tmp_buf, 1);
                break;
            }

            switch(Buf[1]){
                case 0x30 + 1:      // Функция 1 - терминальный резистор
                    if (Length == 3) {
                        if (Buf[2] == 0x31)
                            TerminalResistorControl(ENABLE);
                        else
                            TerminalResistorControl(DISABLE);
                        
                        settings_save();
                    }
                    
                    tmp = sprintf(tmp_buf, "f%01x", StateTerminalResistor);
                    tmp_buf[tmp] = LAWICEL_END;
                    usbcanWriteToBuffer(tmp_buf, tmp);
                    break;
                
                case 0x30 + 2:
                    if (Length == 3) {
                        if (Buf[2] == 0x31)
                            PowerCANBusDeviceControl(ENABLE);
                        else
                            PowerCANBusDeviceControl(DISABLE);
                        
                        settings_save();
                    }
                    tmp = sprintf(tmp_buf, "f%01x", StatePowerCANBusDevice);
                    tmp_buf[tmp] = LAWICEL_END;
                    usbcanWriteToBuffer(tmp_buf, tmp);
                    break;

                default:
                    tmp_buf[0] = LAWICEL_ERROR;
                    usbcanWriteToBuffer(tmp_buf, 1);
            }

            break;
        
        case 'p':
            if  (Length != 1)
            {
                tmp_buf[0] = LAWICEL_ERROR;
                usbcanWriteToBuffer(tmp_buf, 1);
                break;
            }

            HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1,500);
            l1 = HAL_ADC_GetValue(&hadc1);
            HAL_ADC_Stop(&hadc1);

            ft2 = (l1 * 3.3 / 4095) * 2;

            tmp = sprintf(tmp_buf, "p%.2f", ft2);
            tmp_buf[tmp] = LAWICEL_END;
            usbcanWriteToBuffer(tmp_buf, tmp);
            break;

        default:
            tmp_buf[0] = LAWICEL_ERROR;
            usbcanWriteToBuffer(tmp_buf, 1);
            break;
    };
};

/* @breif: Обрабатывает сообщения полученные по CAN и отправляет их на USB хост
*/
void usbcanRxCanProcess(stRxMsgTimeStamp* pRxMsgTS){
    CanRxMsgTypeDef*    pRxMsg;
    uint8_t             tmp_buf[40];
    uint8_t             len_buf = 0;

    pRxMsg = &pRxMsgTS->RxMsg;

    // Data
    if (pRxMsg->IDE == CAN_ID_STD){
        // Идентификатор стандартный (11bit)
        tmp_buf[0] = (pRxMsg->RTR == CAN_RTR_DATA) ? 't' : 'r';
        ltoh(pRxMsg->StdId, &tmp_buf[1], 3);
        tmp_buf[4] = 0x30 + pRxMsg->DLC;
        len_buf = 5;
    }else{
        // Идентификатор расширенный (29bit)
        tmp_buf[0] = (pRxMsg->RTR == CAN_RTR_DATA) ? 'T' : 'R';
        ltoh(pRxMsg->ExtId, &tmp_buf[1], 8);
        tmp_buf[9] = 0x30 + pRxMsg->DLC;
        len_buf = 10;
    }

    if (pRxMsg->RTR == CAN_RTR_DATA) {
        for (uint8_t i = 0; i < pRxMsg->DLC; i++){
            ltoh(pRxMsg->Data[i], &tmp_buf[len_buf], 2);
            len_buf += 2;
        }
    }

    // TimeStamp
    if (uTimeStampState){
        ltoh(pRxMsgTS->TimeStamp, &tmp_buf[len_buf], 4);
        len_buf += 4;
    }

    tmp_buf[len_buf] = LAWICEL_END;
    len_buf += 1;
    usbcanWriteToBuffer(tmp_buf, len_buf);
}

/* @breif: Устанавливает скорость шины CAN из набора (Sx)
    Speed   PrS BS1 BS2
    1Mb/s   9	2	1
    800Kb/s 9	3	1
    500Kb/s 9	5	2
    250Kb/s 9	11	4
    125Kb/s 18	12	3
    100Kb/s 18	15	4
    50Kb/s  36	15	4
    20Kb/s  90	15	4
    10Kb/s  180	15	4
*/
uint8_t can_switch_speed(uint8_t num){
    switch(num){
        case '0':
            hCan.Init.Prescaler = 180;
            hCan.Init.BS1 = CAN_BS1_15TQ;
            hCan.Init.BS2 = CAN_BS2_4TQ;
            break;
        
        case '1':
            hCan.Init.Prescaler = 90;
            hCan.Init.BS1 = CAN_BS1_15TQ;
            hCan.Init.BS2 = CAN_BS2_4TQ;
            break;
        
        case '2':
            hCan.Init.Prescaler = 36;
            hCan.Init.BS1 = CAN_BS1_15TQ;
            hCan.Init.BS2 = CAN_BS2_4TQ;
            break;
        
        case '3':
            hCan.Init.Prescaler = 18;
            hCan.Init.BS1 = CAN_BS1_15TQ;
            hCan.Init.BS2 = CAN_BS2_4TQ;
            break;
        
        case '4':
            hCan.Init.Prescaler = 18;
            hCan.Init.BS1 = CAN_BS1_12TQ;
            hCan.Init.BS2 = CAN_BS2_3TQ;
            break;
        
        case '5':
            hCan.Init.Prescaler = 9;
            hCan.Init.BS1 = CAN_BS1_11TQ;
            hCan.Init.BS2 = CAN_BS2_4TQ;
            break;
        
        case '6':
            hCan.Init.Prescaler = 9;
            hCan.Init.BS1 = CAN_BS1_5TQ;
            hCan.Init.BS2 = CAN_BS2_2TQ;
            break;
        
        case '7':
            hCan.Init.Prescaler = 9;
            hCan.Init.BS1 = CAN_BS1_3TQ;
            hCan.Init.BS2 = CAN_BS2_1TQ;
            break;
        
        case '8':
            hCan.Init.Prescaler = 9;
            hCan.Init.BS1 = CAN_BS1_2TQ;
            hCan.Init.BS2 = CAN_BS2_1TQ;
            break;

        default:
            return -1;
    }
    return 0;
}

/* @breif: Открывает канал в указанном режиме. И настраивает фильтры
*/
HAL_StatusTypeDef can_open_channel(uint32_t CanMode){
    HAL_StatusTypeDef status;

    hCan.Init.Mode = CanMode;

    status = HAL_CAN_Init(&hCan);
    if (status != HAL_OK ){
        return status;
    }

    status = HAL_CAN_ConfigFilter(&hCan, &sFilterConfig);
    if (status != HAL_OK)
    {
        return status;
    }

    StatusFlagsRegister = 0;

    HAL_CAN_Receive_IT(&hCan, CAN_FIFO0);

    return HAL_OK;
}

/* @breif: Конвертирует символы из hex в число
*/
uint32_t htol(uint8_t* buf, uint8_t len){
    uint32_t result = 0;
    uint8_t c;

    for (uint8_t i = 0; i < len; i++){
        c = buf[len - 1 - i];
        
        if ((c >= 0x41) && (c <=0x46))
        {
            result += ((c - 0x41) + 10) * pow(16, i);
            continue;
        }

        if ( (c>=0x61) && (c <= 0x66)){
            result += ((c - 0x61) + 10) * pow(16, i);
            continue;
        }

        result += (c - 0x30) * pow(16,i);
    }

    return result;
}

/* @breif: Конвертирует число в hex строку
   Если возвращает не ноль, то не все символы поместились в буфер
*/
uint8_t ltoh(uint32_t l, uint8_t* buf, uint8_t len){
    uint32_t    cel;
    uint8_t     seg;
    uint8_t     index = 0;
    
    while ( (index < len) && (l != 0) ) {
        seg = l % 16;    
        buf[ len - index - 1] = (seg <= 9) ? 0x30 + seg : 0x41 + seg - 10;
    
        l = (int)(l / 16);
        index += 1;
    }
    // Добиваем нулями старшие разряды
    while(index < len){
        buf[ len - index - 1] = 0x30;
        index += 1;
    }

    return l;
}

/* @breif: Коллбэк для CAN
*/
void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan){
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(xAccessTxCan, &pxHigherPriorityTaskWoken);
}

void HAL_CAN_RxCpltCallback ( CAN_HandleTypeDef * hcan){
    // Сделать проверка на FULL FIFO0 (FULL0)

    StructInMsg inMsgTmp;
    stRxMsgTimeStamp* tmpRxMsgTS;
    BaseType_t xStatus;
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;

    if (hcan->ErrorCode == HAL_CAN_ERROR_NONE){
        tmpRxMsgTS = malloc(sizeof(stRxMsgTimeStamp));
        memcpy(&tmpRxMsgTS->RxMsg, hcan->pRxMsg, sizeof(CanRxMsgTypeDef));
        tmpRxMsgTS->TimeStamp = uwTickTimer6;

        inMsgTmp.type = INMSG_CANIN;
        inMsgTmp.PointerData = tmpRxMsgTS;

        xStatus = xQueueSendToBackFromISR(xInMsgProcess, &inMsgTmp, &pxHigherPriorityTaskWoken); 
        if (xStatus != pdPASS){
            free(tmpRxMsgTS);
            // Overrun - устанавливаем флаг
            StatusFlagsRegister = StatusFlagsRegister | STATUS_FLAGS_RX_OVERRUN_MASK;
            SetLed1Mode();
        }
    }else if (hcan->ErrorCode == HAL_CAN_ERROR_FOV0){
        StatusFlagsRegister = StatusFlagsRegister | STATUS_FLAGS_RX_OVERRUN_MASK;
        SetLed1Mode();
    }

    HAL_CAN_Receive_IT(hcan, CAN_FIFO0);
}

void HAL_CAN_ErrorCallback ( CAN_HandleTypeDef * hcan){
    if (hcan->ErrorCode == HAL_CAN_ERROR_EPV){
        StatusFlagsRegister = StatusFlagsRegister | STATUS_FLAGS_ERROR_PASSIVE;
        SetLed1Mode();
    }
    if (hcan->ErrorCode == HAL_CAN_ERROR_BOF){
        StatusFlagsRegister = StatusFlagsRegister | STATUS_FLAGS_BUS_OFF;
        SetLed1Mode();
    }
}

/* @breif: Запускает таймер для штампов времени
*/
void start_timer_time_stamp(){
    uwTickTimer6 = 0;
    HAL_TIM_Base_Start_IT(&htim6);
}

/* @breif: Останавливает таймер для штампов времени
*/
void stop_timer_time_stamp(){
    HAL_TIM_Base_Stop_IT(&htim6);
}

// ============================

/*  @breif  Управление терминальным резистором
    @param  state - может быть ENABLE/DISABLE
*/
void TerminalResistorControl(FunctionalState state){
    StateTerminalResistor = state;

    if (state == ENABLE){
        HAL_GPIO_WritePin(PIN_CAN_TERMINATION_EN_GPIO_Port, PIN_CAN_TERMINATION_EN_Pin, GPIO_PIN_SET);
    }else{
        HAL_GPIO_WritePin(PIN_CAN_TERMINATION_EN_GPIO_Port, PIN_CAN_TERMINATION_EN_Pin, GPIO_PIN_RESET);
    }
}

/* @breif   Управление питанием CAN шины
   @param   state - может быть ENABLE/DISABLE
*/
void PowerCANBusDeviceControl(FunctionalState state){
    StatePowerCANBusDevice = state;

    if (state == ENABLE){
        HAL_GPIO_WritePin(PIN_CAN_POWER_EN_GPIO_Port, PIN_CAN_POWER_EN_Pin, GPIO_PIN_SET);
    }else{
        HAL_GPIO_WritePin(PIN_CAN_POWER_EN_GPIO_Port, PIN_CAN_POWER_EN_Pin, GPIO_PIN_RESET);
    }
}