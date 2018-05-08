#include "settings.h"

/* @breif: Сохранение настроек
*/
void settings_save(){
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PAGEError = 0;
    uint32_t t32;

    HAL_FLASH_Unlock();

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.NbPages =1;
    EraseInitStruct.PageAddress = 0x0800F800;

    HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);

    //
    t32 = uTimeStampState;
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x0800F800, t32);

    //
    t32 = (uint32_t) StateTerminalResistor;
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x0800F804, t32);

    //
    t32 = (uint32_t) StatePowerCANBusDevice;
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x0800F808, t32);

    HAL_FLASH_Lock();
}

/* @breif: Получить значение для переменной TimeStampState
*/
uint8_t settings_get_TimeStampState(){
    uint32_t t32;
    
    t32 = (*(__IO uint32_t*) 0x0800F800);
    
    return (uint8_t) t32;
}

FunctionalState settings_get_StateTerminalResistor(){
    uint32_t t32;
    
    t32 = (*(__IO uint32_t*) 0x0800F804);
    
    return (FunctionalState) t32;
}

FunctionalState settings_get_StatePowerCANBusDevice(){
    uint32_t t32;
    
    t32 = (*(__IO uint32_t*) 0x0800F808);
    
    return (FunctionalState) t32;
}
