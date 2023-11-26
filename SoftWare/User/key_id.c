#include "key_id.h"
#include "communication.h"
#include <gpio.h>
#include <stm32f1xx_hal_flash.h>

uint8_t ID = 1;

static uint8_t IDLedFlag = 0;
static uint8_t i = 0;
static uint8_t tempID = 0;

static void FLASH_WriteByte(uint32_t addr ,uint16_t *flashdata,uint8_t len);
static void Read_Flash(uint32_t addr,uint16_t* buffer, uint8_t len);

/*	id led timer callback function	*/
void IDLEDAndIDSet(void const * argument)
{
    if(IDLedFlag == 0)
    {
        if (i <= 2*ID)
        {
            HAL_GPIO_TogglePin(IDLED_GPIO_Port, IDLED_Pin);
        }
        else if (i == 2*ID + 4)
        {
            i = -1;
        }
    }
    else
    {
        if (i > 3)
        {
            i = -1;
            IDLedFlag = 0;
            WriteID(tempID);
            CAN_Filter_configure(tempID);
            tempID = 0;
        }
    }
    i++;
}

/*	idkey NVIC handler	*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
    {
        /*  消抖，因为设置ID时不会有其他操作，所以可以直接延时  */
        HAL_Delay(10);
        if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
        {
            IDLedFlag = 1;
            tempID++;
            i = 0;
        }
    }
}

void ID_init(void)
{
    uint16_t id = 0;
    Read_Flash(ID_ADDR, &id, 1);
    if (id == 0 || id > 128)
    {
        id = 1;
    }
    ID = id&0xff;
}

void WriteID(uint8_t id)
{
    ID = id;
    uint16_t temp = id&0x00ff;
    FLASH_WriteByte(ID_ADDR, &temp, 1);
}

/*
功能：向flash中写入数据
参数：addr:地址  flashdata:写入的数据  len:写入长度
返回值：
 */
void FLASH_WriteByte(uint32_t addr ,uint16_t *flashdata,uint8_t len)
{
    int i = 0;
    FLASH_EraseInitTypeDef my_flash;

    HAL_FLASH_Unlock(); //解锁flash

    my_flash.TypeErase = FLASH_TYPEERASE_PAGES; //页擦除
    my_flash.PageAddress = addr; //擦除的页地址
    my_flash.NbPages = 1; //擦除页数

    uint32_t PageError = 0;
    HAL_FLASHEx_Erase(&my_flash, &PageError); //擦除flash
    
    for(i=0; i<len;i++)
    {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + i*2, flashdata[i]);//写入flash(半字节写入
    }
    HAL_FLASH_Lock();//上锁 
}

/*
功能：读取FLASH中指定地址的值
参数：addr：起始地址 buffer：数据缓冲区 len：读取的长度
返回值：无
 */
void Read_Flash(uint32_t addr,uint16_t* buffer, uint8_t len)
{
	uint8_t i=0;
	
	HAL_FLASH_Unlock();//解锁flash
	
	for(i=0;i<len;i++)
	{
		buffer[i] = *(uint16_t *)(addr + i*2);    //读取flash地址中的值
	}
	
	HAL_FLASH_Lock();//上锁
}


