/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-07-11     RT-Thread    first version
 */
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "bsp_i2c_OLED.h"
#include "bsp_NRF24L01.h"
#include "gun_info.h"
#include "stmflash.h"


#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

uint8_t power_stat;
uint8_t soundcard_play_txbuf[]={0x7E,0x03,0x11,0x12,0xEF};

rt_timer_t oled_tm;
rt_timer_t send_tm;
uint32_t send_shoot_count; //按下扳机后再取10次点再关灯
uint8_t send_release_count; //LOADED时 松开扳机30ms才算松开
uint8_t send_press_count; //压30ms才算压着

uint8_t gun_sound;

struct Flash_Struct{
    uint32_t shootnum;
    uint32_t serialnum;
    uint8_t oled_serialnum;
}flash_struct;

struct Sem_Struct{
    struct rt_semaphore soundcard_sem;
    struct rt_semaphore motor_sem;
    struct rt_semaphore nrf_sem;

    //pause
    rt_sem_t loaded_pause_sem;
    rt_sem_t serialnum_pause_sem;

} sem_struct;

Thread_Struct thread_struct;

uint8_t* int2ascii(uint32_t val);
uint8_t* int2char(uint32_t val);

void KEYON_th_entry(void*parameter)
{
    while(1){
        if(thread_struct.oled_struct.key2_stat ==KEY2_STAT_OFF){
            continue;
        }

        if((HAL_GPIO_ReadPin(KEYON_GPIO, KEYON_PIN)==GPIO_PIN_RESET)&&(power_stat==0)){
            rt_thread_mdelay(50);
            if(HAL_GPIO_ReadPin(KEYON_GPIO, KEYON_PIN)==GPIO_PIN_RESET){
                power_stat=1;
                HAL_GPIO_WritePin(PWR_EN_GPIO, PWR_EN_PIN, GPIO_PIN_SET);
                continue;
            }
        }

        if((HAL_GPIO_ReadPin(KEYON_GPIO, KEYON_PIN)==GPIO_PIN_SET)&&(power_stat==1)){
            power_stat=2;
            continue;  //必须使得按钮松开过，再按才产生关机，防止开机一直按着会直接跳关机
        }
        if((HAL_GPIO_ReadPin(KEYON_GPIO, KEYON_PIN)==GPIO_PIN_RESET)&&(power_stat==2)){
            rt_thread_mdelay(3000);
            if(HAL_GPIO_ReadPin(KEYON_GPIO, KEYON_PIN)==GPIO_PIN_RESET){
                power_stat=3;
                HAL_GPIO_WritePin(PWR_EN_GPIO, PWR_EN_PIN, GPIO_PIN_RESET);
                continue;
            }
        }
    }
}

void SOUNDCARD_th_entry(void*parameter)
{
    while(1){
        rt_sem_take(&sem_struct.soundcard_sem, RT_WAITING_FOREVER);
        HAL_UART_Transmit_IT(&huart1, soundcard_play_txbuf, 5);
    }
}

void MOTOR_th_entry(void*parameter)
{
    while(1){
        rt_sem_take(&sem_struct.motor_sem, RT_WAITING_FOREVER);
        HAL_GPIO_WritePin(MOTOR_GPIO, MOTOR_PIN, GPIO_PIN_SET);
        rt_thread_mdelay(100);
        HAL_GPIO_WritePin(MOTOR_GPIO, MOTOR_PIN, GPIO_PIN_RESET);
    }
}

void REDL_th_entry(void*parameter)
{
    while(1){
        switch(thread_struct.redl_struct.redl_stat){
            case(REDL_ALWAYS_OFF):{
                REDL_OFF();
                break;
            }
            case(REDL_ALWAYS_ON):{
                REDL_ON();
                break;
            }
            case(REDL_TRIGGER_ON):{
                if((int)thread_struct.pt_struct.press_val>=1){
                    REDL_ON();
                }
                else{
                    REDL_OFF();
                }
                break;
            }
        }
    }
}

void oled_tm_callback(void *parameter)
{
    OLED_Clear();
    thread_struct.oled_struct.oled_screen.state = OLED_STAT_START;
    thread_struct.oled_struct.oled_screen.screen = OLED_SCREEN_SLEEP;
}

void SELECT_th_entry(void*parameter)
{
    while(1){
//        if(rt_sem_take(sem_struct.loaded_pause_sem, 0)==RT_EOK){
//            rt_thread_suspend(thread_struct.select_struct.select_th);
//            rt_schedule();
//        }

//        if(HAL_GPIO_ReadPin(KEY2_GPIO, KEY2_PIN)==GPIO_PIN_SET){//另一按键没有同时按着情况下



        //确认键
        if(thread_struct.select_struct.sel_stat!=SEL_STAT_SERIALNUM){
            if(HAL_GPIO_ReadPin(KEYON_GPIO, KEYON_PIN)==GPIO_PIN_RESET){
                rt_thread_mdelay(10);
                if(HAL_GPIO_ReadPin(KEYON_GPIO, KEYON_PIN)==GPIO_PIN_RESET){
                    while(HAL_GPIO_ReadPin(KEYON_GPIO, KEYON_PIN)==GPIO_PIN_RESET);
                    if((thread_struct.oled_struct.oled_screen.screen == OLED_SCREEN_SERIALNUM)&&(thread_struct.select_struct.sel_stat==SEL_STAT_ELSE)){
                        thread_struct.oled_struct.key2_stat=KEY2_STAT_OFF;
                        thread_struct.select_struct.sel_stat=SEL_STAT_SERIALNUM;
                        thread_struct.select_struct.serialnum_bit=1;
                        OLED_ShowStr(10,5,(uint8_t*)"*    ",2);
//                        rt_sem_release(sem_struct.serialnum_pause_sem);
                        continue;
                    }

                    if(thread_struct.oled_struct.oled_screen.screen == OLED_SCREEN_LASER){
                        if(thread_struct.redl_struct.redl_stat<3){
                            thread_struct.redl_struct.redl_stat++;
                        }
                        else if(thread_struct.redl_struct.redl_stat==3){
                            thread_struct.redl_struct.redl_stat=0;
                        }
                        switch(thread_struct.redl_struct.redl_stat){
                            case REDL_ALWAYS_OFF:{OLED_ShowStr(10,3,(uint8_t*)"ALWAYS OFF",2); break;}
                            case REDL_ALWAYS_ON:{OLED_ShowStr(10,3, (uint8_t*)"ALWAYS ON ",2); break;}
                            case REDL_TRIGGER_ON:{OLED_ShowStr(10,3,(uint8_t*)"TRIGGER ON",2); break;}
                            case REDL_SHOOT_ON:{OLED_ShowStr(10,3,  (uint8_t*)"SHOOT ON  ",2); break;}
                        }
                        continue;
                    }

                    if(thread_struct.oled_struct.oled_screen.screen == OLED_SCREEN_SHOCK){
                        if(thread_struct.motor_struct.motor_stat == MOTOR_SHCOK_ON){
                            thread_struct.motor_struct.motor_stat = MOTOR_SHCOK_OFF;
                        }
                        else if(thread_struct.motor_struct.motor_stat == MOTOR_SHCOK_OFF){
                            thread_struct.motor_struct.motor_stat= MOTOR_SHCOK_ON;
                        }
                        switch(thread_struct.motor_struct.motor_stat){
                            case MOTOR_SHCOK_ON:{OLED_ShowStr(10,3,(uint8_t*)"ON",2); break;}
                            case MOTOR_SHCOK_OFF:{OLED_ShowStr(10,3,(uint8_t*)"OFF",2); break;}
                        }
                    }
                }
            }
        }

            if(thread_struct.select_struct.sel_stat==SEL_STAT_SERIALNUM){
                if(HAL_GPIO_ReadPin(KEYON_GPIO, KEYON_PIN)==GPIO_PIN_RESET){
                    rt_thread_mdelay(1);
                    if(HAL_GPIO_ReadPin(KEYON_GPIO, KEYON_PIN)==GPIO_PIN_RESET){
                        while(HAL_GPIO_ReadPin(KEYON_GPIO, KEYON_PIN)==GPIO_PIN_RESET);
                        if(thread_struct.select_struct.serialnum_bit==1){
                            thread_struct.select_struct.serialnum_bit=2;
                            OLED_ShowStr(10,5,(uint8_t*)" *   ",2);
                        }
                        else if(thread_struct.select_struct.serialnum_bit==2){
                            thread_struct.select_struct.serialnum_bit=3;
                            OLED_ShowStr(10,5,(uint8_t*)"  *  ",2);
                        }
                        else if(thread_struct.select_struct.serialnum_bit==3){
                            thread_struct.select_struct.serialnum_bit=4;
                            OLED_ShowStr(10,5,(uint8_t*)"   * ",2);
                        }
                        else if(thread_struct.select_struct.serialnum_bit==4){
                            thread_struct.select_struct.serialnum_bit=5;
                            OLED_ShowStr(10,5,(uint8_t*)"    *",2);
                        }
                        else if(thread_struct.select_struct.serialnum_bit==5){
                            thread_struct.select_struct.serialnum_bit=0;
                            thread_struct.select_struct.sel_stat=SEL_STAT_ELSE;
                            thread_struct.oled_struct.key2_stat=KEY2_STAT_ON;
                            thread_struct.select_struct.serialnum_sum=0;
                            OLED_ShowStr(10,5,(uint8_t*)"     ",2);
//                            for(int i=0;i<5;i++){
//                                thread_struct.select_struct.serialnum_sum =
//                                        thread_struct.select_struct.serialnum_sum + thread_struct.select_struct.serialnum_val[i];
//                                guninfo_struct.serialnum = thread_struct.select_struct.serialnum_sum;
//                            }

                            thread_struct.select_struct.serialnum_sum=10000*thread_struct.select_struct.serialnum_val[0]+1000*thread_struct.select_struct.serialnum_val[1]+
                                    100*thread_struct.select_struct.serialnum_val[2]+10*thread_struct.select_struct.serialnum_val[3]+thread_struct.select_struct.serialnum_val[4];
                            guninfo_struct.serialnum = thread_struct.select_struct.serialnum_sum;

                            flash_struct.serialnum = guninfo_struct.serialnum;
                            FLASH_EEPROM_Write(ADDRPOS_SERIALNUM, flash_struct.serialnum);

//                            rt_thread_resume(thread_struct.oled_th);
                        }
                    }
                }

                if(HAL_GPIO_ReadPin(KEY2_GPIO, KEY2_PIN)==GPIO_PIN_RESET){
                    rt_thread_mdelay(10);
                    if(HAL_GPIO_ReadPin(KEY2_GPIO, KEY2_PIN)==GPIO_PIN_RESET){
                       while(HAL_GPIO_ReadPin(KEY2_GPIO, KEY2_PIN)==GPIO_PIN_RESET);
                       if(thread_struct.select_struct.serialnum_val[thread_struct.select_struct.serialnum_bit-1]<9){
                           thread_struct.select_struct.serialnum_val[thread_struct.select_struct.serialnum_bit-1]++;
                       }
                       else if(thread_struct.select_struct.serialnum_val[thread_struct.select_struct.serialnum_bit-1]==9){
                           thread_struct.select_struct.serialnum_val[thread_struct.select_struct.serialnum_bit-1]=0;
                       }
                       thread_struct.oled_struct.oled_serialnum_ascii[thread_struct.select_struct.serialnum_bit-1] =
                               0x30|(uint8_t)thread_struct.select_struct.serialnum_val[thread_struct.select_struct.serialnum_bit-1];
//                       OLED_ShowStr(10,3,(uint8_t*)"      ",2);
                       OLED_ShowStr(10,3,thread_struct.oled_struct.oled_serialnum_ascii,2);
                    }
                }

            }


//        }
    }
}

void OLED_th_entry(void*parameter)
{
    while(1){
//        if(rt_sem_take(sem_struct.loaded_pause_sem, 0)==RT_EOK){
//            rt_thread_suspend(thread_struct.oled_th);
//            rt_schedule();
//        }
//        if(rt_sem_take(sem_struct.serialnum_pause_sem, 0)==RT_EOK){
//            rt_thread_suspend(thread_struct.oled_th);
//            rt_schedule();
//        }

        if(thread_struct.oled_struct.key2_stat ==KEY2_STAT_OFF) {
            continue;
        }
        if(HAL_GPIO_ReadPin(KEY2_GPIO, KEY2_PIN)==GPIO_PIN_RESET){ //按下
            rt_thread_mdelay(10);
            if((HAL_GPIO_ReadPin(KEY2_GPIO, KEY2_PIN)==GPIO_PIN_RESET)&&(thread_struct.oled_struct.oled_screen.state == OLED_STAT_START)){
//                    rt_timer_stop(oled_tm);
                    thread_struct.oled_struct.oled_screen.state = OLED_STAT_DYNAMIC;
                    continue;
            }
        }

        if(HAL_GPIO_ReadPin(KEY2_GPIO, KEY2_PIN)==GPIO_PIN_SET){ //松开 //导致延时的部分?
            //rt_thread_mdelay(1);
            if((HAL_GPIO_ReadPin(KEY2_GPIO, KEY2_PIN)==GPIO_PIN_SET)&&(thread_struct.oled_struct.oled_screen.state == OLED_STAT_DYNAMIC)){
                thread_struct.oled_struct.oled_screen.state = OLED_STAT_START;
//                rt_timer_start(oled_tm);
                if(thread_struct.oled_struct.oled_screen.screen == OLED_SCREEN_SLEEP){
                    OLED_Clear();
                    OLED_ShowStr(1,0,(uint8_t*)"BATTERY: 000",2);
                    OLED_ShowStr(1,3,(uint8_t*)"ACGUN SHOOTER",2);
                    thread_struct.oled_struct.oled_screen.screen =OLED_SCREEN_MAIN;
                    continue;
                }
                if(thread_struct.oled_struct.oled_screen.screen == OLED_SCREEN_MAIN){
                    OLED_Clear();
                    OLED_ShowStr(1,0,(uint8_t*)"SHOOTING TIMES",2);
                    OLED_ShowStr(10,3,(uint8_t*)"123456",2);
                    thread_struct.oled_struct.oled_screen.screen =OLED_SCREEN_SHOOTNUM;
                    continue;
                }
                if(thread_struct.oled_struct.oled_screen.screen == OLED_SCREEN_SHOOTNUM){
                    OLED_Clear();
                    OLED_ShowStr(1,0,(uint8_t*)"SERIAL NUMBER",2);
                    OLED_ShowStr(10,3,thread_struct.oled_struct.oled_serialnum_ascii,2);
                    thread_struct.oled_struct.oled_screen.screen =OLED_SCREEN_SERIALNUM;
                    continue;
                }
                if(thread_struct.oled_struct.oled_screen.screen == OLED_SCREEN_SERIALNUM){
                     OLED_Clear();
                     OLED_ShowStr(1,0,(uint8_t*)"LASER MODE",2);
                     switch(thread_struct.redl_struct.redl_stat){
                         case REDL_ALWAYS_OFF:{OLED_ShowStr(10,3,(uint8_t*)"ALWAYS OFF",2); break;}
                         case REDL_ALWAYS_ON:{OLED_ShowStr(10,3, (uint8_t*)"ALWAYS ON ",2); break;}
                         case REDL_TRIGGER_ON:{OLED_ShowStr(10,3,(uint8_t*)"TRIGGER ON",2); break;}
                         case REDL_SHOOT_ON:{OLED_ShowStr(10,3,  (uint8_t*)"SHOOT ON  ",2); break;}
                     }
                     thread_struct.oled_struct.oled_screen.screen =OLED_SCREEN_LASER;
                     continue;
                 }
                if(thread_struct.oled_struct.oled_screen.screen == OLED_SCREEN_LASER){
                     OLED_Clear();
                     OLED_ShowStr(1,0,(uint8_t*)"SHOCK MODE",2);
                     switch(thread_struct.motor_struct.motor_stat){
                         case MOTOR_SHCOK_ON:{OLED_ShowStr(10,3,(uint8_t*)"ON",2); break;}
                         case MOTOR_SHCOK_OFF:{OLED_ShowStr(10,3,(uint8_t*)"OFF",2); break;}
                     }
                     thread_struct.oled_struct.oled_screen.screen =OLED_SCREEN_SHOCK;
                     continue;
                 }
                if(thread_struct.oled_struct.oled_screen.screen == OLED_SCREEN_SHOCK){
                     OLED_Clear();
                     OLED_ShowStr(1,0,(uint8_t*)"VOICE MODE",2);
                     OLED_ShowStr(10,3,(uint8_t*)"ON",2);
                     thread_struct.oled_struct.oled_screen.screen =OLED_SCREEN_VOICE;
                     continue;
                 }
                if(thread_struct.oled_struct.oled_screen.screen == OLED_SCREEN_VOICE){
                    OLED_Clear();
                    OLED_ShowStr(1,0,(uint8_t*)"BATTERY: 000",2);
                    OLED_ShowStr(10,3,(uint8_t*)"ACGUN SHOOTER",2);
                    thread_struct.oled_struct.oled_screen.screen =OLED_SCREEN_MAIN;
                    continue;
                 }

            }
        }
    }
}

void NRF_th_entry(void*parameter)
{
//    while(1){
//        rt_sem_take(&sem_struct.nrf_sem, RT_WAITING_FOREVER);
//        guninfo_struct.press_val = thread_struct.pt_struct.press_val;
//        guninfo_struct.shootflg = SHOOTFLAG_PRESS;
//        if(thread_struct.mrs_struct.mrs_stat == MRS_LOADED){
//            guninfo_send();
//            continue;
//        }
//        else if(thread_struct.mrs_struct.mrs_stat == MRS_SHOOT){
//            guninfo_struct.shootflg=SHOOTFLAG_SHOOT;
//            guninfo_send();
//            thread_struct.mrs_struct.mrs_stat=MRS_END;
//            continue;
//        }
//        else if(thread_struct.mrs_struct.mrs_stat == MRS_END){
//            guninfo_send();
//            continue;
//        }
//        else if(thread_struct.mrs_struct.mrs_stat == MRS_RELEASE){
//            guninfo_struct.shootflg=SHOOTFLAG_RELEASE;
//            guninfo_send();
//            thread_struct.mrs_struct.mrs_stat =MRS_NORMAL;
//            continue;
//        }
//        guninfo_send();
//    }
}

void PT_th_entry(void*parameter)
{
    while(1){
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1,1);
        thread_struct.pt_struct.adc_converted_val=HAL_ADC_GetValue(&hadc1);
        thread_struct.pt_struct.adc_local =
                (float)(thread_struct.pt_struct.adc_converted_val*(3.3/4096));
        thread_struct.pt_struct.adc_val_avg=thread_struct.pt_struct.adc_local;

        thread_struct.pt_struct.press_val=(thread_struct.pt_struct.adc_val_avg/11)*10.2*100;
        thread_struct.pt_struct.press_val_record = thread_struct.pt_struct.press_val;
    }
}

void send_tm_callback(void*parameter)
{
    if(thread_struct.mrs_struct.mrs_stat == MRS_LOADED){
        guninfo_struct.press_val=thread_struct.pt_struct.press_val_record;
        if((int)thread_struct.pt_struct.press_val_record<5){
            if(guninfo_struct.shootflg == SHOOTFLAG_NORMAL){
                return;
            }
            else if(guninfo_struct.shootflg == SHOOTFLAG_PRESS){
                send_release_count++;
                if(send_release_count>10){
                    send_release_count=0;
                    guninfo_struct.shootflg = SHOOTFLAG_RELEASE;
                    guninfo_struct.uwtick = HAL_GetTick();
                    guninfo_send();
                    send_press_count=0; //松开 压计数清零
                    return;
                }
                guninfo_struct.shootflg = SHOOTFLAG_PRESS;
                guninfo_struct.uwtick = HAL_GetTick();
                guninfo_send();
                return;
            }
            return;
        }
        if(send_press_count==0){
            send_press_count++;
        }
        else if(send_press_count<=10){
            send_press_count++;
        }
        else if(send_press_count>10){
            send_release_count=0;
            guninfo_struct.shootflg = SHOOTFLAG_PRESS;
            guninfo_struct.uwtick = HAL_GetTick();
            guninfo_send();
            return;
        }

    }

    else if(thread_struct.mrs_struct.mrs_stat == MRS_SHOOT){
        //SHOOT
        guninfo_struct.press_val=thread_struct.pt_struct.press_val_record;
        guninfo_struct.shootflg=SHOOTFLAG_SHOOT;
        guninfo_struct.uwtick = HAL_GetTick();
        guninfo_send();
        thread_struct.mrs_struct.mrs_stat = MRS_LIGHTOFF;
        return;
    }
    else if(thread_struct.mrs_struct.mrs_stat == MRS_LIGHTOFF){
        send_shoot_count++;
        guninfo_struct.press_val=thread_struct.pt_struct.press_val_record;
        guninfo_struct.shootflg=SHOOTFLAG_PRESS;
        guninfo_struct.uwtick = HAL_GetTick();
        if(send_shoot_count>10){
            UVL_OFF();
            guninfo_struct.shootflg=SHOOTFLAG_LIGHTOFF;
            send_shoot_count=0;
            thread_struct.mrs_struct.mrs_stat = MRS_END;
        }
        guninfo_send();
        return;
    }

    else if(thread_struct.mrs_struct.mrs_stat == MRS_END){
        guninfo_struct.press_val=thread_struct.pt_struct.press_val_record;
        guninfo_struct.shootflg = SHOOTFLAG_PRESS;
        if(thread_struct.pt_struct.press_val_record*10 >=5){
            guninfo_struct.uwtick = HAL_GetTick();
            guninfo_send();
            return;
        }
        else{
            guninfo_struct.shootflg=SHOOTFLAG_RELEASE;
            guninfo_struct.uwtick = HAL_GetTick();
            guninfo_send();
            thread_struct.mrs_struct.mrs_stat = MRS_NORMAL;

//            rt_thread_resume(thread_struct.select_struct.select_th);
//            rt_thread_resume(thread_struct.oled_th);
            return;
        }
    }

}

void MRS_th_entry(void*parameter)
{
    while(1){
        if(thread_struct.mrs_struct.mrs_stat == MRS_NORMAL ){
            if(thread_struct.redl_struct.redl_stat==REDL_SHOOT_ON){
                REDL_OFF();
            }
            if(HAL_GPIO_ReadPin(MRS_GPIO, MRS_PIN)==MRS_GPIO_GET){ //上膛
                UVL_ON();
                thread_struct.mrs_struct.mrs_stat = MRS_LOADED;
                //
//                rt_sem_release(sem_struct.loaded_pause_sem);
            }
        }
        else if(thread_struct.mrs_struct.mrs_stat == MRS_LOADED){
            if(HAL_GPIO_ReadPin(MRS_GPIO, MRS_PIN)==MRS_GPIO_NORMAL){ //击发
                thread_struct.mrs_struct.mrs_stat = MRS_SHOOT;
                rt_sem_release(&sem_struct.soundcard_sem);
                send_shoot_count=0;
                if(thread_struct.motor_struct.motor_stat == MOTOR_SHCOK_ON){
                    rt_sem_release(&sem_struct.motor_sem);
                }
                if(thread_struct.redl_struct.redl_stat==REDL_SHOOT_ON){
                    REDL_ON();
                }
                continue;
            }
        }

    }
}

int main(void)
{
    HAL_Init();
    MX_GPIO_Init();
    MRS_GPIO_Init();
    MX_ADC1_Init();
    MX_I2C1_Init();
    MX_SPI1_Init();
    MX_SPI2_Init();
    MX_USART1_UART_Init();

    uint8_t *p_serialnum1,*p_serialnum2;
//    FLASH_EEPROM_Write(ADDRPOS_SERIALNUM, 12345);
    flash_struct.serialnum = FLASH_EEPROM_Read(ADDRPOS_SERIALNUM);
    p_serialnum1 = int2ascii(flash_struct.serialnum);
    for(int i=0;i<5;i++){
        *(thread_struct.oled_struct.oled_serialnum_ascii+i) = *(p_serialnum1+i);
    }
    p_serialnum2 = int2char(flash_struct.serialnum);
    for(int i=0;i<5;i++){
        *(thread_struct.select_struct.serialnum_val+i) = *(p_serialnum2+i);
    }

    OLED_Init();
    NRF24L01_SPI_Init();
    NRF24L01_TX_Mode();

    OLED_Clear();

    thread_struct.mrs_th_entry = MRS_th_entry;
    thread_struct.mrs_struct.mrs_th = rt_thread_create("mrs_th", thread_struct.mrs_th_entry, NULL, 512, 20, 5);
    rt_thread_startup(thread_struct.mrs_struct.mrs_th);

    thread_struct.keyon_th_entry =KEYON_th_entry;
    thread_struct.keyon_th=rt_thread_create("keyon_th", thread_struct.keyon_th_entry, NULL, 512, 20, 5);
    rt_thread_startup(thread_struct.keyon_th);

    thread_struct.oled_struct.oled_screen.state = OLED_STAT_START;
    thread_struct.oled_struct.oled_screen.screen = OLED_SCREEN_MAIN;
    OLED_ShowStr(1,0,(uint8_t*)"BATTERY: 000",2);
    OLED_ShowStr(1,3,(uint8_t*)"ACGUN SHOOTER",2);

//    flash_struct.serialnum=FLASH_EEPROM_Read(ADDRPOS_SERIALNUM);
//    for(int i=0;i<5;i++){
//        thread_struct.oled_struct.oled_serialnum_ascii[i]=0x30;
//    }
    thread_struct.redl_struct.redl_stat = REDL_ALWAYS_OFF;


//    oled_tm=rt_timer_create("oled_tm", oled_tm_callback, NULL, 10000,RT_TIMER_FLAG_PERIODIC|RT_TIMER_FLAG_SOFT_TIMER);
//    rt_timer_start(oled_tm);

    send_tm = rt_timer_create("send_tm", send_tm_callback, NULL, 3,RT_TIMER_FLAG_PERIODIC|RT_TIMER_FLAG_SOFT_TIMER);
    rt_timer_start(send_tm);

    thread_struct.redl_struct.redl_th_entry =REDL_th_entry;
    thread_struct.redl_struct.redl_th=rt_thread_create("redl_th", thread_struct.redl_struct.redl_th_entry, NULL, 512, 20, 5);
    rt_thread_startup(thread_struct.redl_struct.redl_th);

    rt_sem_init(&sem_struct.soundcard_sem, "soundcard_sem", 0, RT_IPC_FLAG_FIFO);
    thread_struct.soundcard_th_entry =SOUNDCARD_th_entry;
    thread_struct.soundcard_th=rt_thread_create("soundcard_th", thread_struct.soundcard_th_entry, NULL, 512, 15, 5);
    rt_thread_startup(thread_struct.soundcard_th);

//    rt_sem_init(&sem_struct.nrf_sem, "nrf_sem", 0, RT_IPC_FLAG_FIFO);
//    thread_struct.nrf_th_entry = NRF_th_entry;
//    thread_struct.nrf_th = rt_thread_create("nrf_th", thread_struct.nrf_th_entry, NULL, 1024, 20, 5);
//    rt_thread_startup(thread_struct.nrf_th);

    thread_struct.pt_th_entry = PT_th_entry;
    thread_struct.pt_struct.pt_th = rt_thread_create("pt_th", thread_struct.pt_th_entry, NULL, 1024, 20, 5);
    rt_thread_startup(thread_struct.pt_struct.pt_th);

    thread_struct.select_th_entry = SELECT_th_entry;
    thread_struct.select_struct.select_th = rt_thread_create("select_th", thread_struct.select_th_entry, NULL, 1024, 20, 5);
    rt_thread_startup(thread_struct.select_struct.select_th);

    thread_struct.oled_struct.oled_th_entry = OLED_th_entry;
    thread_struct.oled_struct.oled_th=rt_thread_create("oled_th", thread_struct.oled_struct.oled_th_entry, NULL, 1024, 20, 5);
    rt_thread_startup(thread_struct.oled_struct.oled_th);

    rt_sem_init(&sem_struct.motor_sem, "motor_sem", 0, RT_IPC_FLAG_FIFO);
    thread_struct.motor_th_entry = MOTOR_th_entry;
    thread_struct.motor_struct.motor_th = rt_thread_create("motor_th", thread_struct.motor_th_entry, NULL, 512, 15, 5);
    rt_thread_startup(thread_struct.motor_struct.motor_th);

    sem_struct.loaded_pause_sem = rt_sem_create("loaded_pause_sem", 0, RT_IPC_FLAG_PRIO);
    sem_struct.serialnum_pause_sem = rt_sem_create("serialnum_pause_sem", 0, RT_IPC_FLAG_PRIO);

    return RT_EOK;
}

uint8_t* int2ascii(uint32_t val)
{
    uint8_t *p,asc[5];
    asc[0]=((uint8_t)(val/10000%10))|0x30;
    asc[1]=((uint8_t)(val/1000%10))|0x30;
    asc[2]=((uint8_t)(val/100%10))|0x30;
    asc[3]=((uint8_t)(val/10%10))|0x30;
    asc[4]=((uint8_t)(val/1%10))|0x30;
    p=asc;
    return p;
}

uint8_t* int2char(uint32_t val)
{
    uint8_t *p,car[5];
    car[0]=(uint8_t)(val/10000%10);
    car[1]=(uint8_t)(val/1000%10);
    car[2]=(uint8_t)(val/100%10);
    car[3]=(uint8_t)(val/10%10);
    car[4]=(uint8_t)(val/1%10);
    p=car;
    return p;
}


