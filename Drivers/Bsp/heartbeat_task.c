
#include <heartbeat_task.h>

#include <board.h>

struct offline_event_obj offline_event_table[] =
{
    /* event | enable | level | beep_times | offline time | offline_first_func| offline_func | online_first_func | online_func */
    {SYSTEM_PROTECT, DISABLE, OFFLINE_WARNING_LEVEL, 0, 0, 0, NULL, NULL, NULL},
    {OFFLINE_ENCODER_TASK1, DISABLE, OFFLINE_WARNING_LEVEL, 1, 1000, NULL, NULL, NULL, NULL},
    {OFFLINE_ENCODER_TASK2, DISABLE, OFFLINE_WARNING_LEVEL, 1, 1000, NULL, NULL, NULL, NULL},
    {OFFLINE_ENCODER_TASK3, DISABLE, OFFLINE_WARNING_LEVEL, 1, 1000, NULL, NULL, NULL, NULL},
    {OFFLINE_ENCODER_FORWARD, DISABLE, OFFLINE_WARNING_LEVEL, 1, 1000, NULL, NULL, NULL, NULL},
};

static heartbeat_led_func_t g_heartbeat_led_func = NULL;
static offline_led_func_t g_offline_led_func = NULL;

// 注册心跳灯控制函数
void register_heartbeat_led_func(heartbeat_led_func_t func) {
    g_heartbeat_led_func = func;
}
// 注册离线灯控制函数
void register_offline_led_func(offline_led_func_t func) {
    g_offline_led_func = func;
}
// 心跳灯翻转
void toggle_heartbeat_led(void) {
    if (g_heartbeat_led_func) g_heartbeat_led_func();
}
// 离线灯开关
void set_offline_led(int on) {
    if (g_offline_led_func) g_offline_led_func(on);
}

static void offline_event_callback(struct offline_manage_obj *obj);

struct offline_manage_obj offline_manage[OFFLINE_EVENT_MAX_NUM] = {NO_OFFLINE};

/**
 * @brief  get system offline event
 * @param
 * @retval void
 */
uint8_t get_system_status(void)
{
   return offline_manage[SYSTEM_PROTECT].online_state;
}

/**
 * @brief  register offline event
 * @param
 * @retval void
 */
void offline_event_init(struct offline_manage_obj obj)
{
   offline_event event = obj.event;

   offline_manage[event].event = event;
   offline_manage[event].enable = obj.enable;
   offline_manage[event].error_level = obj.error_level;
   offline_manage[event].online_first_func = obj.online_first_func;
   offline_manage[event].offline_first_func = obj.offline_first_func;
   offline_manage[event].online_func = obj.online_func;
   offline_manage[event].offline_func = obj.offline_func;

   offline_manage[event].beep_times = obj.beep_times;
   offline_manage[event].offline_time = obj.offline_time;
}

/**
 * @brief  called when offline happened.
 * @param
 * @retval void
 */
void offline_event_callback(struct offline_manage_obj *obj)
{
   // offline
   if (obj->online_state == STATE_OFFLINE)
   {
       if (obj->last_state == STATE_ONLINE)
       {
           if (obj->offline_first_func != NULL)
           {
               obj->offline_first_func();
           }
       }
       else
       {
           if (obj->offline_func != NULL)
           {
               obj->offline_func();
           }
       }
   }
   else // online
   {
       obj->online_state = STATE_ONLINE;
       if (obj->last_state == STATE_OFFLINE)
       {
           if (obj->online_first_func != NULL)
           {
               obj->online_first_func();
           }
       }
       else
       {
           if (obj->online_func != NULL)
           {
               obj->online_func();
           }
       }
   }
   obj->last_state = obj->online_state;
}

/**
 * @brief  update the information receive time.
 * @param
 * @retval void
 */
void offline_event_time_update(offline_event event)
{
   offline_manage[event].last_time = get_time_ms();
}

/**
 * @brief  enable event offline check
 * @param
 * @retval void
 */
void offline_event_enable(offline_event event)
{
   offline_manage[event].enable = 1;
}

void offline_event_disable(offline_event event)
{
   offline_manage[event].enable = 0;
}

void init_offline_event(){

    struct offline_manage_obj offline_obj;
    int offline_tab_size = sizeof(offline_event_table) / sizeof(struct offline_event_obj);
    if (offline_tab_size > 0)
    {
        for (int i = 0; i < offline_tab_size; i++)
        {
            offline_obj.event = offline_event_table[i].event;
            offline_obj.enable = offline_event_table[i].enable;
            offline_obj.error_level = offline_event_table[i].level;
  
            offline_obj.online_first_func = offline_event_table[i].online_first_func;
            offline_obj.offline_first_func = offline_event_table[i].offline_first_func;
            offline_obj.online_func = offline_event_table[i].online_func;
            offline_obj.offline_func = offline_event_table[i].offline_func;
  
            offline_obj.beep_times = offline_event_table[i].beep_times;
            offline_obj.offline_time = offline_event_table[i].offline_time;
            offline_event_init(offline_obj);
        }
    }
    
    for (int i = 1; i < OFFLINE_EVENT_MAX_NUM; i++)
    {
        offline_manage[i].online_state = STATE_OFFLINE;
    }
 }
 
 void update_offline_event(){
 
    // check
    uint32_t time_now = get_time_ms();
    offline_event display_event = NO_OFFLINE;
    uint8_t error_level = 0XFF;
     char event_str[128] = "offline event: ";
    for (int i = 1; i < OFFLINE_EVENT_MAX_NUM; i++)
    {
        if ((time_now - offline_manage[i].last_time > offline_manage[i].offline_time) && (offline_manage[i].enable))
        {
            offline_manage[i].online_state = STATE_OFFLINE;
            if (error_level > offline_manage[i].error_level)
            {
                error_level = offline_manage[i].error_level;
            }
 
            if (offline_manage[i].error_level <= OFFLINE_WARNING_LEVEL)
            {
                display_event = (offline_event)i;
                char numbuf[8];
                snprintf(numbuf, sizeof(numbuf), "%d,", i);
                strncat(event_str, numbuf, sizeof(event_str) - strlen(event_str) - 1);
            }
        }
        else
        {
            offline_manage[i].online_state = STATE_ONLINE;
        }
    }
 
    // deal error level
    if ((error_level == OFFLINE_ERROR_LEVEL) && (offline_manage[SYSTEM_PROTECT].enable))
    {
        offline_manage[SYSTEM_PROTECT].online_state = STATE_OFFLINE;
        offline_event_callback(&offline_manage[SYSTEM_PROTECT]);
    }
    else // deal normal level
    {
        offline_manage[SYSTEM_PROTECT].online_state = STATE_ONLINE;
        offline_event_callback(&offline_manage[SYSTEM_PROTECT]);
 
        for (int i = 1; i < OFFLINE_EVENT_MAX_NUM; i++)
        {
            offline_event_callback(&offline_manage[i]);
        }
    }
 
    if (display_event != NO_OFFLINE)
    {
        set_offline_led(1);
          if(event_str[0] != '\0') {
              printf("%s\r\n", event_str);
          }
    }
    else
    {
        set_offline_led(0);
    }
    toggle_heartbeat_led();
 }

void StartHeartbeatTask(void *argument)
{
//    init_offline_event();

   for(;;)
   {
    // update_offline_event();
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
    
#ifdef DEBUG_TASK_STACK
    UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    printf("HeartbeatTask stack high water mark: %lu bytes\r\n", stackHighWaterMark);
#endif
    osDelay(1000);
   }
}
