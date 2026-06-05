#ifndef __ALERT_H
#define __ALERT_H

#include "main.h"

// 声光任务类型
typedef enum {
    ALERT_NONE = 0,
    ALERT_TARGET_FOUND, // 找柱子：响2次，闪2次 (每次>1s)
    ALERT_PARKING       // 停车：长鸣 3s
} AlertType_t;

void Alert_Start(AlertType_t type);
uint8_t Alert_Process(void); // 在 while(1) 中循环调用，返回 1 表示动作执行完毕

#endif