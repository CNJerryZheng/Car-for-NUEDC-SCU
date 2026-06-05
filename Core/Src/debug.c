#include "debug.h"

// 将数据定义在 .c 文件中，彻底杜绝 multiple definition 报错
const uint8_t JUSTFLOAT_TAIL[4] = {0x00, 0x00, 0x80, 0x7F};