
#ifndef CONFIG_H
#define CONFIG_H

#include "debug.h"
#include <stdint.h>

#define USART3_RECV_BUF_SIZE      256
#define USART3_SEND_BUF_SIZE      1024

#define UART5_RECV_BUF_SIZE       2048
#define UART5_SEND_BUF_SIZE       256

#define LDS_BUF_SIZE       128
#define CMD_RECV_BUF_SIZE   USART3_RECV_BUF_SIZE
#define CMD_SEND_BUF_SIZE   USART3_SEND_BUF_SIZE

#define CMD_LINE_SIZE   128
#define CMD_NAME_SIZE   16

#define T_SPI_SS_DELAY  14  // delay for weak pull-up, 14 us
#define SPI_SS_DELAY    500 // 

#define INTEGRAL_MAX    3500




#define STATE_OK            0
#define STATE_PENDING       1



#define OFFSET(s, m)    (size_t)&(((s *)0)->m)


#endif /* CONFIG_H */

/* end of config.h */
