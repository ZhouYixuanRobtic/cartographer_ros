#ifndef __SERIAL_H__
#define __SERIAL_H__
#include <stdint.h>
#include <stdbool.h>
#ifdef __cplusplus  
extern "C" {  
#endif 
int open_serial_port(void);
int close_serial_port(void);
int read_once(int16_t Acc[3], int16_t Gyo[3], int16_t Mag[3], float Eular[3], float Quat[4]);
#ifdef __cplusplus  
}  
#endif 
#endif
