
/* interface for infrared command */


#ifndef __SCT_CAPTURE_H_
#define __SCT_CAPTURE_H_

#include "FreeRTOS.h"
#include "queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	 initiate SCT0 and create  irCommandQueue to hold received infrared command
 * @param	Nothing
  * @return	Nothing
	* @note	 must be called before using irCommandQueue queue
 */
void InfraRed_init(void);
	/* queue hold the  infrared received command*/
extern QueueHandle_t irCommandQueue; 
	
/**
 * @}
 */

 #ifdef __cplusplus
}
#endif

#endif /* __SCT_CAPTURE_H_ */