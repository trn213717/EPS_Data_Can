/*
 * Can.h
 *
 *  Created on: Sep 13, 2023
 *      Author: gk842
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_


void CAN_TX(unsigned int CAN_ID, int16_t a, double b);
void CAN_RX(void);
#endif /* INC_CAN_H_ */
