/*
 * seg.h
 *
 *  Created on: Apr 8, 2018
 *      Author: eric
 */

#ifndef SEG_H_
#define SEG_H_

#include "main.h"

//DEFINES
#define SEGDIG_0 SEG_g_Pin | SEG_h_Pin
#define SEGDIG_1 SEG_a_Pin | SEG_d_Pin | SEG_e_Pin | SEG_f_Pin | SEG_g_Pin | SEG_h_Pin
#define SEGDIG_2 SEG_c_Pin | SEG_f_Pin | SEG_h_Pin
#define SEGDIG_3 SEG_e_Pin | SEG_f_Pin | SEG_h_Pin
#define SEGDIG_4 SEG_a_Pin | SEG_d_Pin | SEG_e_Pin | SEG_h_Pin
#define SEGDIG_5 SEG_b_Pin | SEG_e_Pin | SEG_h_Pin
#define SEGDIG_6 SEG_b_Pin | SEG_h_Pin
#define SEGDIG_7 SEG_d_Pin | SEG_e_Pin | SEG_f_Pin | SEG_g_Pin | SEG_h_Pin
#define SEGDIG_8 SEG_h_Pin
#define SEGDIG_9 SEG_e_Pin | SEG_h_Pin
#define ALLSEG SEG_a_Pin | SEG_b_Pin | SEG_c_Pin | SEG_d_Pin | SEG_e_Pin | SEG_f_Pin | SEG_g_Pin | SEG_h_Pin


#define ANODE_A ANODE_A_Pin
#define ANODE_B ANODE_B_Pin /// THIS IS ON PORT B
#define ANODE_C
#define ANODE_D

//FUNCTION PROTOTYPES
void write_digit(uint8_t num);


#endif /* SEG_H_ */
