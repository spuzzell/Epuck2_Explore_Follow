//Include statements
#include <stdio.h>              	  //Standard input and output                    (Default)
#include <stdlib.h>             	  //Memory allocation, random number generation  (Default)
#include <string.h>           	      //String manipulation functions                (Default)
#include <math.h>             	      //Mathematical functions and constants         (Default)
#include <stdbool.h>                  //Boolean data type                            (Added)

#include "ch.h"                       //                                             (Default)
#include "hal.h"                      //                                             (Default)
#include "memory_protection.h"        //                                             (Default)
#include <main.h>                     //                                             (Default)

#include "epuck1x/uart/e_uart_char.h" //For Bluetooth                                (Added)
#include "stdio.h"                    //For Bluetooth                                (Added)
#include "serial_comm.h"              //For Bluetooth                                (Added)
#include "motors.h"                   //For Motor                                    (Added)
#include "selector.h"                 //For Selector                                 (Added)
#include "sensors/proximity.h"        //For Proximity                                (Added)
#include "sensors/VL53L0X/VL53L0X.h"  //For Distance                                 (Added)

//Inter process communication bus
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

//Functions Declarations
void observe(void);                   //Reads calibrated proximity data to IR_value_raw / IR_values_act
void push_move(int,int);              //Sets wheel velocities based on an input state and an angle

//Symbolic Constants
#define TIME_STEP 100                  //The time step between while loop
#define LIMIT     500                 //velocity limit
#define NUM_OF_IR 8                   //Number of ir sensors
#define IR_THRESHOLD 1000             //Thresehold for detecting obsticle (higher is closer)
#define SPEED_DEG 283.588111888       //Angular velocity at full speed

int IR_values_raw[NUM_OF_IR];         //Stores IR proximity data
bool   IR_values_act[NUM_OF_IR];      //Stores whether a IR sensor is active based on a thresehold value

int main(void){
//Default
	halInit();chSysInit();mpu_init();                                     
//proximity
	messagebus_init(&bus, &bus_lock, &bus_condvar); 
    proximity_start(0);calibrate_ir();                                 
//Bluetooth
	serial_start(); 
  //Motor                               
	motors_init();                                


    
	int rand_angle;
    char str[100];
    int str_length;
    int min_t=50;
    int max_t=750;
    double pause =0;

    while (1) {
    	//read sensor
    	observe();

    	if (IR_values_act[1] && IR_values_act[6]) {
    		rand_angle = (rand() % (101)) + 130;                   //picks angle between 130 and 230
    		pause = (rand_angle / (SPEED_DEG*(LIMIT/1000)))*1000;
			left_motor_set_speed(LIMIT);
			right_motor_set_speed(-LIMIT);
			chThdSleepMilliseconds(pause);

    	} else if (IR_values_act[0] && IR_values_act[7]) {
			rand_angle = (rand() % 161) + 100;                     //picks angle between 100 and 260
			pause = (rand_angle / (SPEED_DEG*(LIMIT/1000)))*1000;
			left_motor_set_speed(LIMIT);
			right_motor_set_speed(-LIMIT);
			chThdSleepMilliseconds(pause);

		} else if (IR_values_act[7] || IR_values_act[6]) {
			left_motor_set_speed(LIMIT);
			right_motor_set_speed(LIMIT/2);
    		chThdSleepMilliseconds(rand() % (max_t + 1 - min_t) + min_t);

		} else if (IR_values_act[0] || IR_values_act[1]) {
			left_motor_set_speed(LIMIT/2);
			right_motor_set_speed(LIMIT);
    		chThdSleepMilliseconds(rand() % (max_t + 1 - min_t) + min_t);

		} else {
			left_motor_set_speed(LIMIT);
			right_motor_set_speed(LIMIT);                     //Forward
		}

    	chThdSleepMilliseconds(TIME_STEP);
        }


}


//Epuck Default Definitions 
 #define STACK_CHK_GUARD 0xe2dee396
 uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

 void __stack_chk_fail(void)
 {
     chSysHalt("Stack smashing detected");
 }


void observe(void){
    for(int i = 0; i < NUM_OF_IR; i++){
    	IR_values_raw[i] = get_calibrated_prox(i);
    	
    	if(IR_values_raw[i]<=IR_THRESHOLD){
    		IR_values_act[i]=false;
    	}else{IR_values_act[i]=true;
    };
  }
}
