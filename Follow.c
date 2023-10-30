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

#include "motors.h"                   //For Motor                                    (Added)
#include "selector.h"                 //For Selector                                 (Added)
#include "sensors/proximity.h"        //For Proximity                                (Added)


//Inter process communication bus
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

//Functions Declarations
void observe(void);                   //Reads calibrated proximity data to IR_value_raw / IR_values_act

//Symbolic Constants
#define TIME_STEP 100                  //The time step between while loop
#define LIMIT     500                  //velocity limit
#define UPPER     2500                 //TOO Close
#define LOWER     2000                 //TOO Far


int    IR_raw[8];        //Stores IR proximity data
int    IR_max;                   //Max IR
int    IR_sec;                   //Second IR
int    velocity_turn[2] ={0,0};


 
int main(void){
//Default
	halInit();chSysInit();mpu_init();                                     
//proximity
	messagebus_init(&bus, &bus_lock, &bus_condvar); 
    proximity_start(0);calibrate_ir();                                 
//Motor                               
	motors_init();  


                             

    while (1) {
    	//read sensor
    	observe();

    	//Selects velocity profile
    	if(IR_raw[IR_max]>LOWER){
    		velocity_turn[0]=LIMIT;
    		velocity_turn[1]=-LIMIT;
    	}else if(IR_raw[IR_max]<LOWER){
			velocity_turn[0]=LIMIT;
			velocity_turn[1]=LIMIT/2;
		}else{
			velocity_turn[0]=0;
			velocity_turn[1]=0;
		}

		//Rotates to face
		if (IR_max<4&&IR_max!=0){
			left_motor_set_speed(velocity_turn[0]);
			right_motor_set_speed(velocity_turn[1]);
		}else if(IR_max>3&&IR_max!=7){
			left_motor_set_speed(velocity_turn[1]);
			right_motor_set_speed(velocity_turn[0]);
		}

		if ((IR_max==0&&IR_sec==7)||(IR_max==7&&IR_sec==0)){
			if (IR_raw[IR_max]>UPPER){
				left_motor_set_speed(-LIMIT);
				right_motor_set_speed(-LIMIT);
			}else if(IR_raw[IR_max]<LOWER){
				left_motor_set_speed(LIMIT);
				right_motor_set_speed(LIMIT);
			}else{
				left_motor_set_speed(0);
				right_motor_set_speed(0);
			}
		}else if(IR_max==0&&IR_sec!=7){
			left_motor_set_speed(velocity_turn[0]/2);
			right_motor_set_speed(velocity_turn[1]/2);
		}else if(IR_max==7&&IR_sec!=0){
			left_motor_set_speed(velocity_turn[1]/2);
			right_motor_set_speed(velocity_turn[0]/2);
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


void observe(void) {
    IR_max = -1; // Initialize IR_max to an invalid index
    IR_sec = -1; // Initialize IR_sec to an invalid index

    for (int i = 0; i < 8; i++) {
        IR_raw[i] = get_calibrated_prox(i); //

        if (IR_max == -1 || IR_raw[i] > IR_raw[IR_max]) {
            IR_sec = IR_max;
            IR_max = i;
        } else if (IR_sec == -1 || IR_raw[i] > IR_raw[IR_sec]) {
            IR_sec = i;
        }
    }
}
