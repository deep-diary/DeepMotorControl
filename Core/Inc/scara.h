// scara.h
#ifndef SCARA_H
#define SCARA_H

#include <stdbool.h>
#include <stdint.h>
#include "fdcan.h"
// ?????????
#define L1 140.0f    // ????(?)
#define L2 200.0f    // ????(?)
#define D 160.0f     // ????????(?)
#define MOTOR_ID_L 		0X02
#define MOTOR_ID_R 		0X01
#define PI 						3.14159
#define MAX_STP_BASEON_10MS						50
#define ARM_LOC_X 0x8000       // 机械臂X坐标
#define ARM_LOC_Y 0x8001       // 机械臂Y坐标


// SCARA????????
typedef struct {
    float x;       // ??? x ??
    float y;       // ??? y ??
    float l1;      // ????
    float l2;      // ????
    float theta1;  // ????(??)
    float theta2;  // ????(??)
		uint8_t id;
		float theta_mt;  // ????(??)
} ScaraArm;

// ?? SCARA ??????,???????
typedef struct {
    ScaraArm left_arm;
    ScaraArm right_arm;
		float test_x;       // req x point in mm
    float test_y;       // req y point in mm
		float req_x;       // req x point in mm
    float req_y;       // req y point in mm
		float tar_x;       // target x point in mm
    float tar_y;       // target y point in mm
		bool 	isValid;		 // check whether the point is valid or not
} ParallelScara;

extern ParallelScara scaraArm;
extern bool validPos;

// ????? SCARA ???
void init_parallel_scara(ParallelScara *scara);

// ???????
bool calculate_scara_angles(ParallelScara *scara, float xt, float yt);

void arm_init(ParallelScara *scara); // init arm
void arm_zero(ParallelScara *scara); // back to zero
void arm_reset(ParallelScara *scara); // move to the init pos and disable the system
void arm_set_pos(ParallelScara *scara, float xt, float yt); // set position
void arm_mov_line(ParallelScara *scara, float x0, float y0, float x1, float y1, uint8_t stps); // move line
// void uart_cmd(ParallelScara *scara);
#endif // SCARA_H



