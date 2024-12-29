// scara.c

#include "scara.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

ParallelScara scaraArm;

bool validPos;

// ????? SCARA ???????????
void init_parallel_scara(ParallelScara *scara) {
    // ???????
    scara->left_arm.x = -D / 2.0f;
    scara->left_arm.y = 0.0f;
    scara->left_arm.l1 = L1;
    scara->left_arm.l2 = L2;		
		scara->left_arm.id = MOTOR_ID_L;
		
	
    // ???????
    scara->right_arm.x = D / 2.0f;
    scara->right_arm.y = 0.0f;
    scara->right_arm.l1 = L1;
    scara->right_arm.l2 = L2;
		scara->right_arm.id = MOTOR_ID_R;
	
		// set the default require position
		scara->req_x = 0;
		scara->req_y = 200;
		scara->tar_x = 0;   // zero pos
		scara->tar_y = 323.3;  // zero pos
}

// ???? SCARA ??????,?????????
bool calculate_scara_angles(ParallelScara *scara, float xt, float yt) {
    // ????
    float dx_left = xt - scara->left_arm.x;
    float dy_left = yt - scara->left_arm.y;
    float D_left = sqrt(dx_left * dx_left + dy_left * dy_left);

    // ????????????
    if (D_left > (L1 + L2) || D_left < fabs(L2 - L1)) {
        //printf("???????????\n");
        return false;
    }

    float cos_theta2_left = (D_left * D_left - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
    float theta2_left_joint = acosf(cos_theta2_left);

    float alpha_left = atan2f(dy_left, dx_left);
    float beta_left = atan2f(L2 * sinf(theta2_left_joint), L1 + L2 * cosf(theta2_left_joint));

    // ????? y ????????????
    if (yt >= 0) {
        scara->left_arm.theta1 = alpha_left + beta_left;  // ?????? y ???
				scara->left_arm.theta2 = scara->left_arm.theta1 - theta2_left_joint;
    } else {
        scara->left_arm.theta1 = alpha_left - beta_left;  // ?????? y ???
				scara->left_arm.theta2 = scara->left_arm.theta1 + theta2_left_joint;
    }
		scara->left_arm.theta1 = scara->left_arm.theta1<0?scara->left_arm.theta1+2*PI:scara->left_arm.theta1;
		scara->left_arm.theta_mt = PI / 2 - scara->left_arm.theta1;
    

    // ????
    float dx_right = xt - scara->right_arm.x;
    float dy_right = yt - scara->right_arm.y;
    float D_right = sqrt(dx_right * dx_right + dy_right * dy_right);

    // ????????????
    if (D_right > (L1 + L2) || D_right < fabs(L2 - L1)) {
        //printf("???????????\n");
        return false;
    }

    float cos_theta2_right = (D_right * D_right - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
    float theta2_right_joint = acosf(cos_theta2_right);

    float alpha_right = atan2f(dy_right, dx_right);
    float beta_right = atan2f(L2 * sinf(theta2_right_joint), L1 + L2 * cosf(theta2_right_joint));

    // ????? y ????????????
    if (yt >= 0) {
        scara->right_arm.theta1 = alpha_right - beta_right;  // ?????? y ???
			  scara->right_arm.theta2 = scara->right_arm.theta1 + theta2_right_joint;
    } else {
        scara->right_arm.theta1 = alpha_right + beta_right;  // ?????? y ???
				scara->right_arm.theta2 = scara->right_arm.theta1 - theta2_right_joint;
    }
		scara->right_arm.theta1 = scara->right_arm.theta1<0?scara->right_arm.theta1+2*PI:scara->right_arm.theta1;
    scara->right_arm.theta_mt = PI / 2 - scara->right_arm.theta1;

    return true;
}

void arm_init(ParallelScara *scara)
{
	init_parallel_scara(scara);
	motor_init(scara->left_arm.id);
	motor_init(scara->right_arm.id);
}

void arm_zero(ParallelScara *scara)
{
	arm_set_pos(scara, 0, 323.3); // nearly zero point
	HAL_Delay(2000);
}

void arm_reset(ParallelScara *scara)
{
	arm_zero(scara);
	motor_reset(scara->left_arm.id, MASTER_ID);
	motor_reset(scara->right_arm.id, MASTER_ID);
}

//typedef struct {
//    ScaraArm left_arm;
//    ScaraArm right_arm;
//		float tar_x;       // target x point in mm
//    float tar_y;       // target y point in mm
//		bool 	isValid;		 // check whether the point is valid or not
//} ParallelScara;

void arm_filter_10ms(ParallelScara *scara, float xt, float yt, float max_stp)
{
	// currunt position is scara->tar_x and scara->tar_y
	// require position is xt, yt
	// calculate the distance dist_mov between current and require position
	// if the dist_mov > max_stp than calculate the max step position o x_max and y_max base on x_max = xt + X-axis component of max_stp, the same as yt
	// if the dist_mov < -max_stp than calculate the max step position o x_max and y_max base on x_max = xt - X-axis component of max_stp, the same as yt
	// otherwise just set the current position as require position
    float dx = xt - scara->tar_x;
    float dy = yt - scara->tar_y;
    float dist_mov = sqrtf(dx * dx + dy * dy);  // ????:sqrt((x2 - x1)^2 + (y2 - y1)^2)

    // ?????? max_stp
    if (dist_mov > max_stp) {
        // ???????????
        float step_ratio = max_stp / dist_mov;
        float x_max = scara->tar_x + dx * step_ratio;
        float y_max = scara->tar_y + dy * step_ratio;
        
        // ??????? x_max ? y_max
        scara->tar_x = x_max;
        scara->tar_y = y_max;
    } else if (dist_mov < -max_stp) {
        // ???????????
        float step_ratio = max_stp / dist_mov;
        float x_max = scara->tar_x - dx * step_ratio;
        float y_max = scara->tar_y - dy * step_ratio;
        
        // ??????? x_max ? y_max
        scara->tar_x = x_max;
        scara->tar_y = y_max;
    } else {
        // ????????,?????????
        scara->tar_x = xt;
        scara->tar_y = yt;
    }

}

void arm_set_pos(ParallelScara *scara, float xt, float yt)
{
	arm_filter_10ms(scara, xt, yt, MAX_STP_BASEON_10MS);
	scara->isValid = calculate_scara_angles(scara, scara->tar_x, scara->tar_y);
	if(!scara->isValid)
		return;
	// step5: Set position
	index = 0X7016 ;  // SPEED 0x7017
	ref = scara->left_arm.theta_mt;       // value
	motor_write(scara->left_arm.id, MASTER_ID);
	HAL_Delay(1);	
	ref = scara->right_arm.theta_mt;       // value
	motor_write(scara->right_arm.id, MASTER_ID);
	HAL_Delay(1);
}

void arm_mov_line(ParallelScara *scara, float x0, float y0, float x1, float y1, uint8_t stps)
{
	float delta_x,delta_y;
	float xt,yt;
	uint8_t	i;
	// calc each step movement
	delta_x = (x1 - x0) / stps;
	delta_y = (y1 - y0) / stps;
	
	for(i=0;i<=stps;i++)
	{
			// calc the target pos
			xt = x0 + delta_x*i;
			yt = y0 + delta_y*i;
			arm_set_pos(scara, xt, yt);
			HAL_Delay(200);
	}
}

// Function to resolve UART command
	// index is combine from TxData 0~1
	// value is combine from TxData 4~7
	// e.g. globle value: TxData = [01 80 00 00 00 00 80 3f] in hex
	// than the expected index is 0x8001, expected value is 1, which seems from 0x 3f800000
// void uart_cmd(ParallelScara *scara) {
//     uint16_t index;
//     float value;

//     // Check if mode is 0x12, otherwise return
//     if (txCanIdInfo.mode != 0x12)
//         return;

//     // Combine TxData[0] and TxData[1] to get the index
//     index = (TxData[1] << 8) | TxData[0];

//     // Combine TxData[4] to TxData[7] to get the value as a float
//     uint32_t value_int = (TxData[7] << 24) | (TxData[6] << 16) | (TxData[5] << 8) | TxData[4];
//     value = *(float*)&value_int;  // Reinterpret the 32-bit integer as a float

//     // Process command based on the index
//     switch(index) {
//         case 0x8000:
//             // Store the x required point
// 						//scara->test_x = value;
//             scara->req_x = value;
//             break;
//         case 0x8001:
//             // Store the y required point
// 						//scara->test_y = value;
//             scara->req_y = value;
//             break;
//         default:
//             // No matching command; do nothing
//             break;
//     }
// }