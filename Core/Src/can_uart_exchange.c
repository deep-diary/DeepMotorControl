#include "can_uart_exchange.h"


// can to uart 
void can_to_uart(void)
{
	HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&can2uart_frame, sizeof(can2uart_frame));
}

// uart to can
void uart_to_can(void)
{
	memcpy((uint8_t *)&TxHeader.Identifier,(uint8_t *)&uart2can_frame.canID,sizeof(uart2can_frame.canID));
  memcpy((uint8_t *)&TxData,(uint8_t *)&uart2can_frame.data,uart2can_frame.data_length);
	can_txd();
}
// resolve uart cmd
void uart_cmd(void)
{
    uint16_t index;
    float value;

    // Check if mode is 0x12, otherwise return
    if (txCanIdInfo.mode != 0x12)
        return;

    // Combine TxData[0] and TxData[1] to get the index
    index = (TxData[1] << 8) | TxData[0];

    // Combine TxData[4] to TxData[7] to get the value as a float
    uint32_t value_int = (TxData[7] << 24) | (TxData[6] << 16) | (TxData[5] << 8) | TxData[4];
    value = *(float*)&value_int;

    // Process command based on the index
    switch(index) {
		// arm parameters
			case ARM_LOC_X:
            // Store the x required point
                        //scara->test_x = value;
            scaraArm.req_x = value;
            break;
        case ARM_LOC_Y:
            // Store the y required point
                        //scara->test_y = value;
            scaraArm.req_y = value;
            break;
		// camera parameters
        case CAMERA_ERROR_X:
            platform_status.x_error = value;
            platform_status.error_updated = true;
            platform_status.angle_updated = false;
            platform_status.cmd_received = true;  // 设置指令接收标志
            break;
        case CAMERA_ERROR_Y:
            platform_status.y_error = value;
            platform_status.error_updated = true;
            platform_status.angle_updated = false;
            platform_status.cmd_received = true;  // 设置指令接收标志
            break;
        case CAMERA_H_ANGLE:
            platform_status.h_angle = value;
            platform_status.error_updated = false;
            platform_status.angle_updated = true;
            platform_status.cmd_received = true;  // 设置指令接收标志
            break;
        case CAMERA_V_ANGLE:
            platform_status.v_angle = value;
            platform_status.error_updated = false;
            platform_status.angle_updated = true;
            platform_status.cmd_received = true;  // 设置指令接收标志
            break;
        case CAMERA_TARGET_DETECTED:
            set_target_locked(value > 0.5);
            platform_status.cmd_received = true;  // 设置指令接收标志
            break;
        default:
            // No matching command; do nothing
            break;
    }
}



