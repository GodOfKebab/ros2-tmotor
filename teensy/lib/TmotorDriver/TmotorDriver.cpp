#include "TmotorDriver.h"

uint32_t tmotor_id = 0;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CanBus;

rcl_publisher_t tmotor_state_publisher;
rcl_subscription_t set_position_subscriber;


void TmotorDriver::init(rclc_executor_t* executor, rcl_node_t* node) {

    // create publisher
    RCCHECK(rclc_publisher_init_default(
            &tmotor_state_publisher,
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(custom_messages, msg, TmotorState),
            "/micro_ros_teensy/motor_state"));

    // create subscriber
    RCCHECK(rclc_subscription_init_default(
            &set_position_subscriber,
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
            "/micro_ros_teensy/set_position"));

    std_msgs__msg__Float64 msg;
    RCCHECK(rclc_executor_add_subscription(executor,
                                           &set_position_subscriber,
                                           &msg,
                                           &TmotorDriver::set_position_callback,
                                           ON_NEW_DATA));

    CanBus.begin();
    CanBus.setBaudRate(1000000);
    CanBus.setMaxMB(16);
    CanBus.enableFIFO();
    CanBus.enableFIFOInterrupt();
    CanBus.onReceive(TmotorDriver::tmotor_state_sniffer);
    CanBus.mailboxStatus();
}

void TmotorDriver::check_events() {
    CanBus.events();
}


void TmotorDriver::tmotor_state_sniffer(const CAN_message_t &can_msg) {
    custom_messages__msg__TmotorState msg;
    // Assume failure at teensy-side, if not override
    msg.error_code = 7;
    if (can_msg.len == 8) {
        tmotor_id = can_msg.id;
        msg.angular_position = (int16_t)(can_msg.buf[1] | ((int16_t)can_msg.buf[0] << 8)) * 0.00174533;  // 1 / 10 * 6 * pi / 180
        msg.angular_velocity = (int16_t)(can_msg.buf[3] | ((int16_t)can_msg.buf[2] << 8)) * 0.01047198;  // 1 / 10 * 6 * pi / 180
        msg.current          = (int16_t)(can_msg.buf[5] | ((int16_t)can_msg.buf[4] << 8)) * 0.01;
        msg.temp             = (int8_t)can_msg.buf[6];
        msg.error_code       = (int8_t)can_msg.buf[7];
    }
    RCSOFTCHECK(rcl_publish(&tmotor_state_publisher, &msg, NULL));

}

void TmotorDriver::set_duty_cycle_callback(const void *msgin) {

}

void TmotorDriver::set_current_loop_callback(const void *msgin) {

}

void TmotorDriver::set_current_brake_callback(const void *msgin) {

}

void TmotorDriver::set_velocity_callback(const void *msgin) {

}

void TmotorDriver::set_position_callback(const void *msgin) {
    const auto * msg = (const std_msgs__msg__Float64 *)msgin;
    uint8_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(msg->data * 10000.0), &send_index);
    comm_can_transit_eid((tmotor_id  & 0xFF) | ((uint32_t) CAN_PACKET_SET_POS << 8), buffer, send_index);

}

void TmotorDriver::set_position_velocity_loop_callback(const void *msgin) {

}

// Helper functions from the datasheet

void TmotorDriver::comm_can_transit_eid(uint32_t id, const uint8_t* data, uint8_t len) {
    // Force a max length of 8 bytes
    len = len > 8 ? 8 : len;
    CAN_message_t m;
    m.id = id;
    m.flags.extended = true;
    m.len = len;
    for (int i = 0; i < len; ++i) { m.buf[i] = data[i]; }
    CanBus.write(m);
}


void TmotorDriver::buffer_append_int32(uint8_t* buffer, int32_t number, uint8_t* index) {
    buffer[(*index)++] = number >> 24;
    buffer[(*index)++] = number >> 16;
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void TmotorDriver::buffer_append_int16(uint8_t* buffer, int16_t number, uint8_t* index) {
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

