#include "TmotorDriver.h"

uint32_t tmotor_id = 0;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CanBus;

rcl_publisher_t tmotor_state_publisher;

custom_messages__msg__TmotorServoDutyCycleCommand servo_duty_cycle_msg;
rcl_subscription_t set_duty_cycle_subscriber;
custom_messages__msg__TmotorServoCurrentLoopCommand servo_current_loop_msg;
rcl_subscription_t set_current_loop_subscriber;
custom_messages__msg__TmotorServoCurrentBrakeCommand servo_current_brake_msg;
rcl_subscription_t set_current_brake_subscriber;
custom_messages__msg__TmotorServoVelocityCommand servo_velocity_msg;
rcl_subscription_t set_velocity_subscriber;
custom_messages__msg__TmotorServoPositionCommand servo_position_msg;
rcl_subscription_t set_position_subscriber;
custom_messages__msg__TmotorServoPositionVelocityLoopCommand servo_position_velocity_loop_msg;
rcl_subscription_t set_position_velocity_loop_subscriber;


void TmotorDriver::init(rclc_executor_t* executor, rcl_node_t* node) {

    // create state publisher
    RCCHECK(rclc_publisher_init_default(
            &tmotor_state_publisher,
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(custom_messages, msg, TmotorState),
            "/micro_ros_teensy/motor_state"));

    // create duty cycle subscriber
    RCCHECK(rclc_subscription_init_default(
            &set_duty_cycle_subscriber,
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(custom_messages, msg, TmotorServoDutyCycleCommand),
            "/micro_ros_teensy/set_duty_cycle"));

    RCCHECK(rclc_executor_add_subscription(executor,
                                           &set_duty_cycle_subscriber,
                                           &servo_duty_cycle_msg,
                                           &TmotorDriver::set_duty_cycle_callback,
                                           ON_NEW_DATA));

    // create current loop subscriber
    RCCHECK(rclc_subscription_init_default(
            &set_current_loop_subscriber,
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(custom_messages, msg, TmotorServoCurrentLoopCommand),
            "/micro_ros_teensy/set_current_loop"));

    RCCHECK(rclc_executor_add_subscription(executor,
                                           &set_current_loop_subscriber,
                                           &servo_current_loop_msg,
                                           &TmotorDriver::set_current_loop_callback,
                                           ON_NEW_DATA));

    // create current brake subscriber
    RCCHECK(rclc_subscription_init_default(
            &set_current_brake_subscriber,
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(custom_messages, msg, TmotorServoCurrentBrakeCommand),
            "/micro_ros_teensy/set_current_brake"));

    RCCHECK(rclc_executor_add_subscription(executor,
                                           &set_current_brake_subscriber,
                                           &servo_current_brake_msg,
                                           &TmotorDriver::set_current_brake_callback,
                                           ON_NEW_DATA));

    // create velocity subscriber
    RCCHECK(rclc_subscription_init_default(
            &set_velocity_subscriber,
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(custom_messages, msg, TmotorServoVelocityCommand),
            "/micro_ros_teensy/set_velocity"));

    RCCHECK(rclc_executor_add_subscription(executor,
                                           &set_velocity_subscriber,
                                           &servo_velocity_msg,
                                           &TmotorDriver::set_velocity_callback,
                                           ON_NEW_DATA));

    // create position subscriber
    RCCHECK(rclc_subscription_init_default(
            &set_position_subscriber,
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(custom_messages, msg, TmotorServoPositionCommand),
            "/micro_ros_teensy/set_position"));

    RCCHECK(rclc_executor_add_subscription(executor,
                                           &set_position_subscriber,
                                           &servo_position_msg,
                                           &TmotorDriver::set_position_callback,
                                           ON_NEW_DATA));

    // create position velocity loop subscriber
    RCCHECK(rclc_subscription_init_default(
            &set_position_velocity_loop_subscriber,
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(custom_messages, msg, TmotorServoPositionVelocityLoopCommand),
            "/micro_ros_teensy/set_position_velocity_loop"));

    RCCHECK(rclc_executor_add_subscription(executor,
                                           &set_position_velocity_loop_subscriber,
                                           &servo_position_velocity_loop_msg,
                                           &TmotorDriver::set_position_velocity_loop_callback,
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
    const auto * msg = (const custom_messages__msg__TmotorServoDutyCycleCommand *)msgin;
    uint8_t send_index = 0;
    uint8_t buffer[4];
    auto duty_cycle_clipped = max(-1., min(1., msg->duty_cycle)) * 100000.;
    buffer_append_int32(buffer, (int32_t)(duty_cycle_clipped), &send_index);
    comm_can_transit_eid((tmotor_id  & 0xFF) | ((uint32_t) CAN_PACKET_SET_DUTY << 8), buffer, send_index);
}

// !!! NOT TESTED !!!
void TmotorDriver::set_current_loop_callback(const void *msgin) {
    const auto * msg = (const custom_messages__msg__TmotorServoCurrentLoopCommand *)msgin;
    uint8_t send_index = 0;
    uint8_t buffer[4];
    auto current_clipped = max(-60., min(60., msg->current)) * 1000.;
    buffer_append_int32(buffer, (int32_t)(current_clipped), &send_index);
    comm_can_transit_eid((tmotor_id  & 0xFF) | ((uint32_t) CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
}

void TmotorDriver::set_current_brake_callback(const void *msgin) {
    const auto * msg = (const custom_messages__msg__TmotorServoCurrentBrakeCommand *)msgin;
    uint8_t send_index = 0;
    uint8_t buffer[4];
    auto current_clipped = max(0., min(60., msg->current)) * 10000.;
    buffer_append_int32(buffer, (int32_t)(current_clipped), &send_index);
    comm_can_transit_eid((tmotor_id  & 0xFF) | ((uint32_t) CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, send_index);
}

void TmotorDriver::set_velocity_callback(const void *msgin) {
    const auto * msg = (const custom_messages__msg__TmotorServoVelocityCommand *)msgin;
    uint8_t send_index = 0;
    uint8_t buffer[4];
    auto velocity_clipped = max(-100000., min(100000., msg->angular_velocity * 954.92966)); // 10*60/(2*pi)
    buffer_append_int32(buffer, (int32_t)(velocity_clipped), &send_index);
    comm_can_transit_eid((tmotor_id  & 0xFF) | ((uint32_t) CAN_PACKET_SET_RPM << 8), buffer, send_index);
}

void TmotorDriver::set_position_callback(const void *msgin) {
    const auto * msg = (const custom_messages__msg__TmotorServoPositionCommand *)msgin;
    uint8_t send_index = 0;
    uint8_t buffer[4];
    auto position_clipped = max(-360000000., min(360000000., msg->angular_position * 572957.80)); // 10000. * 180/pi
    buffer_append_int32(buffer, (int32_t)(position_clipped), &send_index);
    comm_can_transit_eid((tmotor_id  & 0xFF) | ((uint32_t) CAN_PACKET_SET_POS << 8), buffer, send_index);
}

void TmotorDriver::set_position_velocity_loop_callback(const void *msgin) {
    const auto * msg = (const custom_messages__msg__TmotorServoPositionVelocityLoopCommand *)msgin;
    uint8_t send_index = 0;
    uint8_t buffer[8];
    auto position_clipped = max(-360000000., min(360000000., msg->angular_position * 572957.80)); // 10000. * 180/pi
    buffer_append_int32(buffer, (int32_t)(position_clipped), &send_index);
    auto velocity_clipped = max(-32768., min(32767., msg->angular_velocity * 95.492966)); // 60/(2*pi)
    buffer_append_int16(buffer, (int16_t)(velocity_clipped), &send_index);
    auto acceleration_clipped = max(0., min(200., msg->angular_acceleration * 95.492966)); // 60/(2*pi)
    buffer_append_int16(buffer, (int16_t)(acceleration_clipped), &send_index);
    comm_can_transit_eid((tmotor_id  & 0xFF) | ((uint32_t) CAN_PACKET_SET_POS_SPD << 8), buffer, send_index);
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

