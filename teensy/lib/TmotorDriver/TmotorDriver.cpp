#include "TmotorDriver.h"


FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CanBus;

rcl_publisher_t tmotor_servo_state_publisher;
rcl_publisher_t tmotor_motor_state_publisher;

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
custom_messages__msg__TmotorMotorControlCommand motor_control_msg;
rcl_subscription_t set_motor_control_subscriber;
rcl_service_t set_motor_mode_service;
custom_messages__srv__TmotorMotorSetMode_Response motor_mode_res;
custom_messages__srv__TmotorMotorSetMode_Request motor_mode_req;


uint8_t enter_mode[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
uint8_t exit_mode[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
uint8_t zero_mode[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};

void TmotorDriver::init(rclc_executor_t* executor, rcl_node_t* node) {

    // create state publisher
    RCCHECK(rclc_publisher_init_default(
            &tmotor_servo_state_publisher,
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(custom_messages, msg, TmotorServoState),
            "/micro_ros_teensy/servo_state"));

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

    // create state publisher
    RCCHECK(rclc_publisher_init_default(
            &tmotor_motor_state_publisher,
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(custom_messages, msg, TmotorMotorState),
            "/micro_ros_teensy/motor_state"));

    // create motor control subscriber
    RCCHECK(rclc_subscription_init_default(
            &set_motor_control_subscriber,
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(custom_messages, msg, TmotorMotorControlCommand),
            "/micro_ros_teensy/set_motor_control"));

    RCCHECK(rclc_executor_add_subscription(executor,
                                           &set_motor_control_subscriber,
                                           &motor_control_msg,
                                           &TmotorDriver::set_motor_control_callback,
                                           ON_NEW_DATA));

    // create motor set mode service
    RCCHECK(rclc_service_init_default(
            &set_motor_mode_service,
            node,
            ROSIDL_GET_SRV_TYPE_SUPPORT(custom_messages, srv, TmotorMotorSetMode),
            "/micro_ros_teensy/set_motor_mode"));

    RCCHECK(rclc_executor_add_service(executor,
                                      &set_motor_mode_service,
                                      &motor_mode_req,
                                      &motor_mode_res,
                                      TmotorDriver::set_motor_mode_callback));

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
    if (can_msg.flags.extended) {
        custom_messages__msg__TmotorServoState servo_msg;
        servo_msg.error_code = 7;
        if (can_msg.len == 8) {
            servo_msg.id               = (uint16_t) can_msg.id & 0xFF;
            servo_msg.angular_position =
                    (int16_t) (can_msg.buf[1] | ((int16_t) can_msg.buf[0] << 8)) * 0.00174533;  // 1 / 10 * 6 * pi / 180
            servo_msg.angular_velocity =
                    (int16_t) (can_msg.buf[3] | ((int16_t) can_msg.buf[2] << 8)) * 0.01047198;  // 1 / 10 * 6 * pi / 180
            servo_msg.current          = (int16_t) (can_msg.buf[5] | ((int16_t) can_msg.buf[4] << 8)) * 0.01;
            servo_msg.temp             = (int8_t) can_msg.buf[6];
            servo_msg.error_code       = (uint8_t) can_msg.buf[7];
        }
        RCSOFTCHECK(rcl_publish(&tmotor_servo_state_publisher, &servo_msg, NULL));
    } else {
        custom_messages__msg__TmotorMotorState motor_msg;
        motor_msg.error_code = 7;
        if (can_msg.len == 8) {
            motor_msg.id               = (uint8_t) can_msg.buf[0];
            motor_msg.angular_position =
                    uint_to_float((int16_t)(((int16_t)can_msg.buf[1] << 8) | can_msg.buf[2]), T_MOTOR_P_MIN, T_MOTOR_P_MAX, 16);
            if (motor_msg.angular_position < T_MOTOR_P_MIN) motor_msg.angular_position += T_MOTOR_P_MAX - T_MOTOR_P_MIN;
            motor_msg.angular_velocity =
                    uint_to_float((int16_t)(((int16_t)can_msg.buf[3] << 4) | ((int16_t)can_msg.buf[4] >> 4)), T_MOTOR_V_MIN, T_MOTOR_V_MAX, 12);
            motor_msg.torque           =
                    uint_to_float((int16_t)((((int16_t)can_msg.buf[4]&0xF) << 8) | can_msg.buf[5]), T_MOTOR_T_MIN, T_MOTOR_T_MAX, 12);
            motor_msg.temp             = (int8_t) can_msg.buf[6] - 32;
            motor_msg.error_code       = (uint8_t) can_msg.buf[7];
        }
        RCSOFTCHECK(rcl_publish(&tmotor_motor_state_publisher, &motor_msg, NULL));
    }
}

void TmotorDriver::set_duty_cycle_callback(const void *msgin) {
    const auto * msg = (const custom_messages__msg__TmotorServoDutyCycleCommand *)msgin;
    uint8_t send_index = 0;
    uint8_t buffer[4];
    auto duty_cycle_clipped = max(-1., min(1., msg->duty_cycle)) * 100000.;
    buffer_append_int32(buffer, (int32_t)(duty_cycle_clipped), &send_index);
    comm_can_transit_eid_servo(msg->id | ((uint32_t) CAN_PACKET_SET_DUTY << 8), buffer, send_index);
}

// !!! Scaling might be off !!!
void TmotorDriver::set_current_loop_callback(const void *msgin) {
    const auto * msg = (const custom_messages__msg__TmotorServoCurrentLoopCommand *)msgin;
    uint8_t send_index = 0;
    uint8_t buffer[4];
    auto current_clipped = max(-60., min(60., msg->current)) * 100000.;
    buffer_append_int32(buffer, (int32_t)(current_clipped), &send_index);
    comm_can_transit_eid_servo(msg->id | ((uint32_t) CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
}

void TmotorDriver::set_current_brake_callback(const void *msgin) {
    const auto * msg = (const custom_messages__msg__TmotorServoCurrentBrakeCommand *)msgin;
    uint8_t send_index = 0;
    uint8_t buffer[4];
    auto current_clipped = max(0., min(60., msg->current)) * 100000.;
    buffer_append_int32(buffer, (int32_t)(current_clipped), &send_index);
    comm_can_transit_eid_servo(msg->id | ((uint32_t) CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, send_index);
}

void TmotorDriver::set_velocity_callback(const void *msgin) {
    const auto * msg = (const custom_messages__msg__TmotorServoVelocityCommand *)msgin;
    uint8_t send_index = 0;
    uint8_t buffer[4];
    auto velocity_clipped = max(-100000., min(100000., msg->angular_velocity * 954.92966)); // 10*60/(2*pi)
    buffer_append_int32(buffer, (int32_t)(velocity_clipped), &send_index);
    comm_can_transit_eid_servo(msg->id | ((uint32_t) CAN_PACKET_SET_RPM << 8), buffer, send_index);
}

void TmotorDriver::set_position_callback(const void *msgin) {
    const auto * msg = (const custom_messages__msg__TmotorServoPositionCommand *)msgin;
    uint8_t send_index = 0;
    uint8_t buffer[4];
    auto position_clipped = max(-360000000., min(360000000., msg->angular_position * 572957.80)); // 10000. * 180/pi
    buffer_append_int32(buffer, (int32_t)(position_clipped), &send_index);
    comm_can_transit_eid_servo(msg->id | ((uint32_t) CAN_PACKET_SET_POS << 8), buffer, send_index);
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
    comm_can_transit_eid_servo(msg->id | ((uint32_t) CAN_PACKET_SET_POS_SPD << 8), buffer, send_index);
}

void TmotorDriver::set_motor_control_callback(const void *msgin) {
    const auto * msg = (const custom_messages__msg__TmotorMotorControlCommand *)msgin;
    uint8_t buffer[8];
    auto angular_position_int = float_to_uint_w_bounds(msg->angular_position, T_MOTOR_P_MIN, T_MOTOR_P_MAX, 16);
    auto k_p_int = float_to_uint_w_bounds(msg->k_p, T_MOTOR_K_P_MIN, T_MOTOR_K_P_MAX, 12);
    auto angular_velocity_int = float_to_uint_w_bounds(msg->angular_velocity, T_MOTOR_V_MIN, T_MOTOR_V_MAX, 12);
    auto k_d_int = float_to_uint_w_bounds(msg->k_d, T_MOTOR_K_D_MIN, T_MOTOR_K_D_MAX, 12);
    auto torque_int = float_to_uint_w_bounds(msg->torque, T_MOTOR_T_MIN, T_MOTOR_T_MAX, 12);

    buffer[0] = angular_position_int >> 8;                            // position 8 higher
    buffer[1] = angular_position_int & 0xFF;                          // position 8 lower
    buffer[2] = angular_velocity_int >> 4;                            // velocity 8 higher
    buffer[3] = ((angular_velocity_int & 0xF) << 4) | (k_p_int >> 8); // velocity 4 lower + k_p 4 higher
    buffer[4] = k_p_int & 0xFF;                                       // k_p 8 lower
    buffer[5] = k_d_int >> 4;                                         // k_d 8 higher
    buffer[6] = ((k_d_int & 0xF) << 4) | (torque_int >> 8);           // k_d 4 lower + torque 4 higher
    buffer[7] = torque_int & 0xFF;                                    // torque 8 lower

    comm_can_transit_eid_motor(msg->id, buffer, 8);
}

void TmotorDriver::set_motor_mode_callback(const void * req, void * res) {
    custom_messages__srv__TmotorMotorSetMode_Request * req_in = (custom_messages__srv__TmotorMotorSetMode_Request *) req;
    custom_messages__srv__TmotorMotorSetMode_Response * res_in = (custom_messages__srv__TmotorMotorSetMode_Response *) res;

    res_in->success = true;
    switch (req_in->mode) {
        case 1:
            comm_can_transit_eid_motor(req_in->id, enter_mode, 8);
            break;
        case 2:
            comm_can_transit_eid_motor(req_in->id, exit_mode, 8);
            break;
        case 3:
            comm_can_transit_eid_motor(req_in->id, zero_mode, 8);
            break;
        default:
            res_in->success = false;
            break;
    }
}

// Helper functions from the datasheet

void TmotorDriver::comm_can_transit_eid_servo(uint32_t id, const uint8_t* data, uint8_t len) {
    // Force a max length of 8 bytes
    len = len > 8 ? 8 : len;
    CAN_message_t m;
    m.id = id;
    m.flags.extended = true;
    m.len = len;
    for (int i = 0; i < len; ++i) { m.buf[i] = data[i]; }
    CanBus.write(m);
}

void TmotorDriver::comm_can_transit_eid_motor(uint32_t id, const uint8_t* data, uint8_t len) {
    // Force a max length of 8 bytes
    len = len > 8 ? 8 : len;
    CAN_message_t m;
    m.id = id;
    m.flags.extended = false;
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

int TmotorDriver::float_to_uint_w_bounds(double x, double x_min, double x_max, uint16_t bits) {
    x = min(max(x_min, x), x_max);
    double span = x_max - x_min;
    return (int) ((x-x_min) * ((double)(1<<bits)/span));
}

double TmotorDriver::uint_to_float(int x, double x_min, double x_max, uint16_t bits) {
    double span = x_max - x_min;
    return ((double)x)*span/((double)((1<<bits)-1)) + x_min;
}