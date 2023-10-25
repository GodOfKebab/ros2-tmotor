#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <FlexCAN_T4.h>
#include <custom_messages/msg/tmotor_state.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

rcl_publisher_t publisher;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
    while(1) {
        delay(100);
    }
}

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

void tmotor_sniffer(const CAN_message_t &can_msg) {
    custom_messages__msg__TmotorState msg;
    // Assume failure at teensy-side, if not override
    msg.error_code = 7;
    if (can_msg.len == 8) {
        msg.angular_position = (int16_t)(can_msg.buf[1] | ((int16_t)can_msg.buf[0] << 8)) * 0.00174533;  // 1 / 10 * 6 * pi / 180
        msg.angular_velocity = (int16_t)(can_msg.buf[3] | ((int16_t)can_msg.buf[2] << 8)) * 0.01047198;  // 1 / 10 * 6 * pi / 180
        msg.current          = (int16_t)(can_msg.buf[5] | ((int16_t)can_msg.buf[4] << 8)) / 100.;
        msg.temp             = (int8_t)can_msg.buf[6];
        msg.error_code       = (int8_t)can_msg.buf[7];
    }
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
}

void setup() {
    // Configure serial transport
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    delay(2000);

    allocator = rcl_get_default_allocator();

    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "micro_ros_teensy_node", "", &support));

    // create publisher
    custom_messages__msg__TmotorState msg;
    RCCHECK(rclc_publisher_init_default(
            &publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(custom_messages, msg, TmotorState),
            "/micro_ros_teensy/motor_state"));

    Can0.begin();
    Can0.setBaudRate(1000000);
    Can0.setMaxMB(16);
    Can0.enableFIFO();
    Can0.enableFIFOInterrupt();
    Can0.onReceive(tmotor_sniffer);
    Can0.mailboxStatus();
}

void loop() {
//    delay(100);
    Can0.events();
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
}