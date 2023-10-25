#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <FlexCAN_T4.h>
#include <custom_messages/msg/tmotor_state.h>
#include <std_msgs/msg/float64.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

rcl_publisher_t publisher;
rcl_subscription_t subscriber;

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

uint32_t tmotor_id;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

void tmotor_sniffer(const CAN_message_t &can_msg) {
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
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
}


void buffer_append_int32(uint8_t* buffer, int32_t number, uint8_t* index) {
    buffer[(*index)++] = number >> 24;
    buffer[(*index)++] = number >> 16;
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void buffer_append_int16(uint8_t* buffer, int16_t number, uint8_t* index) {
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void comm_can_transit_eid(uint32_t id, const uint8_t* data, uint8_t len) {
    // Force a max length of 8 bytes
    len = len > 8 ? 8 : len;
    CAN_message_t msg;
    msg.id = id;
    msg.flags.extended = true;
    msg.len = len;
    for (int i = 0; i < len; ++i) { msg.buf[i] = data[i]; }
    Can0.write(msg);
}

typedef enum {
    CAN_PACKET_SET_DUTY = 0,      // Duty cycle mode
    CAN_PACKET_SET_CURRENT,       // Current loop mode
    CAN_PACKET_SET_CURRENT_BRAKE, // Current brake mode
    CAN_PACKET_SET_RPM,           // Velocity mode
    CAN_PACKET_SET_POS,           // Position mode
    CAN_PACKET_SET_ORIGIN_HERE,   // Set origin mode
    CAN_PACKET_SET_POS_SPD,       // Position velocity loop mode

} CAN_PACKET_ID;

void set_position_callback(const void * msgin) {
    const std_msgs__msg__Float64 * msg = (const std_msgs__msg__Float64 *)msgin;
    float_t pos = msg->data;
    uint8_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(pos * 10000.0), &send_index);
    comm_can_transit_eid((tmotor_id  & 0xFF) | ((uint32_t) CAN_PACKET_SET_POS << 8), buffer, send_index);
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
    RCCHECK(rclc_publisher_init_default(
            &publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(custom_messages, msg, TmotorState),
            "/micro_ros_teensy/motor_state"));

    // create subscriber
    RCCHECK(rclc_subscription_init_default(
            &subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
            "/micro_ros_teensy/set_position"));

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    std_msgs__msg__Float64 msg;
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &set_position_callback, ON_NEW_DATA));

    Can0.begin();
    Can0.setBaudRate(1000000);
    Can0.setMaxMB(16);
    Can0.enableFIFO();
    Can0.enableFIFOInterrupt();
    Can0.onReceive(tmotor_sniffer);
    Can0.mailboxStatus();
}

void loop() {
    Can0.events();
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
}