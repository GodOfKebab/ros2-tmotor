#pragma once
#include <ros_tmotor.h>
#include <FlexCAN_T4.h>

#include <custom_messages/msg/tmotor_state.h>
#include <custom_messages/msg/tmotor_servo_duty_cycle_command.h>
#include <custom_messages/msg/tmotor_servo_current_loop_command.h>
#include <custom_messages/msg/tmotor_servo_current_brake_command.h>
#include <custom_messages/msg/tmotor_servo_velocity_command.h>
#include <custom_messages/msg/tmotor_servo_position_command.h>
#include <custom_messages/msg/tmotor_servo_position_velocity_loop_command.h>

typedef enum {
    CAN_PACKET_SET_DUTY = 0,      // Duty cycle mode
    CAN_PACKET_SET_CURRENT,       // Current loop mode
    CAN_PACKET_SET_CURRENT_BRAKE, // Current brake mode
    CAN_PACKET_SET_RPM,           // Velocity mode
    CAN_PACKET_SET_POS,           // Position mode
    CAN_PACKET_SET_ORIGIN_HERE,   // Set origin mode
    CAN_PACKET_SET_POS_SPD,       // Position velocity loop mode

} CAN_PACKET_ID;

class TmotorDriver {

public:
    static void init(rclc_executor_t* executor, rcl_node_t* node);
    static void check_events();

private:

    // CAN Bus callback for state updates that publishes to ROS2 topic /micro_ros_teensy/motor_state
    static void tmotor_state_sniffer(const CAN_message_t &can_msg);

    // ROS2 topic /micro_ros_teensy/set_duty_cycle callback handler
    static void set_duty_cycle_callback(const void * msgin);

    // ROS2 topic /micro_ros_teensy/set_current_loop callback handler
    static void set_current_loop_callback(const void * msgin);

    // ROS2 topic /micro_ros_teensy/set_current_brake callback handler
    static void set_current_brake_callback(const void * msgin);

    // ROS2 topic /micro_ros_teensy/set_velocity callback handler
    static void set_velocity_callback(const void * msgin);

    // ROS2 topic /micro_ros_teensy/set_position callback handler
    static void set_position_callback(const void * msgin);

    // ROS2 topic /micro_ros_teensy/set_position_velocity_loop callback handler
    static void set_position_velocity_loop_callback(const void * msgin);

    // Helper functions from the datasheet
    static void comm_can_transit_eid(uint32_t id, const uint8_t* data, uint8_t len);
    static void buffer_append_int32(uint8_t* buffer, int32_t number, uint8_t* index);
    static void buffer_append_int16(uint8_t* buffer, int16_t number, uint8_t* index);

};
