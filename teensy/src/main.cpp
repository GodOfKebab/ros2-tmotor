#include <ros_tmotor.h>
#include <TmotorDriver.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;


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

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

    TmotorDriver::init(&executor, &node);
}

void loop() {
    TmotorDriver::check_events();
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
}