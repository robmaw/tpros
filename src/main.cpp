#include <TinyPICO.h>
#include <wificonfig.h>
#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

// #if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
// #error This example is only avaliable for Arduino framework with serial transport.
// #endif

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

TinyPICO tp = TinyPICO();

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
  }
}

void setup() {
  tp.DotStar_SetPixelColor( 255, 128, 0 );
  // Configure serial transport
  Serial.begin(115200);
  //set_microros_serial_transports(Serial);
  //set_microros_wifi_transports("WIFI SSID", "WIFI PASS", "192.168.1.57", 8888);
  set_microros_wifi_transports(ssid,psk,agent_ip,agent_port)
  delay(2000);

  allocator = rcl_get_default_allocator();
  
  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_platformio_node_publisher"));
  

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg.data = 42;
  //delay(1000);
  tp.DotStar_SetPixelColor( 0, 255, 0 );
}

void loop() {
  //tp.DotStar_SetPixelColor( 255, 128, 0 );
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  //tp.DotStar_Clear();
}


// /**
//  * Blink
//  *
//  * Turns on an LED on for one second,
//  * then off for one second, repeatedly.
//  */
// #include "Arduino.h"

// #ifndef LED_BUILTIN
// #define LED_BUILTIN 13
// #endif

// void setup()
// {
//   // initialize LED digital pin as an output.
//   pinMode(LED_BUILTIN, OUTPUT);
// }

// void loop()
// {
//   // turn the LED on (HIGH is the voltage level)
//   digitalWrite(LED_BUILTIN, HIGH);

//   // wait for a second
//   delay(1000);

//   // turn the LED off by making the voltage LOW
//   digitalWrite(LED_BUILTIN, LOW);

//    // wait for a second
//   delay(1000);
// }