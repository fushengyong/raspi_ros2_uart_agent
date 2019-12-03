// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <numeric>
#include <memory>
#include <atomic>
#include <mutex>
#include <sstream>
#include <string>
#include <string_view>
#include <iomanip>
#include <thread>
#include <iostream>        // Include all needed libraries here
#include <wiringPi.h>
#include <wiringSerial.h>

#include "rclcpp/rclcpp.hpp"
#include "ros2_uart_agent/helpers.hpp"
#include "ros2_control_interfaces/msg/joint_control.hpp"
#include <sensor_msgs/msg/joint_state.hpp>


using std::placeholders::_1;
using ros2_control_interfaces::msg::JointControl;
using sensor_msgs::msg::JointState;
class MinimalSubscriber : public rclcpp::Node {
public:
    MinimalSubscriber()
            : Node("minimal_subscriber") {
        if ((device_ = serialOpen("/dev/serial0", 1000000)) < 0) {
            std::cerr << "Unable to open serial port" << std::endl;
        }
        subscription_ = this->create_subscription<JointControl>(
                "/arm_standalone/control", 300, std::bind(&MinimalSubscriber::topic_callback, this, _1));
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1000);
    }

    int device_ = 0;
    rclcpp::Publisher<JointState>::SharedPtr publisher_;
private:
    void topic_callback(const JointControl::SharedPtr msg) const {
        // RCLCPP_INFO(this->get_logger(), "Msg time: %ld %ld", msg->header.stamp.sec, msg->header.stamp.nanosec);
        std::array<char, 128> buffer{};
        int pos = 0;
        std::ostringstream oss;
        for(unsigned int i = 0;i<msg->joints.size();i++){
            // RCLCPP_INFO(this->get_logger(), "Joint: %s, Value: %f", msg->joints[i].c_str(), msg->goals[i]);
            // serialPrintf(device_, "%f\n", msg->goals[i]);
            int chars_written;
            if (i == msg->joints.size()-1){
                chars_written = std::sprintf(buffer.data()+pos, "% 08f\n", msg->goals[i]);
            }
            else{
                chars_written = std::sprintf(buffer.data()+pos, "% 08f\t", msg->goals[i]);
            }
            if(chars_written > 0){
                pos += chars_written;
            }
        }
        std::string_view joint_values_sv(buffer.data(), pos);
        // std::cout << joint_values_sv << "pos: " << pos << (buffer[pos] == '\0') << ' ' << joint_values_sv.length() << std::endl;
        std::array<char, 256> message_buffer{};
        auto message_length = helpers::generate_message(message_buffer, joint_values_sv, msg->header.stamp.sec, msg->header.stamp.nanosec);
        std::string_view message_sv(message_buffer.data(), message_length);
        serialPrintf(device_, "%s", message_sv.data());
    }
    rclcpp::Subscription<JointControl>::SharedPtr subscription_;
};

std::atomic_bool data_ready{false};
std::atomic_uint32_t valid_count{0};
std::vector<char> rx_data{};
std::mutex buffer_mutex;

void publish_data(std::atomic_bool &stop_flag, rclcpp::Publisher<JointState>::SharedPtr &publisher_){
    while(!stop_flag.load()){
        if(data_ready){
            std::vector<char> local_rx_data;
            {
                std::lock_guard<std::mutex> lck (buffer_mutex);
                local_rx_data = std::move(rx_data);
                rx_data.clear();
            }
            data_ready = false;
            auto payload_opt = helpers::process_input(local_rx_data);
            if (payload_opt) {
                valid_count++;
                auto [rx_payload, sec, nsec] = payload_opt.value();
                auto msg = helpers::create_joint_state_msg(rx_payload, sec, nsec);
                if(msg){
                    publisher_->publish(*msg);
                }
            }
        }
    }
}

void read_serial(std::atomic_bool &stop_flag, int device){
    // std::array<char, 1024> buffer;
    std::vector<char> buffer(1024);
    int count = 0;
    while(!stop_flag.load()){
        while (serialDataAvail(device)) {
            auto char_val = static_cast<char>(serialGetchar(device));
            buffer.emplace_back(char_val);
            if(char_val == '\x04'){
                {
                    std::lock_guard<std::mutex> lck (buffer_mutex);
                    rx_data = std::move(buffer);
                }
                data_ready = true;
                count++;
                buffer.clear();
            }
        }
    }
    std::cout << "Received: " << count << ", Valid: " << valid_count.load() << std::endl;
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::atomic_bool stop_thread_1{false};
    wiringPiSetup();            // Setup the library/
    auto subscriber = std::make_shared<MinimalSubscriber>();
    auto func1 = [&stop_thread_1, &subscriber](){
        read_serial(stop_thread_1, subscriber->device_);};
    auto func2 = [&stop_thread_1, &subscriber](){
        publish_data(stop_thread_1, subscriber->publisher_);
    };
    std::thread first (func1);
    std::thread second (func2);

    rclcpp::spin(subscriber);
    stop_thread_1.store(true);
    first.join();
    second.join();
    rclcpp::shutdown();
    return 0;
}
