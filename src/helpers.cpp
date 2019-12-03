//
// Created by pohzhiee on 11/26/19.
//

#include "ros2_uart_agent/helpers.hpp"


#include <iostream>
#include <iomanip>
#include <cstdio>
#include <vector>
#include <deque>
#include <numeric>
#include <vector>

namespace helpers{

    std::array<std::uint_fast32_t, 256> generate_crc_lookup_table() noexcept
    {
        auto const reversed_polynomial = std::uint_fast32_t{0xEDB88320uL};

        // This is a function object that calculates the checksum for a value,
        // then increments the value, starting from zero.
        struct byte_checksum
        {
            std::uint_fast32_t operator()() noexcept
            {
                auto checksum = static_cast<std::uint_fast32_t>(n++);

                for (auto i = 0; i < 8; ++i)
                    checksum = (checksum >> 1) ^ ((checksum & 0x1u) ? reversed_polynomial : 0);

                return checksum;
            }

            unsigned n = 0;
        };

        auto table = std::array<std::uint_fast32_t, 256>{};
        std::generate(table.begin(), table.end(), byte_checksum{});

        return table;
    }

    std::tuple<long, long, long> get_control_char_positions(const std::vector<char> &array) {
        auto stx_it = std::find(array.cbegin(), array.cend(), '\x02');
        auto etx_it = std::find (stx_it, array.cend(), '\x03');
        auto eot_it = std::find (etx_it, array.cend(), '\x04');
        if (etx_it != array.cend() && eot_it != array.cend() && stx_it != array.cend())
        {
            long stx_pos = stx_it - array.cbegin() + 1;
            // std::cout << "STX found at position : " << stx_pos << std::endl;
            long etx_pos = etx_it - array.cbegin() + 1;
            // std::cout << "ETX found at position : " << etx_pos << std::endl;
            long eot_pos = eot_it - array.cbegin() + 1;
            // std::cout << "EOT found at position : " << eot_pos << std::endl;
            if(stx_pos > etx_pos){
                return {-1, -1, -1};
            }
            if(etx_pos > eot_pos){
                return {-1, -1, -1};
            }
            return {stx_pos, etx_pos, eot_pos};
        }
        return {-1,-1,-1};
    }

    std::tuple<bool, std::vector<char>> process_input(const std::vector<char> &array) {
        const auto [stx_pos, etx_pos, eot_pos] = get_control_char_positions(array);
        if(stx_pos == -1){
            return {false, {}};
        }
        std::basic_string_view<char> main_content(&array[stx_pos], etx_pos-stx_pos-1);
        std::basic_string_view<char> crc(&array[etx_pos], eot_pos-etx_pos-1);

        auto self_generated_crc = helpers::CRC32(main_content.cbegin(), main_content.cend());
        char crc_buffer[30];
        auto crc_length = sprintf(crc_buffer, "%X", self_generated_crc);
        std::string_view crc_generated(crc_buffer, crc_length);
        // std::cout << "Substring: " << main_content << std::endl;
        // std::cout << "CRC32: " << crc << std::endl;
        // std::cout << "CRC32 Generated: " << crc_generated << std::endl;

        return {crc == crc_generated, std::vector<char>(main_content.cbegin(), main_content.cend())};
    }

    int get_count(const std::vector<char> &array){
        auto num_string = std::string(array.end()-6, array.end());
        return std::stoi(num_string);
    }

    std::array<std::deque<double>,3> previous_velocities{};
    std::array<double, 3> previous_joint_states{};
    std::tuple<int32_t, uint32_t> previous_time{};
    constexpr int queue_max_depth = 8;

    std::unique_ptr<JointState> create_joint_state_msg(const std::array<char, 128> &array, int32_t sec, uint32_t nsec){
        auto joint_states = helpers::get_joint_states(array);
        if (joint_states[0] < -5) {
            // This check is as such because on error case the angles are set to -100
            return nullptr;
        }

        auto joint_state = std::make_unique<JointState>();
        std::vector<double> velocity_array{};
        velocity_array.resize(3);
        for(unsigned int i = 0;i<joint_states.size();i++){
            auto &vel_deque = previous_velocities.at(i);
            auto [prev_sec, prev_nsec] = previous_time;
            auto time_diff = (sec-prev_sec) * 1000000000 + (nsec - prev_nsec);
            auto pos_diff = previous_joint_states[i] - joint_states[i];
            auto current_vel = pos_diff * 1000000000 / (double)time_diff;
            vel_deque.push_back(current_vel);
            if(vel_deque.size() > queue_max_depth){
                vel_deque.pop_front();
            }
            auto sum = std::accumulate(vel_deque.cbegin(), vel_deque.cend(), 0.0);
            velocity_array[i] = sum/vel_deque.size();
            previous_joint_states[i] = joint_states[i];
        }
        previous_time = {sec, nsec};

        joint_state->header.stamp.sec = sec;
        joint_state->header.stamp.nanosec = nsec;
        joint_state->name = {"Joint1", "Joint2", "Joint3"};
        joint_state->position = std::vector<double>({joint_states[0], joint_states[1], joint_states[2]});
        joint_state->velocity = velocity_array;
        // joint_state->velocity = {};
        return joint_state;
    }
}