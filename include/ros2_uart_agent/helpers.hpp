//
// Created by pohzhiee on 11/26/19.
//

#ifndef ROS2_UART_AGENT_HELPER_HPP
#define ROS2_UART_AGENT_HELPER_HPP
#include <algorithm>
#include <array>
#include <tuple>
#include <vector>
#include <cstdint>
#include <numeric>
#include <cstring>
#include <iostream>
#include <memory>
#include <optional>
#include "sensor_msgs/msg/joint_state.hpp"


namespace std_extensions{
   template<class T>
   struct is_array : std::is_array<T> {};
   template<class T, std::size_t N>
   struct is_array<std::array<T, N>> : std::true_type {};
   template<>
   struct is_array<std::string> : std::true_type {};
   template<class T>
   struct is_array<std::vector<T>> : std::true_type {};
}


namespace helpers{
    using sensor_msgs::msg::JointState;
// Generates a lookup table for the checksums of all 8-bit values.
    std::array<std::uint_fast32_t, 256> generate_crc_lookup_table() noexcept;

// Calculates the CRC for any sequence of values. (You could use type traits and a
// static assert to ensure the values can be converted to 8 bits.)
    template<typename InputIterator>
    std::uint_fast32_t CRC32(InputIterator first, InputIterator last) {
        // Generate lookup table only on first use then cache it - this is thread-safe.
        static auto const table = generate_crc_lookup_table();

        // Calculate the checksum - make sure to clip to 32 bits, for systems that don't
        // have a true (fast) 32-bit type.
        return std::uint_fast32_t{0xFFFFFFFFuL} &
               ~std::accumulate(first, last,
                                ~std::uint_fast32_t{0} & std::uint_fast32_t{0xFFFFFFFFuL},
                                [](std::uint_fast32_t checksum, std::uint_fast8_t value) {
                                    return table[(checksum ^ value) & 0xFFu] ^ (checksum >> 8);
                                });
    }

    /**
     * Obtain the control character positions from a uart transmission
     * @tparam T std::array<char, N> is expected
     * @param array incoming data from UART
     * @return a tuple of STX, ETX, and EOT, in that order
     */
    template<typename T>
    std::tuple<long, long, long, long> get_control_char_positions(const T &array);

    /**
     * Checks whether the input is valid and returns the payload if valid
     * @tparam T std::array<char, N> is expected
     * @param array
     * @return optional payload char array
     */
    template<typename T>
    std::optional<std::tuple<std::array<char, 128>, uint32_t, uint32_t>> process_input(const T& array);

    int get_count(const std::vector<char> &array);


    /**
     * Generates a standard message with CRC32 and control characters in the proper places.
     * Currently this message is expected to have time component in header
     * @tparam T std::array<char, N> is expected
     * @param data_array buffer to store the data of the generated message
     * @param payload_sv payload of the message
     * @param sec seconds portion of current time
     * @param nsec nanoseconds portion of current time
     * @return
     */
    template<typename T>
    unsigned long
    generate_message(T &data_array, const std::string_view &payload_sv, const int32_t sec, const uint32_t nsec);

    /**
    * 
    * @tparam N Degrees, either 180 or 270 for the LDX servo for lobot
    * @param adc_value adc reading from microcontroller
    * @param low_val adc reading at lowest angle (-90/-135 deg)
    * @param high_val adc reading at highest angle (90/135 deg)
    * @return corresponding joint angle value in radians
    */
    template<long N>
    double calculate_joint_state_from_adc(long adc_value, long low_val, long high_val);

    template<typename T>
    std::array<double,3> get_joint_states(const T &array);

    std::unique_ptr<JointState> create_joint_state_msg(const std::array<char, 128> &array, int32_t sec, uint32_t nsec);
}

#include "helpers.tpp"


#endif //UNTITLED1_HELPER_TEST_HPP
