namespace helpers{
    template<typename T>
    std::tuple<long, long, long, long> get_control_char_positions(const T &array) {
        auto soh_it = std::find(array.cbegin(), array.cend(), '\x01');
        auto stx_it = std::find(soh_it, array.cend(), '\x02');
        auto etx_it = std::find (stx_it, array.cend(), '\x03');
        auto eot_it = std::find (etx_it, array.cend(), '\x04');
        long soh_pos = soh_it - array.cbegin() + 1;
        long stx_pos = stx_it - array.cbegin() + 1;
        long eot_pos = eot_it - array.cbegin() + 1;
        long etx_pos = etx_it - array.cbegin() + 1;
        if (etx_it != array.cend() && eot_it != array.cend() && stx_it != array.cend() && soh_it != array.cend())
        {
//            std::cout << "STX found at position : " << stx_pos << std::endl;
//            std::cout << "ETX found at position : " << etx_pos << std::endl;
//            std::cout << "EOT found at position : " << eot_pos << std::endl;
            return {soh_pos, stx_pos, etx_pos, eot_pos};
        }
        char print_buffer[90];
        auto char_count = std::sprintf(print_buffer, 
            "Bad pos, soh: %ld, stx: %ld, etx: %ld, eot: %ld", soh_pos, stx_pos, eot_pos, etx_pos);
        std::cout << std::string_view(print_buffer, char_count) << std::endl;
        return {-1,-1,-1,-1};
    }

    template<typename T>
    std::optional<std::tuple<std::array<char, 128>, uint32_t, uint32_t>> process_input(const T& array) {
        const auto [soh_pos, stx_pos, etx_pos, eot_pos] = get_control_char_positions(array);
        if(soh_pos == -1){
            return std::nullopt;
        }
        std::string_view timestamp(&array[soh_pos], stx_pos-soh_pos-1);
        std::basic_string_view<char> main_content(&array[stx_pos], etx_pos-stx_pos-1);
        std::basic_string_view<char> crc(&array[etx_pos], eot_pos-etx_pos-1);
        auto time_tab_iter = std::find(timestamp.cbegin(), timestamp.cend(), '\t');
        uint32_t sec = std::stod(timestamp.data(), nullptr);
        uint32_t nsec = std::stod(&(*(time_tab_iter+1)), nullptr);
        uint_fast32_t self_generated_crc = CRC32(main_content.cbegin(), main_content.cend());
        char crc_buffer[30];
        auto crc_length = sprintf(crc_buffer, "%08X", self_generated_crc);
        std::string_view crc_generated(crc_buffer, crc_length);

        if(crc == crc_generated){
            std::array<char, 128> main_content_array{};
            std::copy(main_content.cbegin(), main_content.cend(), main_content_array.data());
            main_content_array.at(main_content.length()) = '\0';
            return std::make_tuple(main_content_array, sec, nsec);
        }
        else{
            return std::nullopt;
        }
    }

   template<typename T>
    unsigned long
    generate_message(T &data_array, const std::string_view &payload_sv, const int32_t sec, const uint32_t nsec){
        static_assert(std_extensions::is_array<T>::value, "T must be an array type");
        auto len = payload_sv.length();
        auto num_padding_bytes = len % 4 == 0 ? 0 : 4 - (len % 4);
        data_array[0] = '\x01'; // SOH character to mark start of heading
        //insert sec and nsec data into header, separated by TAB character, \t
        auto sec_char_count = std::sprintf(&(data_array.at(1)), "%07u", sec);
        data_array.at(1 + sec_char_count) = '\t';
        auto nsec_char_count = std::sprintf(&(data_array.at(2 + sec_char_count)), "%09u", nsec);
        auto stx_char_pos = nsec_char_count + 2 + sec_char_count;
        data_array.at(stx_char_pos) = '\x02'; // STX character to mark start of text
        // copy the data from the payload to the buffer
        std::copy(payload_sv.cbegin(), payload_sv.cend(), data_array.begin() + 1 + stx_char_pos);
        // pad the payload such that it has a byte count of multiple 4 such that we can do CRC32
        for (unsigned long i = 0; i < num_padding_bytes; i++) {
            data_array.at(stx_char_pos + len + i + 1) = '\x1A'; // SUB character is used to pad
        }
        data_array.at(stx_char_pos + len + num_padding_bytes + 1) = '\x03'; // ETX character to mark end of text
        std::string_view padded_payload_sv(data_array.data() + 1 + stx_char_pos, len + num_padding_bytes);
        auto padded_length = padded_payload_sv.length();
        if (padded_length % 4 != 0) {
            std::cerr << "generate_message payload padding has issue, please check" << std::endl;
        }
        std::uint_fast32_t crc_val = CRC32(padded_payload_sv.cbegin(), padded_payload_sv.cend());
        auto crc_num_chars = std::sprintf(&(data_array.at(stx_char_pos + padded_length + 2)), "%08X", crc_val);
        data_array.at(stx_char_pos + padded_length + crc_num_chars + 2) = '\x04';
        // add null terminator such that C-style operations can be performed successfully on this data array
        data_array.at(stx_char_pos + padded_length + crc_num_chars + 3) = '\0'; 
        return stx_char_pos + padded_length + crc_num_chars + 4;
    }

    template<long N>
    double calculate_joint_state_from_adc(long adc_value, long low_val, long high_val){
        auto range = high_val - low_val;
        auto mid_val = high_val + low_val;
        constexpr double constant = N * 3.14159265358979 / 360.0; // pi/2 for 180, 3pi/4 for 270
        return ((double)adc_value*2.0 - mid_val)/ (double)range * constant;
    }

    template<typename T>
    std::array<double,3> get_joint_states(const T &array){
        if(array[4] == '\t' && array[9] == '\t' && array[14] == '\n'){
            std::basic_string_view<char> data1(array.data(), 5);
            std::basic_string_view<char> data2(array.data()+5, 5);
            std::basic_string_view<char> data3(array.data()+10, 5);
            auto a = std::strtol(data1.data(), nullptr, 10);
            auto b = std::strtol(data2.data(), nullptr, 10);
            auto c = std::strtol(data3.data(), nullptr, 10);
            // Calibrated on 3 DEC 2019, on 1 LDX218 to have adc reading of ~890 when set to -pi/2 and ~3020 when set to pi/2 
            // LDX227 values are assumed to be the same for now
            double joint_1_val = calculate_joint_state_from_adc<270>(a, 890, 3020);
            double joint_2_val = calculate_joint_state_from_adc<180>(b, 890, 3020);
            double joint_3_val = calculate_joint_state_from_adc<180>(c, 890, 3020);
            // std::cout << "Joint 3 val: " << c << ", converted: " << joint_3_val;
            // std::cout << ", original: " << data3 << std::endl;
            return std::array<double, 3>({joint_1_val,joint_2_val,joint_3_val});
        }
        else{
            return {-100,-100,-100};
        }
    }
}