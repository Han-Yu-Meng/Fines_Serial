/*******************************************************************************
 * Copyright (c) 2025.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#include <fins/node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <queue>
#include <mutex>
#include <chrono>
#include <thread>
#include <atomic>

#include "serial/serial.h"
#include "msg_types.hpp"
#include "utils/crc.hpp"
#include "utils/data_convert.hpp"

struct SerialConfig {
    std::string port = "/dev/ttyUSB0";
    uint32_t baudrate = 921600;
    int tx_handle_period = 1; // ms
    int rx_handle_period = 1; // ms
};

class SerialStationNode : public fins::Node {
public:
    void define() override {
        set_name("SerialStationNode");
        set_description("Migration of Fines_Serial to FINS framework.");
        set_category("Navigation>Communication");

        // FINS Input: Subscribe to /cmd_vel
        register_input<0, geometry_msgs::msg::Twist>("cmd_vel", &SerialStationNode::on_cmd_vel);

        // Parameters
        register_parameter<std::string>("port", &SerialStationNode::on_port_changed, "/dev/ttyUSB0");
        register_parameter<int>("baudrate", &SerialStationNode::on_baudrate_changed, 921600);
    }

    void initialize() override {
        open_serial();
    }

    void run() override {
        if (is_running_) return;
        is_running_ = true;
        
        // Start TX and RX threads
        tx_thread_ = std::thread(&SerialStationNode::tx_handle_loop, this);
        rx_thread_ = std::thread(&SerialStationNode::rx_handle_loop, this);
    }

    void pause() override {
        stop_threads();
    }

    void reset() override {
        std::lock_guard<std::mutex> lock(tx_mutex_);
        while(!tx_queue_.empty()) tx_queue_.pop();
    }

    ~SerialStationNode() {
        stop_threads();
        std::lock_guard<std::mutex> lock(serial_mutex_);
        if (serial_.isOpen()) {
            serial_.close();
        }
    }

private:
    void on_port_changed(const std::string &v) {
        if (config_.port == v) return;
        config_.port = v;
        logger->info("Port changed to {}. Reopening...", v);
        reopen_serial();
    }

    void on_baudrate_changed(const int &v) {
        if (config_.baudrate == (uint32_t)v) return;
        config_.baudrate = (uint32_t)v;
        logger->info("Baudrate changed to {}. Reopening...", v);
        reopen_serial();
    }

    void open_serial() {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        try {
            serial_.setPort(config_.port);
            serial_.setBaudrate(config_.baudrate);
            serial::Timeout timeout(0, 0, 0, 0, 0); // Non-blocking
            serial_.setTimeout(timeout);
            serial_.open();
            if (serial_.isOpen()) {
                logger->info("Serial port {} opened successfully.", config_.port);
                serial_.flush();
            } else {
                logger->error("Failed to open serial port {}.", config_.port);
            }
        } catch (const std::exception &e) {
            logger->error("Serial Open Exception: {}", e.what());
        }
    }

    void reopen_serial() {
        stop_threads();
        {
            std::lock_guard<std::mutex> lock(serial_mutex_);
            if (serial_.isOpen()) serial_.close();
        }
        open_serial();
        run();
    }

    void stop_threads() {
        is_running_ = false;
        if (tx_thread_.joinable()) tx_thread_.join();
        if (rx_thread_.joinable()) rx_thread_.join();
    }

    // FINS Callback for cmd_vel
    void on_cmd_vel(const fins::Msg<geometry_msgs::msg::Twist> &msg) {
        std::vector<uint8_t> data;
        
        // Use convert_float2bytes from utils/data_convert.hpp
        convert_float2bytes(data, msg->linear.x);
        convert_float2bytes(data, msg->linear.y);
        convert_float2bytes(data, msg->linear.z);
        convert_float2bytes(data, msg->angular.x);
        convert_float2bytes(data, msg->angular.y);
        convert_float2bytes(data, msg->angular.z);

        // Encode frame
        encode_frame(data, (uint8_t)CMD_VEL);
        
        // Push to TX queue
        {
            std::lock_guard<std::mutex> lock(tx_mutex_);
            tx_queue_.push(std::move(data));
        }
    }

    void encode_frame(std::vector<uint8_t>& data, uint8_t frame_identifier) {
        const uint8_t FRAME_HEADER = 0xAA;
        const uint8_t FRAME_TRAILER = 0xBB;
        
        uint8_t crc = calculate_crc(data);
        uint8_t data_length = static_cast<uint8_t>(data.size());

        data.insert(data.begin(), data_length);
        data.insert(data.begin(), frame_identifier);
        data.insert(data.begin(), FRAME_HEADER);

        data.push_back(crc);
        data.push_back(FRAME_TRAILER);
    }

    void tx_handle_loop() {
        while (is_running_) {
            std::vector<uint8_t> data_to_send;
            {
                std::lock_guard<std::mutex> lock(tx_mutex_);
                if (!tx_queue_.empty()) {
                    data_to_send = std::move(tx_queue_.front());
                    tx_queue_.pop();
                }
            }

            if (!data_to_send.empty()) {
                std::lock_guard<std::mutex> s_lock(serial_mutex_);
                if (serial_.isOpen()) {
                    try {
                        serial_.write(data_to_send);
                    } catch (const std::exception &e) {
                        logger->error("Serial Write Exception: {}", e.what());
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(config_.tx_handle_period));
        }
    }

    void rx_handle_loop() {
        std::vector<uint8_t> rx_buffer;
        rx_buffer.reserve(200);
        
        while (is_running_) {
            size_t bytes_read = 0;
            {
                std::lock_guard<std::mutex> s_lock(serial_mutex_);
                if (serial_.isOpen()) {
                    try {
                        bytes_read = serial_.read(rx_buffer, 200);
                    } catch (const std::exception &e) {
                        logger->error("Serial Read Exception: {}", e.what());
                    }
                }
            }

            if (bytes_read > 0) {
                // Mimic original decode(): log hex received
                std::string hex_str;
                for (size_t i = 0; i < bytes_read; ++i) {
                    hex_str += fmt::format("{:02X} ", rx_buffer[i]);
                }
                logger->info("RX Data (Total {} bytes): {}", bytes_read, hex_str);
                rx_buffer.clear();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(config_.rx_handle_period));
        }
    }

private:
    serial::Serial serial_;
    std::mutex serial_mutex_;
    SerialConfig config_;

    std::queue<std::vector<uint8_t>> tx_queue_;
    std::mutex tx_mutex_;

    std::atomic<bool> is_running_{false};
    std::thread tx_thread_;
    std::thread rx_thread_;
};

EXPORT_NODE(SerialStationNode)
DEFINE_PLUGIN_ENTRY()
