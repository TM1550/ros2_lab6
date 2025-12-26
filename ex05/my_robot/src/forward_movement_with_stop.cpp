#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <memory>

class ForwardMovementWithStop : public rclcpp::Node {
public:
    ForwardMovementWithStop() : Node("forward_movement_with_stop") {
        // Объявление параметров с значениями по умолчанию
        this->declare_parameter("forward_speed", 0.3);
        this->declare_parameter("stop_distance", 0.5);
        this->declare_parameter("resume_distance", 0.7);
        this->declare_parameter("wait_before_start", 5.0);
        this->declare_parameter("center_region_size", 20);
        this->declare_parameter("log_throttle_duration", 1.0);
        this->declare_parameter("min_valid_pixels", 50);  // Новый параметр

        // Получение параметров
        forward_speed_ = this->get_parameter("forward_speed").as_double();
        stop_distance_ = this->get_parameter("stop_distance").as_double();
        resume_distance_ = this->get_parameter("resume_distance").as_double();
        wait_before_start_ = this->get_parameter("wait_before_start").as_double();
        center_region_size_ = this->get_parameter("center_region_size").as_int();
        log_throttle_duration_ = this->get_parameter("log_throttle_duration").as_double();
        min_valid_pixels_ = this->get_parameter("min_valid_pixels").as_int();

        // Издатели и подписчики
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
        
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.best_effort();
        
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/depth/image",
            qos,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                this->process_image(msg);
            }
        );

        // Инициализация
        start_time_ = std::chrono::high_resolution_clock::now();
        is_moving_ = false;
        is_waiting_ = true;
        last_min_distance_ = 10.0f;  // Большое значение по умолчанию
        
        RCLCPP_INFO(this->get_logger(), "Forward movement node initialized");
        RCLCPP_INFO(this->get_logger(), "Waiting %.1f seconds before starting...", wait_before_start_);
        RCLCPP_INFO(this->get_logger(), "Forward speed: %.2f m/s", forward_speed_);
        RCLCPP_INFO(this->get_logger(), "Stop distance: %.2f m", stop_distance_);
        RCLCPP_INFO(this->get_logger(), "Resume distance: %.2f m", resume_distance_);
    }

private:
    // Вспомогательные методы
    void process_image(const sensor_msgs::msg::Image::SharedPtr msg) {
        auto current_time = std::chrono::high_resolution_clock::now();
        
        // Проверка времени ожидания перед стартом
        if (is_waiting_) {
            auto elapsed = std::chrono::duration<double>(current_time - start_time_).count();
            if (elapsed < wait_before_start_) {
                // Троттлинг логов ожидания
                auto throttle_elapsed = std::chrono::duration<double>(current_time - last_log_time_).count();
                if (throttle_elapsed >= 1.0) {
                    RCLCPP_INFO(this->get_logger(), "Wait: %.1f s", wait_before_start_ - elapsed);
                    last_log_time_ = current_time;
                }
                return;
            } else {
                is_waiting_ = false;
                RCLCPP_INFO(this->get_logger(), "Starting forward movement");
            }
        }

        float min_distance = 10.0f;  // Значение по умолчанию - большое расстояние
        auto cmd = std::make_unique<geometry_msgs::msg::Twist>();

        try {
            if (msg->encoding == "32FC1") {
                // Упрощенный расчет минимальной дистанции в центральной области
                min_distance = calculate_min_distance(msg);
                
                // Логика управления движением с гистерезисом
                update_movement_state(min_distance, cmd.get());
            } else {
                // Обработка неподдерживаемого формата изображения
                RCLCPP_ERROR(this->get_logger(), "Unsupported image encoding: %s", msg->encoding.c_str());
                cmd->linear.x = 0.0;
            }
        } catch (const std::exception& e) {
            // Обработка исключений
            RCLCPP_ERROR(this->get_logger(), "Error processing depth image: %s", e.what());
            cmd->linear.x = 0.0;
        }

        // Всегда устанавливаем нулевую угловую скорость
        cmd->angular.z = 0.0;
        
        // Публикация команды управления
        cmd_vel_pub_->publish(*cmd);
        
        // Логирование состояния с троттлингом
        log_state(current_time, min_distance);
    }

    float calculate_min_distance(const sensor_msgs::msg::Image::SharedPtr msg) {
        auto* data = reinterpret_cast<const float*>(msg->data.data());
        int width = msg->width;
        int height = msg->height;
        
        // Расчет границ центральной области (упрощенная версия)
        int center_x = width / 2;
        int center_y = height / 2;
        int half_size = center_region_size_ / 2;
        
        int start_x = std::max(0, center_x - half_size);
        int end_x = std::min(width, center_x + half_size);
        int start_y = std::max(0, center_y - half_size);
        int end_y = std::min(height, center_y + half_size);
        
        // Поиск минимального расстояния в центральной области
        float min_distance = 10.0f;
        
        for (int y = start_y; y < end_y; y++) {
            int row_start = y * width;
            for (int x = start_x; x < end_x; x++) {
                int idx = row_start + x;
                float depth = data[idx];
                
                // Проверка на валидность данных
                if (!std::isnan(depth) && !std::isinf(depth) && depth > 0) {
                    min_distance = std::min(min_distance, depth);
                }
            }
        }
        
        return min_distance;
    }

    void update_movement_state(float min_distance, geometry_msgs::msg::Twist* cmd) {
        if (is_moving_) {
            // Если движемся и обнаружили препятствие - останавливаемся
            if (min_distance < stop_distance_) {
                cmd->linear.x = 0.0;
                is_moving_ = false;
                RCLCPP_WARN(this->get_logger(), "Stopping! Obstacle at %.2f m", min_distance);
            } else {
                // Продолжаем движение
                cmd->linear.x = forward_speed_;
            }
        } else {
            // Если остановились и препятствие достаточно далеко - возобновляем движение
            if (min_distance >= resume_distance_) {
                cmd->linear.x = forward_speed_;
                is_moving_ = true;
                RCLCPP_INFO(this->get_logger(), "Resuming movement. Clear path ahead (%.2f m)", min_distance);
            } else {
                // Остаемся на месте
                cmd->linear.x = 0.0;
            }
        }
    }

    void log_state(const std::chrono::high_resolution_clock::time_point& current_time, float min_distance) {
        auto throttle_elapsed = std::chrono::duration<double>(current_time - last_log_time_).count();
        
        // Логирование состояния с троттлингом
        if (throttle_elapsed >= log_throttle_duration_) {
            const char* state_str = is_moving_ ? "MOVING" : "STOPPED";
            if (is_moving_) {
                RCLCPP_INFO(this->get_logger(), "Moving forward. Min distance: %.2f m", min_distance);
            } else if (min_distance < resume_distance_) {
                RCLCPP_INFO(this->get_logger(), "Stopped. Obstacle at %.2f m", min_distance);
            } else {
                RCLCPP_INFO(this->get_logger(), "State: %s, Distance: %.2f m", state_str, min_distance);
            }
            last_log_time_ = current_time;
        }
    }

    // Члены класса
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    
    std::chrono::high_resolution_clock::time_point start_time_;
    std::chrono::high_resolution_clock::time_point last_log_time_;
    
    // Параметры
    double forward_speed_;
    double stop_distance_;
    double resume_distance_;
    double wait_before_start_;
    double log_throttle_duration_;
    int center_region_size_;
    int min_valid_pixels_;  // Минимальное количество валидных пикселей для принятия решения
    
    // Состояние
    bool is_moving_;
    bool is_waiting_;
    float last_min_distance_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ForwardMovementWithStop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}