#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <functional>
#include <fstream>
#include <chrono>
#include "telemetry_pkg/json.hpp"

using json = nlohmann::json;

class Telemetry_publisher : public rclcpp::Node{
    public:
    Telemetry_publisher() : Node("telemetryPublisher_node"){

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&Telemetry_publisher::callback, this));


        std::ifstream input_file(path);
        data = json::parse(input_file);

        // Publishers  
        armMaxRpm = create_publisher<std_msgs::msg::Float32>(
            "/telemetryJSON/arm_max_rpm", 10
        );

        // Subscriber for arm telemetry
        vescTelemetrySub = create_subscription<std_msgs::msg::Float32MultiArray>(
            "/arm/telemetry", 10,
            std::bind(&Telemetry_publisher::armTelemetryCallback, this, std::placeholders::_1)
        );
    }

    private:
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("telemetry_pkg");
    std::string path = package_share_directory + "/config/config.json";
    void callback(){
        auto msg = std_msgs::msg::Float32();
        msg.data = data["motors"][4]["rpm_limit"].get<float>();
        armMaxRpm->publish(msg);
    }

    void armTelemetryCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg){
        // simple log of received values
        RCLCPP_INFO(this->get_logger(), "Received arm telemetry with %zu entries",
                    msg->data.size());
        if(!msg->data.empty()){
            std::ostringstream oss;
            for(size_t i=0;i<msg->data.size();++i){
                oss << msg->data[i];
                if(i+1<msg->data.size()) oss << ", ";
            }
            RCLCPP_INFO(this->get_logger(), "Values: %s", oss.str().c_str());
        }
    }

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr armMaxRpm;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr vescTelemetrySub;
    json data;

    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Telemetry_publisher>());
  rclcpp::shutdown();
  return 0;
}