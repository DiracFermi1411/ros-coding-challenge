#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

/**
 * @class ImagePublisher
 * @brief A ROS2 node for publishing images from a file to a specified topic.
 * 
 * This node reads an image from disk using OpenCV and publishes it as a ROS2 
 * image message to a specified topic. The image is published at a fixed rate 
 * (e.g., 10 Hz), which can be modified as needed. It is primarily intended for 
 * testing or demonstration purposes in a robotics context.
 */
class ImagePublisher : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for the ImagePublisher node.
     * 
     * Initializes the publisher, sets up the timer for periodic publishing, 
     * and loads the image from disk. If the image fails to load, the node 
     * will log an error and shut down.
     */
    ImagePublisher() : Node("image_publisher_node")
    {
        // Initialize the publisher to publish messages of type sensor_msgs::msg::Image
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("my_camera/image", 10);
        
        // Create a timer to trigger the publish_image function at a fixed rate (10 Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 100 ms interval for 10 Hz publishing rate
            std::bind(&ImagePublisher::publish_image, this));
        
        // Load an image using OpenCV (ensure the file path is valid)
        image_ = cv::imread("/home/dheeraj/ros-coding-challenge/Robotics88_test/1.jpeg", cv::IMREAD_COLOR);
        
        // Check if the image was loaded successfully
        if (image_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load the image. Please check the file path.");
            rclcpp::shutdown(); // Shut down the node if the image could not be loaded
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Image loaded successfully. Publishing at 10 Hz.");
        }
    }

private:
    /**
     * @brief Publishes the loaded image as a ROS2 image message.
     * 
     * This function is called periodically by the timer. It converts the 
     * OpenCV image to a ROS2 image message using the cv_bridge library, 
     * and then publishes the message to the configured topic.
     */
    void publish_image()
    {
        if (image_.empty()) return; // If the image is not loaded, do nothing

        // Convert the OpenCV image (cv::Mat) to a ROS2 image message
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_).toImageMsg();
        
        // Publish the image message
        image_publisher_->publish(*msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_; // Publisher for the image topic
    rclcpp::TimerBase::SharedPtr timer_;                                    // Timer to control the publishing rate
    cv::Mat image_;                                                         // OpenCV matrix to store the image
};

int main(int argc, char *argv[])
{
    // Initialize the ROS2 system
    rclcpp::init(argc, argv);
    
    // Create an instance of the ImagePublisher node
    auto image_publisher_node = std::make_shared<ImagePublisher>();
    
    // Spin the node to keep it alive and processing callbacks
    rclcpp::spin(image_publisher_node);
    
    // Shutdown ROS2 when done
    rclcpp::shutdown();
    return 0;
}



