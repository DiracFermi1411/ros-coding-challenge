#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

/**
 * @class ImageProcessor
 * @brief A ROS2 node for processing incoming images and performing contour detection.
 * 
 * This node subscribes to an image topic, processes the image to detect edges 
 * and contours using OpenCV, and then publishes the processed image to a new topic. 
 * It also logs the time taken for each image processing step, providing real-time 
 * performance feedback.
 */
class ImageProcessor : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for the ImageProcessor node.
     * 
     * Initializes the subscription to the input image topic and sets up the publisher 
     * for the processed image. The node listens to the specified input image topic 
     * and processes the received images in the callback function.
     */
    ImageProcessor() : Node("image_processor_node")
    {
        // Create a subscription to receive images from the "my_camera/image" topic
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "my_camera/image", 10, 
            std::bind(&ImageProcessor::image_callback, this, std::placeholders::_1));
        
        // Create a publisher to publish the processed image to the "processed_image" topic
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("processed_image", 10);

        RCLCPP_INFO(this->get_logger(), "ImageProcessor node started and ready to process images.");
    }

private:
    /**
     * @brief Callback function for processing incoming images.
     * 
     * This function is triggered each time a new image message is received. It 
     * performs the following steps:
     *   - Converts the ROS2 image message to an OpenCV image.
     *   - Converts the image to grayscale.
     *   - Applies Canny edge detection to identify edges in the image.
     *   - Finds contours in the edge-detected image.
     *   - Draws the detected contours on the original grayscale image.
     *   - Publishes the processed image as a ROS2 image message.
     *   - Logs the processing time for performance analysis.
     * 
     * @param msg The incoming ROS2 image message.
     */
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Start measuring time for performance evaluation
        auto start_time = std::chrono::high_resolution_clock::now();

        // Convert the ROS image message to an OpenCV image (cv::Mat)
        cv::Mat frame = cv_bridge::toCvCopy(msg, msg->encoding)->image;

        // Convert the image to grayscale
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // Apply Canny edge detection
        cv::Mat edges;
        cv::Canny(gray, edges, 100, 200); // Threshold values can be tuned as needed

        // Find contours in the edge-detected image
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Convert the grayscale image to a 3-channel image for visualization
        cv::Mat gray_3_channel;
        cv::cvtColor(gray, gray_3_channel, cv::COLOR_GRAY2BGR);

        // Draw the detected contours on the 3-channel grayscale image
        cv::drawContours(gray_3_channel, contours, -1, cv::Scalar(0, 255, 0), 2); // Green color for contours

        // Convert the processed OpenCV image back to a ROS2 image message
        auto output_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", gray_3_channel).toImageMsg();

        // Publish the processed image to the "processed_image" topic
        image_publisher_->publish(*output_msg);

        // End measuring time and calculate the duration
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration = end_time - start_time;
        
        // Log the inference time for this processing step
        RCLCPP_INFO(this->get_logger(), "Inference time: %.2f ms", duration.count());
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_; ///< Subscription to the input image topic
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;       ///< Publisher for the processed image
};

int main(int argc, char *argv[])
{
    // Initialize the ROS2 system
    rclcpp::init(argc, argv);
    
    // Create an instance of the ImageProcessor node and spin it
    rclcpp::spin(std::make_shared<ImageProcessor>());
    
    // Shutdown ROS2 when done
    rclcpp::shutdown();
    return 0;
}





