#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <filesystem>
#include <vector>
#include <chrono>
#include <string>

using namespace std::chrono_literals;

class PCDPublisher : public rclcpp::Node
{
public:
    PCDPublisher() : Node("pcd_publisher")
    {
        // Declare and get parameters
        this->declare_parameter<std::string>("pcd_folder", "");
        pcd_folder_ = this->get_parameter("pcd_folder").as_string();
        
        if (pcd_folder_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No PCD folder specified. Use 'pcd_folder' parameter.");
            return;
        }

        // Get folder name for topic
        std::filesystem::path folder_path(pcd_folder_);
        std::string topic_name = folder_path.filename().string();
        
        // Use default topic name if empty
        if (topic_name.empty()) {
            topic_name = "pointcloud";
            RCLCPP_WARN(this->get_logger(), "Using default topic name: %s", topic_name.c_str());
        }
        else{
            name_ = topic_name;
        }
        
        // Create publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(name_, 10);
        
        // Load PCD files
        if (!loadPCDFiles()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load PCD files from folder: %s", pcd_folder_.c_str());
            return;
        }

        // Create timer for publishing
        timer_ = this->create_wall_timer(
            100ms, // 10Hz
            std::bind(&PCDPublisher::timerCallback, this));
            
        RCLCPP_INFO(this->get_logger(), "PCD Publisher started. Publishing on topic: %s", name_.c_str());
    }

private:
    bool loadPCDFiles()
    {
        try {
            for (const auto& entry : std::filesystem::directory_iterator(pcd_folder_)) {
                if (entry.path().extension() == ".pcd") {
                    pcd_files_.push_back(entry.path().string());
                    std::cout<<"get one pcd file : " << entry.path().string();
                }
            }
            
            if (pcd_files_.empty()) {
                RCLCPP_ERROR(this->get_logger(), "No PCD files found in folder: %s", pcd_folder_.c_str());
                return false;
            }
            
            // Sort files alphabetically
            std::sort(pcd_files_.begin(), pcd_files_.end());
            
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error loading PCD files: %s", e.what());
            return false;
        }
    }

    void timerCallback()
    {
        if (pcd_files_.empty()) return;
        
        // Load current PCD file
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_files_[current_index_], *cloud) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read file: %s", pcd_files_[current_index_].c_str());
            return;
        }
        
        // Validate input cloud
        if (cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
            return;
        }

        // Create structured point cloud with proper fields
        sensor_msgs::msg::PointCloud2 msg;
        
        // Set up fields for structured point cloud
        msg.fields.resize(4);  // 四个字段

        // x
        msg.fields[0].name = "x";
        msg.fields[0].offset = 0;
        msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        msg.fields[0].count = 1;

        // y
        msg.fields[1].name = "y";
        msg.fields[1].offset = 4;
        msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        msg.fields[1].count = 1;

        // z
        msg.fields[2].name = "z";
        msg.fields[2].offset = 8;
        msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        msg.fields[2].count = 1;

        // i
        msg.fields[3].name = "intensity";
        msg.fields[3].offset = 12;
        msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
        msg.fields[3].count = 1;
        
        // Set point cloud properties
        msg.height = 1;
        msg.width = cloud->points.size();
        msg.is_bigendian = false;
        msg.point_step = 16;  // 4字段 * 4 bytes 
        msg.row_step = msg.point_step * msg.width;
        msg.is_dense = true;
        msg.data.resize(msg.row_step);
        
        // Allocate and fill data
        msg.data.resize(msg.row_step);
        float* data_ptr = reinterpret_cast<float*>(&msg.data[0]);
        for (size_t i = 0; i < cloud->points.size(); ++i) {
            data_ptr[4 * i + 0] = cloud->points[i].x;
            data_ptr[4 * i + 1] = cloud->points[i].y;
            data_ptr[4 * i + 2] = cloud->points[i].z;
            data_ptr[4 * i + 3] = cloud->points[i].intensity;
        }
        
        // Set header info
        msg.header.stamp = this->now();
        msg.header.frame_id = name_;
        
        // Publish
        publisher_->publish(msg);
        
        // Move to next file
        current_index_ = (current_index_ + 1) % pcd_files_.size();
    }

    std::string name_ = "pointcloud" ;
    std::string pcd_folder_;
    std::vector<std::string> pcd_files_;
    size_t current_index_ = 0;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PCDPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}