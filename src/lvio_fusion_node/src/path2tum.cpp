#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <sstream>

// 将 ROS 时间戳 (secs 和 nsecs) 转换为 TUM 格式的字符串 (2011-09-26 13:02:25.964389445)
std::string formatTimestamp(const ros::Time& stamp) {
    // 使用secs和nsecs
    long nanoseconds = stamp.nsec;
    std::ostringstream oss;

    // 格式化为 TUM 格式 (YYYY-MM-DD HH:MM:SS.nnnnnnnnn)
    oss << stamp.sec << "." << std::setfill('0') << std::setw(9) << nanoseconds;
    return oss.str();
}

void savePathLineToTUM(const geometry_msgs::PoseStamped& pose, std::ofstream& file) {
    // 使用 pose 的时间戳 (secs 和 nsecs)
    std::string timestamp = formatTimestamp(pose.header.stamp);
    const auto& position = pose.pose.position;
    const auto& orientation = pose.pose.orientation;

    file << timestamp << " "
         << position.x << " "
         << position.y << " "
         << position.z << " "
         << orientation.x << " "
         << orientation.y << " "
         << orientation.z << " "
         << orientation.w << std::endl;

    ROS_INFO("Saved pose at timestamp: %s", timestamp.c_str());
}

// 处理路径消息并保存到文件，仅当时间戳变化时保存
void processPath(const nav_msgs::Path::ConstPtr& msg, std::ofstream& file, ros::Time& last_timestamp) {
    for (const auto& pose : msg->poses) {
        if (pose.header.stamp != last_timestamp) {
            savePathLineToTUM(pose, file);
            last_timestamp = pose.header.stamp;  // 更新最后保存的时间戳
        }
    }
}

// 关闭文件流时保存路径
void closeFile(std::ofstream& file) {
    file.close();
    ROS_INFO("File closed successfully.");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_to_tum_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // 获取输出文件路径
    std::string navsat_output_file, path_output_file;
    pnh.param<std::string>("navsat_output_file", navsat_output_file, "navsat_path.tum");
    pnh.param<std::string>("path_output_file", path_output_file, "path.tum");

    // 打开文件流
    std::ofstream navsat_file(navsat_output_file, std::ios::app);
    if (!navsat_file.is_open()) {
        ROS_ERROR("Failed to open file: %s", navsat_output_file.c_str());
        return 1;
    }

    std::ofstream path_file(path_output_file, std::ios::app);
    if (!path_file.is_open()) {
        ROS_ERROR("Failed to open file: %s", path_output_file.c_str());
        return 1;
    }

    // 定义最后一次保存的时间戳
    ros::Time last_timestamp;

    // 定义订阅者
    ros::Subscriber navsat_sub = nh.subscribe<nav_msgs::Path>("/lvio_fusion_node/navsat_path", 1,
        [&navsat_file, &last_timestamp](const nav_msgs::Path::ConstPtr& msg) {
            processPath(msg, navsat_file, last_timestamp);  // 每次接收到路径消息时保存数据，时间戳变化时保存
        });

    ros::Subscriber path_sub = nh.subscribe<nav_msgs::Path>("/lvio_fusion_node/path", 1,
        [&path_file, &last_timestamp](const nav_msgs::Path::ConstPtr& msg) {
            processPath(msg, path_file, last_timestamp);  // 每次接收到路径消息时保存数据，时间戳变化时保存
        });

    ROS_INFO("Path to TUM node is running and saving poses in real-time.");
    ros::spin();

    // 关闭文件流
    closeFile(navsat_file);
    closeFile(path_file);
    return 0;
}
