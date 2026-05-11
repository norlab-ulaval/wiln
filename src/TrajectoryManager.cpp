#include "wiln/TrajectoryManager.hpp"
#include <fstream>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>

namespace wiln {

bool TrajectoryManager::saveLTR(const std::string& filename,
                               const norlab_controllers_msgs::msg::PathSequence& trajectory,
                               const std::string& /*map_service_name*/) {
    std::ofstream ltrFile(filename);
    if (!ltrFile.is_open()) return false;

    // In a real implementation, we would call the mapper service to save the VTK part here
    // For now, we focus on the trajectory data
    ltrFile << TRAJECTORY_DELIMITER << std::endl;
    ltrFile << "frame_id : " << trajectory.header.frame_id << std::endl;

    for (size_t i = 0; i < trajectory.paths.size(); ++i) {
        for (const auto& pose : trajectory.paths[i].poses) {
            ltrFile << pose.pose.position.x << ","
                    << pose.pose.position.y << ","
                    << pose.pose.position.z << ","
                    << pose.pose.orientation.x << ","
                    << pose.pose.orientation.y << ","
                    << pose.pose.orientation.z << ","
                    << pose.pose.orientation.w << std::endl;
        }
        if (i != trajectory.paths.size() - 1) {
            ltrFile << "changing direction" << std::endl;
        }
    }
    return true;
}

bool TrajectoryManager::loadLTR(const std::string& filename,
                               norlab_controllers_msgs::msg::PathSequence& trajectory,
                               const std::string& /*load_map_service_name*/) {
    std::ifstream ltrFile(filename);
    if (!ltrFile.is_open()) return false;

    trajectory.paths.clear();
    std::string line;
    bool parsingMap = true;

    while (std::getline(ltrFile, line)) {
        if (parsingMap) {
            if (line.find(TRAJECTORY_DELIMITER) != std::string::npos) {
                std::getline(ltrFile, line); // frame_id
                trajectory.header.frame_id = line.substr(11);
                parsingMap = false;
            }
            continue;
        }

        if (trajectory.paths.empty()) {
            norlab_controllers_msgs::msg::DirectionalPath dp;
            dp.forward = true;
            trajectory.paths.push_back(dp);
        }

        if (line.find("changing direction") != std::string::npos) {
            norlab_controllers_msgs::msg::DirectionalPath dp;
            dp.forward = !trajectory.paths.back().forward;
            trajectory.paths.push_back(dp);
            continue;
        }

        // Parse CSV pose: x,y,z,qx,qy,qz,qw
        std::stringstream ss(line);
        std::string token;
        std::vector<double> vals;
        while (std::getline(ss, token, ',')) {
            try { vals.push_back(std::stod(token)); }
            catch (...) { break; }
        }
        if (vals.size() == 7) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = trajectory.header.frame_id;
            pose.pose.position.x = vals[0];
            pose.pose.position.y = vals[1];
            pose.pose.position.z = vals[2];
            pose.pose.orientation.x = vals[3];
            pose.pose.orientation.y = vals[4];
            pose.pose.orientation.z = vals[5];
            pose.pose.orientation.w = vals[6];
            trajectory.paths.back().poses.push_back(pose);
        }
    }
    return true;
}

} // namespace wiln
