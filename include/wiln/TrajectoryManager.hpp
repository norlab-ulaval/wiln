#pragma once

#include <string>
#include <vector>
#include <norlab_controllers_msgs/msg/path_sequence.hpp>

namespace wiln {

/**
 * @brief Handles reading and writing of .ltr and .vtk files.
 */
class TrajectoryManager {
public:
    static bool saveLTR(const std::string& filename, 
                       const norlab_controllers_msgs::msg::PathSequence& trajectory,
                       const std::string& map_service_name);

    static bool loadLTR(const std::string& filename,
                       norlab_controllers_msgs::msg::PathSequence& trajectory,
                       const std::string& load_map_service_name);

private:
    static constexpr const char* TRAJECTORY_DELIMITER = "#############################";
};

} // namespace wiln
