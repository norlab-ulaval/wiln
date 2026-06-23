#include "wiln/TrajectoryManager.hpp"
#include <fstream>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>

namespace wiln {

// .ltr format notes
// ─────────────────
// The trajectory section (after TRAJECTORY_DELIMITER) has the structure:
//
//   frame_id : <frame>
//   direction : forward|reverse          ← first segment direction (NEW)
//   x,y,z,qx,qy,qz,qw
//   ...
//   changing direction
//   direction : forward|reverse          ← each subsequent segment (NEW)
//   x,y,z,qx,qy,qz,qw
//   ...
//
// Backward compatibility: old files without "direction :" lines are loaded
// correctly — the loader falls back to alternating forward/reverse.

bool TrajectoryManager::saveLTR(const std::string& filename,
                               const norlab_controllers_msgs::msg::PathSequence& trajectory,
                               const std::string& /*map_service_name*/) {
    std::ofstream ltrFile(filename);
    if (!ltrFile.is_open()) return false;

    ltrFile << TRAJECTORY_DELIMITER << "\n";
    ltrFile << "frame_id : " << trajectory.header.frame_id << "\n";

    for (size_t i = 0; i < trajectory.paths.size(); ++i) {
        if (i > 0) {
            // Segment boundary: write direction of the incoming segment
            ltrFile << "changing direction\n";
        }
        // Persist the forward flag so reload doesn't have to guess by alternation
        ltrFile << "direction : "
                << (trajectory.paths[i].forward ? "forward" : "reverse") << "\n";

        for (const auto& pose : trajectory.paths[i].poses) {
            ltrFile << pose.pose.position.x << ","
                    << pose.pose.position.y << ","
                    << pose.pose.position.z << ","
                    << pose.pose.orientation.x << ","
                    << pose.pose.orientation.y << ","
                    << pose.pose.orientation.z << ","
                    << pose.pose.orientation.w << "\n";
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
        // ── Header section (before TRAJECTORY_DELIMITER) ─────────────────────
        if (parsingMap) {
            if (line.find(TRAJECTORY_DELIMITER) != std::string::npos) {
                // Next line is frame_id
                std::getline(ltrFile, line);
                if (line.size() > 11) {
                    trajectory.header.frame_id = line.substr(11);
                }
                parsingMap = false;

                // First segment — read direction (or use 'forward' as default)
                // We need to peek at the next line to handle both old and new formats.
                // Use a sentinel to create the first path lazily below.
            }
            continue;
        }

        // ── Trajectory section ────────────────────────────────────────────────

        // "changing direction" → start new segment (direction set by next "direction :" line)
        if (line.find("changing direction") != std::string::npos) {
            // Create a placeholder segment; direction will be filled by the
            // upcoming "direction :" line, or toggled (backward compat) if absent.
            norlab_controllers_msgs::msg::DirectionalPath dp;
            dp.forward = trajectory.paths.empty() ? true : !trajectory.paths.back().forward;
            trajectory.paths.push_back(dp);
            continue;
        }

        // "direction : forward|reverse" → sets the direction of the current (or first) segment
        if (line.rfind("direction :", 0) == 0) {
            // Trim and compare the value
            const std::string dir_val = line.substr(11);
            const bool fwd = (dir_val.find("forward") != std::string::npos);
            if (trajectory.paths.empty()) {
                norlab_controllers_msgs::msg::DirectionalPath dp;
                dp.forward = fwd;
                trajectory.paths.push_back(dp);
            } else {
                trajectory.paths.back().forward = fwd;
            }
            continue;
        }

        // Lazy creation of the first segment if no "direction :" line was found yet
        if (trajectory.paths.empty()) {
            norlab_controllers_msgs::msg::DirectionalPath dp;
            dp.forward = true;
            trajectory.paths.push_back(dp);
        }

        // CSV pose: x,y,z,qx,qy,qz,qw
        std::stringstream ss(line);
        std::string token;
        std::vector<double> vals;
        vals.reserve(7);
        while (std::getline(ss, token, ',')) {
            try { vals.push_back(std::stod(token)); }
            catch (...) { break; }
        }
        if (vals.size() == 7) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = trajectory.header.frame_id;
            pose.pose.position.x    = vals[0];
            pose.pose.position.y    = vals[1];
            pose.pose.position.z    = vals[2];
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
