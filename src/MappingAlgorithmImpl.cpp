#include <drone_mapper/MappingAlgorithmImpl.h>

#include <utility>

namespace drone_mapper {

MappingAlgorithmImpl::MappingAlgorithmImpl(types::MissionConfigData mission)
    : mission_(std::move(mission)) {}

types::MovementCommand MappingAlgorithmImpl::nextMove(const types::DroneState& state,
                                               const types::LidarScanResult& latest_scan) {
    types::MovementCommand cmd;
    
    // Extract numeric values from quantities
    double pos_x = state.position.x.numerical_value_is_an_implementation_detail_;
    double pos_y = state.position.y.numerical_value_is_an_implementation_detail_;
    
    double min_x = mission_.boundaries.min_x.numerical_value_is_an_implementation_detail_;
    double max_x = mission_.boundaries.max_x.numerical_value_is_an_implementation_detail_;
    double min_y = mission_.boundaries.min_y.numerical_value_is_an_implementation_detail_;
    double max_y = mission_.boundaries.max_y.numerical_value_is_an_implementation_detail_;
    
    // Check if we're near boundaries and need to turn or move away
    const double boundary_margin = 50.0;  // 50 cm safety margin
    
    // Check if at boundary and need to turn
    if ((pos_x <= min_x + boundary_margin && state.heading.horizontal.numerical_value_is_an_implementation_detail_ < 90.0) ||
        (pos_x >= max_x - boundary_margin && state.heading.horizontal.numerical_value_is_an_implementation_detail_ > 270.0) ||
        (pos_y <= min_y + boundary_margin && state.heading.horizontal.numerical_value_is_an_implementation_detail_ > 180.0) ||
        (pos_y >= max_y - boundary_margin && state.heading.horizontal.numerical_value_is_an_implementation_detail_ < 90.0)) {
        
        // Rotate 45 degrees to change direction
        cmd.type = types::MovementCommandType::Rotate;
        cmd.angle = 45.0 * deg;
        cmd.rotation = types::RotationDirection::Right;
        return cmd;
    }
    
    // Check if we can move forward based on lidar data
    if (!latest_scan.empty()) {
        // Count obstacles detected
        int obstacles = 0;
        for (const auto& hit : latest_scan) {
            if (hit.hit) {
                obstacles++;
            }
        }
        
        // If too many obstacles ahead, turn instead
        if (static_cast<double>(obstacles) / latest_scan.size() > 0.3) {
            cmd.type = types::MovementCommandType::Rotate;
            cmd.angle = 30.0 * deg;
            cmd.rotation = types::RotationDirection::Right;
            return cmd;
        }
    }
    
    // Try to advance forward
    cmd.type = types::MovementCommandType::Advance;
    cmd.distance = 50.0 * cm;  // Advance 50 cm at a time
    return cmd;
}

void MappingAlgorithmImpl::applyVoxelUpdates(const std::vector<types::MappedVoxel>& voxels) {
    // In a more sophisticated implementation, we would:
    // - Track which areas have been explored
    // - Update exploration priorities based on found voxels
    // - Adjust strategy based on map density
    
    // For now, we just accept the updates without processing
    (void)voxels;
}

} // namespace drone_mapper
