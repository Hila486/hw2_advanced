#include <drone_mapper/MappingAlgorithmImpl.h>

#include <algorithm>
#include <cmath>
#include <utility>

namespace drone_mapper {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kObstacleSafetyMarginCm = 10.0;
constexpr double kRotateRightAngleDeg = 90.0;

[[nodiscard]] double xCm(XLength length) {
    return length.force_numerical_value_in(cm);
}

[[nodiscard]] double yCm(YLength length) {
    return length.force_numerical_value_in(cm);
}

[[nodiscard]] double zCm(ZLength length) {
    return length.force_numerical_value_in(cm);
}

[[nodiscard]] double physicalCm(PhysicalLength length) {
    return length.force_numerical_value_in(cm);
}

[[nodiscard]] double horizontalDeg(HorizontalAngle angle) {
    return angle.force_numerical_value_in(deg);
}

[[nodiscard]] double degreesToRadians(double degrees) {
    return degrees * kPi / 180.0;
}

[[nodiscard]] double preferredAdvanceDistanceCm(const types::MissionConfigData& mission) {
    /*
     * We do not have DroneConfigData inside IMappingAlgorithm,
     * so we use the GPS/map resolution as a conservative movement step.
     */
    const double resolution_cm = physicalCm(mission.gps_resolution);

    return std::max(1.0, resolution_cm);
}

[[nodiscard]] types::MovementCommand rotateRightCommand() {
    types::MovementCommand command;
    command.type = types::MovementCommandType::Rotate;
    command.rotation = types::RotationDirection::Right;
    command.angle = kRotateRightAngleDeg * deg;
    return command;
}

[[nodiscard]] types::MovementCommand advanceCommand(double distance_cm) {
    types::MovementCommand command;
    command.type = types::MovementCommandType::Advance;
    command.distance = distance_cm * cm;
    return command;
}

[[nodiscard]] bool hasCloseObstacleAhead(
    const types::LidarScanResult& latest_scan,
    double planned_advance_cm) {
    const double obstacle_threshold_cm =
        planned_advance_cm + kObstacleSafetyMarginCm;

    for (const types::LidarHit& hit : latest_scan) {
        if (!hit.hit) {
            continue;
        }

        const double distance_cm = physicalCm(hit.distance);

        if (distance_cm > 0.0 && distance_cm <= obstacle_threshold_cm) {
            return true;
        }
    }

    return false;
}

[[nodiscard]] bool advanceWouldLeaveMissionBounds(
    const types::DroneState& state,
    const types::MissionConfigData& mission,
    double advance_distance_cm) {
    const double heading_rad =
        degreesToRadians(horizontalDeg(state.heading.horizontal));

    const double next_x =
        xCm(state.position.x) + advance_distance_cm * std::cos(heading_rad);

    const double next_y =
        yCm(state.position.y) + advance_distance_cm * std::sin(heading_rad);

    const double current_z = zCm(state.position.z);

    return next_x < xCm(mission.boundaries.min_x) ||
           next_x > xCm(mission.boundaries.max_x) ||
           next_y < yCm(mission.boundaries.min_y) ||
           next_y > yCm(mission.boundaries.max_y) ||
           current_z < zCm(mission.boundaries.min_height) ||
           current_z > zCm(mission.boundaries.max_height);
}

} // namespace

MappingAlgorithmImpl::MappingAlgorithmImpl(types::MissionConfigData mission)
    : mission_(std::move(mission)) {}

types::MovementCommand MappingAlgorithmImpl::nextMove(
    const types::DroneState& state,
    const types::LidarScanResult& latest_scan) {
    const double advance_distance_cm = preferredAdvanceDistanceCm(mission_);

    if (hasCloseObstacleAhead(latest_scan, advance_distance_cm)) {
        return rotateRightCommand();
    }

    if (advanceWouldLeaveMissionBounds(state, mission_, advance_distance_cm)) {
        return rotateRightCommand();
    }

    return advanceCommand(advance_distance_cm);
}

void MappingAlgorithmImpl::applyVoxelUpdates(
    const std::vector<types::MappedVoxel>& voxels) {
    known_voxel_updates_ += voxels.size();
}

} // namespace drone_mapper