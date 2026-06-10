#include <drone_mapper/MappingAlgorithmImpl.h>

#include <algorithm>
#include <cmath>
#include <utility>

namespace drone_mapper {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kObstacleSafetyMarginCm = 10.0;
constexpr double kRotateRightAngleDeg = 90.0;
constexpr std::size_t kVerticalMovePeriod = 6;

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

[[nodiscard]] double altitudeDeg(AltitudeAngle angle) {
    return angle.force_numerical_value_in(deg);
}

[[nodiscard]] double degreesToRadians(double degrees) {
    return degrees * kPi / 180.0;
}

[[nodiscard]] int toGridIndex(double value_cm, double min_cm, double resolution_cm) {
    return static_cast<int>(
        std::llround((value_cm - min_cm) / std::max(1.0, resolution_cm)));
}

[[nodiscard]] double preferredAdvanceDistanceCm(const types::MissionConfigData& mission) {
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

[[nodiscard]] types::MovementCommand elevateCommand(double distance_cm) {
    types::MovementCommand command;
    command.type = types::MovementCommandType::Elevate;
    command.distance = distance_cm * cm;
    return command;
}

[[nodiscard]] bool isForwardBeam(const types::LidarHit& hit) {
    const double horizontal = std::abs(horizontalDeg(hit.angle.horizontal));
    const double altitude = std::abs(altitudeDeg(hit.angle.altitude));

    return horizontal <= 10.0 && altitude <= 10.0;
}

[[nodiscard]] bool isUpBeam(const types::LidarHit& hit) {
    const double horizontal = std::abs(horizontalDeg(hit.angle.horizontal));
    const double altitude = altitudeDeg(hit.angle.altitude);

    return horizontal <= 10.0 && altitude >= 80.0;
}

[[nodiscard]] bool isDownBeam(const types::LidarHit& hit) {
    const double horizontal = std::abs(horizontalDeg(hit.angle.horizontal));
    const double altitude = altitudeDeg(hit.angle.altitude);

    return horizontal <= 10.0 && altitude <= -80.0;
}

[[nodiscard]] bool hasCloseObstacleAhead(
    const types::LidarScanResult& latest_scan,
    double planned_advance_cm) {
    const double obstacle_threshold_cm =
        planned_advance_cm + kObstacleSafetyMarginCm;

    for (const types::LidarHit& hit : latest_scan) {
        if (!hit.hit || !isForwardBeam(hit)) {
            continue;
        }

        const double distance_cm = physicalCm(hit.distance);

        if (distance_cm >= 0.0 && distance_cm <= obstacle_threshold_cm) {
            return true;
        }
    }

    return false;
}

[[nodiscard]] bool verticalDirectionClear(
    const types::LidarScanResult& latest_scan,
    double planned_elevate_cm,
    bool check_up) {
    const double obstacle_threshold_cm =
        planned_elevate_cm + kObstacleSafetyMarginCm;

    for (const types::LidarHit& hit : latest_scan) {
        const bool relevant_beam =
            check_up ? isUpBeam(hit) : isDownBeam(hit);

        if (!relevant_beam || !hit.hit) {
            continue;
        }

        const double distance_cm = physicalCm(hit.distance);

        if (distance_cm >= 0.0 && distance_cm <= obstacle_threshold_cm) {
            return false;
        }
    }

    return true;
}

[[nodiscard]] double nextXCm(
    const types::DroneState& state,
    double advance_distance_cm) {
    const double heading_rad =
        degreesToRadians(horizontalDeg(state.heading.horizontal));

    return xCm(state.position.x) +
           advance_distance_cm * std::cos(heading_rad);
}

[[nodiscard]] double nextYCm(
    const types::DroneState& state,
    double advance_distance_cm) {
    const double heading_rad =
        degreesToRadians(horizontalDeg(state.heading.horizontal));

    return yCm(state.position.y) +
           advance_distance_cm * std::sin(heading_rad);
}

[[nodiscard]] bool coordinatesInsideMissionBounds(
    const types::MissionConfigData& mission,
    double x_cm,
    double y_cm,
    double z_cm) {
    return x_cm >= xCm(mission.boundaries.min_x) &&
           x_cm <= xCm(mission.boundaries.max_x) &&
           y_cm >= yCm(mission.boundaries.min_y) &&
           y_cm <= yCm(mission.boundaries.max_y) &&
           z_cm >= zCm(mission.boundaries.min_height) &&
           z_cm <= zCm(mission.boundaries.max_height);
}

} // namespace

MappingAlgorithmImpl::MappingAlgorithmImpl(types::MissionConfigData mission)
    : mission_(std::move(mission)) {}

MappingAlgorithmImpl::GridCell MappingAlgorithmImpl::cellFromPosition(
    const Position3D& position) const {
    const double resolution_cm = physicalCm(mission_.gps_resolution);

    return GridCell{
        toGridIndex(
            xCm(position.x),
            xCm(mission_.boundaries.min_x),
            resolution_cm),
        toGridIndex(
            yCm(position.y),
            yCm(mission_.boundaries.min_y),
            resolution_cm),
        toGridIndex(
            zCm(position.z),
            zCm(mission_.boundaries.min_height),
            resolution_cm)};
}

types::MovementCommand MappingAlgorithmImpl::nextMove(
    const types::DroneState& state,
    const types::LidarScanResult& latest_scan) {
    const double advance_distance_cm = preferredAdvanceDistanceCm(mission_);

    const GridCell current_cell = cellFromPosition(state.position);
    visited_cells_.insert(current_cell);

    const double current_x_cm = xCm(state.position.x);
    const double current_y_cm = yCm(state.position.y);
    const double current_z_cm = zCm(state.position.z);

    const bool should_try_vertical =
        state.step_index > 0 &&
        state.step_index % kVerticalMovePeriod == 0;

    if (should_try_vertical) {
        const double vertical_step_cm = advance_distance_cm;

        if (vertical_direction_ == VerticalDirection::Up) {
            const double next_z_cm = current_z_cm + vertical_step_cm;

            const bool inside_bounds =
                coordinatesInsideMissionBounds(
                    mission_,
                    current_x_cm,
                    current_y_cm,
                    next_z_cm);

            const bool clear =
                verticalDirectionClear(
                    latest_scan,
                    vertical_step_cm,
                    true);

            if (inside_bounds && clear) {
                return elevateCommand(vertical_step_cm);
            }

            vertical_direction_ = VerticalDirection::Down;
        }

        if (vertical_direction_ == VerticalDirection::Down) {
            const double next_z_cm = current_z_cm - vertical_step_cm;

            const bool inside_bounds =
                coordinatesInsideMissionBounds(
                    mission_,
                    current_x_cm,
                    current_y_cm,
                    next_z_cm);

            const bool clear =
                verticalDirectionClear(
                    latest_scan,
                    vertical_step_cm,
                    false);

            if (inside_bounds && clear) {
                return elevateCommand(-vertical_step_cm);
            }

            vertical_direction_ = VerticalDirection::Up;
        }
    }

    const double next_x_cm = nextXCm(state, advance_distance_cm);
    const double next_y_cm = nextYCm(state, advance_distance_cm);
    const double next_z_cm = current_z_cm;

    const Position3D next_position{
        next_x_cm * x_extent[cm],
        next_y_cm * y_extent[cm],
        next_z_cm * z_extent[cm],
    };

    const GridCell next_cell = cellFromPosition(next_position);

    const bool obstacle_ahead =
        hasCloseObstacleAhead(latest_scan, advance_distance_cm);

    const bool outside_bounds =
        !coordinatesInsideMissionBounds(
            mission_,
            next_x_cm,
            next_y_cm,
            next_z_cm);

    const bool already_visited =
        visited_cells_.find(next_cell) != visited_cells_.end();

    if (!obstacle_ahead && !outside_bounds && !already_visited) {
        return advanceCommand(advance_distance_cm);
    }

    return rotateRightCommand();
}

void MappingAlgorithmImpl::applyVoxelUpdates(
    const std::vector<types::MappedVoxel>& voxels) {
    known_voxel_updates_ += voxels.size();
}

} // namespace drone_mapper