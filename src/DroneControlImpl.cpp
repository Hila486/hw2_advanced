#include <drone_mapper/DroneControlImpl.h>

#include <drone_mapper/ScanResultToVoxels.h>

#include <exception>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace drone_mapper {

namespace {

types::DroneStepResult makeErrorStepResult(const std::string& message) {
    return types::DroneStepResult{types::DroneStepStatus::Error, message};
}

types::MovementResult executeMovementCommand(
    IDroneMovement& movement,
    const types::MovementCommand& command) {
    switch (command.type) {
        case types::MovementCommandType::Hover:
            return types::MovementResult{true, {}};

        case types::MovementCommandType::Rotate:
            return movement.rotate(command.rotation, command.angle);

        case types::MovementCommandType::Advance:
            return movement.advance(command.distance);

        case types::MovementCommandType::Elevate:
            return movement.elevate(command.distance);
    }

    return types::MovementResult{false, "Unknown movement command type."};
}

void writeMappedVoxelsToMap(
    IMutableMap3D& output_map,
    const std::vector<types::MappedVoxel>& mapped_voxels) {
    for (const types::MappedVoxel& voxel : mapped_voxels) {
        output_map.set(voxel.position, voxel.value);
    }
}

types::LidarScanResult scanAndUpdateMap(
    ILidar& lidar,
    IMutableMap3D& output_map,
    IMappingAlgorithm& mapping_algorithm,
    const types::DroneState& current_state,
    const Orientation& scan_orientation,
    const std::string& scan_name) {
    std::cout << "DroneControl: before " << scan_name << " lidar scan" << std::endl;

    const types::LidarScanResult scan_result =
        lidar.scan(scan_orientation);

    std::cout << "DroneControl: after " << scan_name << " lidar scan, hits="
              << scan_result.size() << std::endl;

    if (!scan_result.empty()) {
        const auto& first_hit = scan_result.front();

        std::cout << "DroneControl: " << scan_name << " first hit details: hit="
                  << first_hit.hit
                  << ", distance_cm="
                  << first_hit.distance.force_numerical_value_in(cm)
                  << ", horizontal_angle_deg="
                  << first_hit.angle.horizontal.force_numerical_value_in(deg)
                  << ", altitude_angle_deg="
                  << first_hit.angle.altitude.force_numerical_value_in(deg)
                  << std::endl;
    }

    std::cout << "DroneControl: converting " << scan_name << " scan to voxels" << std::endl;

    const std::vector<types::MappedVoxel> mapped_voxels =
        ScanResultToVoxels::convert(
            current_state.position,
            current_state.heading,
            scan_result);

    std::cout << "DroneControl: " << scan_name << " mapped voxels="
              << mapped_voxels.size() << std::endl;

    std::cout << "DroneControl: writing " << scan_name
              << " mapped voxels to output map" << std::endl;

    writeMappedVoxelsToMap(output_map, mapped_voxels);

    std::cout << "DroneControl: applying " << scan_name
              << " voxel updates to algorithm" << std::endl;

    mapping_algorithm.applyVoxelUpdates(mapped_voxels);

    return scan_result;
}

} // namespace

DroneControlImpl::DroneControlImpl(types::DroneConfigData drone,
                                   types::MissionConfigData mission,
                                   ILidar& lidar,
                                   IGPS& gps,
                                   IDroneMovement& movement,
                                   IMutableMap3D& output_map,
                                   IMappingAlgorithm& mapping_algorithm)
    : drone_(std::move(drone)),
      mission_(std::move(mission)),
      lidar_(lidar),
      gps_(gps),
      movement_(movement),
      output_map_(output_map),
      mapping_algorithm_(mapping_algorithm) {}

types::DroneStepResult DroneControlImpl::step() {
    try {
        /*
         * HW1-style behavior, adapted to HW2:
         * - DroneControl owns GPS and Lidar usage.
         * - We scan forward, up, and down.
         * - All scans update the output map.
         * - The forward scan is used by the mapping algorithm for movement decision.
         */

        std::cout << "DroneControl: getting state" << std::endl;
        const types::DroneState current_state = state();

        const Orientation forward_scan{
            0.0 * horizontal_angle[deg],
            0.0 * altitude_angle[deg],
        };

        const Orientation up_scan{
            0.0 * horizontal_angle[deg],
            90.0 * altitude_angle[deg],
        };

        const Orientation down_scan{
            0.0 * horizontal_angle[deg],
            -90.0 * altitude_angle[deg],
        };

    types::LidarScanResult latest_scan =
        scanAndUpdateMap(
            lidar_,
            output_map_,
            mapping_algorithm_,
            current_state,
            forward_scan,
            "forward");

    const types::LidarScanResult up_scan_result =
        scanAndUpdateMap(
            lidar_,
            output_map_,
            mapping_algorithm_,
            current_state,
            up_scan,
            "up");

    const types::LidarScanResult down_scan_result =
        scanAndUpdateMap(
            lidar_,
            output_map_,
            mapping_algorithm_,
            current_state,
            down_scan,
            "down");

    latest_scan.insert(
        latest_scan.end(),
        up_scan_result.begin(),
        up_scan_result.end());

    latest_scan.insert(
        latest_scan.end(),
        down_scan_result.begin(),
        down_scan_result.end());

        std::cout << "DroneControl: asking algorithm for next move" << std::endl;

        const types::MovementCommand command =
            mapping_algorithm_.nextMove(current_state, latest_scan);

        std::cout << "DroneControl: executing movement command type="
                  << static_cast<int>(command.type) << std::endl;

        const types::MovementResult movement_result =
            executeMovementCommand(movement_, command);

        std::cout << "DroneControl: movement result success="
                  << movement_result.success << std::endl;

        if (!movement_result.success) {
            return makeErrorStepResult(movement_result.message);
        }

        ++step_index_;

        std::cout << "DroneControl: finished step_index="
                  << step_index_ << std::endl;

        return types::DroneStepResult{
            types::DroneStepStatus::Continue,
            {}};
    } catch (const std::exception& error) {
        std::cout << "DroneControl: exception: " << error.what() << std::endl;
        return makeErrorStepResult(error.what());
    } catch (...) {
        std::cout << "DroneControl: unknown exception" << std::endl;
        return makeErrorStepResult("Unknown error in DroneControlImpl::step.");
    }
}

types::DroneState DroneControlImpl::state() const {
    return types::DroneState{gps_.position(), gps_.heading(), step_index_};
}

} // namespace drone_mapper