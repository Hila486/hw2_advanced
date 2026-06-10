#include <drone_mapper/DroneControlImpl.h>

#include <drone_mapper/ScanResultToVoxels.h>

#include <exception>
#include <iostream>
#include <string>
#include <utility>

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
         * One drone-control step:
         * 1. Read current state from GPS.
         * 2. Ask lidar for a scan in the current heading.
         * 3. Convert scan hits into mapped voxels.
         * 4. Write those voxels into the output map.
         * 5. Let the mapping algorithm decide the next movement command.
         * 6. Execute that command using the movement interface.
         */

        std::cout << "DroneControl: getting state" << std::endl;
        const types::DroneState current_state = state();

        std::cout << "DroneControl: before lidar scan" << std::endl;
        const types::LidarScanResult latest_scan =
            lidar_.scan(Orientation{});
        std::cout << "DroneControl: after lidar scan, hits="
                  << latest_scan.size() << std::endl;

        if (!latest_scan.empty()) {
            const auto& first_hit = latest_scan.front();

            std::cout << "DroneControl: first hit details: hit="
                    << first_hit.hit
                    << ", distance_cm="
                    << first_hit.distance.force_numerical_value_in(cm)
                    << ", horizontal_angle_deg="
                    << first_hit.angle.horizontal.force_numerical_value_in(deg)
                    << ", altitude_angle_deg="
                    << first_hit.angle.altitude.force_numerical_value_in(deg)
                    << std::endl;
        }

        std::cout << "DroneControl: converting scan to voxels" << std::endl;
        const std::vector<types::MappedVoxel> mapped_voxels =
            ScanResultToVoxels::convert(
                current_state.position,
                current_state.heading,
                latest_scan);
        std::cout << "DroneControl: mapped voxels="
                  << mapped_voxels.size() << std::endl;

        std::cout << "DroneControl: writing mapped voxels to output map" << std::endl;
        for (const types::MappedVoxel& voxel : mapped_voxels) {
            output_map_.set(voxel.position, voxel.value);
        }

        std::cout << "DroneControl: applying voxel updates to algorithm" << std::endl;
        mapping_algorithm_.applyVoxelUpdates(mapped_voxels);

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