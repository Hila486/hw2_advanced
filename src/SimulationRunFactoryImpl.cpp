#include <drone_mapper/SimulationRunFactoryImpl.h>

#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/MissionControlImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/SimulationRunImpl.h>

#include <cstddef>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

namespace drone_mapper {

namespace {

std::size_t nextRunId() {
    static std::size_t next_id = 0;
    const std::size_t current_id = next_id;
    ++next_id;
    return current_id;
}

std::string makeRunFolderName(const std::filesystem::path& map_filename) {
    const std::size_t run_id = nextRunId();

    std::ostringstream stream;
    stream << "run_" << std::setw(4) << std::setfill('0') << run_id;

    const std::string map_stem = map_filename.stem().string();
    if (!map_stem.empty()) {
        stream << "_" << map_stem;
    }

    return stream.str();
}

std::filesystem::path makeOutputMapFile(
    const types::SimulationConfigData& simulation,
    const std::filesystem::path& output_path) {
    /*
     * Each simulation run gets its own folder.
     * This prevents different runs from overwriting each other's output map.
     *
     * Example:
     * output_results/run_0000_office_floor1/output_map.npy
     */
    const std::filesystem::path results_root = output_path / "output_results";
    const std::filesystem::path run_folder =
        results_root / makeRunFolderName(simulation.map_filename);

    std::filesystem::create_directories(run_folder);

    return run_folder / "output_map.npy";
}

} // namespace

std::unique_ptr<ISimulationRun> SimulationRunFactoryImpl::create(
    const types::SimulationConfigData& simulation,
    const types::MissionConfigData& mission,
    const types::DroneConfigData& drone,
    const types::LidarConfigData& lidar,
    const std::filesystem::path& output_path) {
    auto hidden_map = std::make_unique<Map3DImpl>(
        simulation.map_filename,
        simulation.map_resolution);

    auto output_map = std::make_unique<Map3DImpl>(
        mission.boundaries,
        mission.gps_resolution);

    auto gps = std::make_unique<MockGPS>(
        simulation.initial_drone_position,
        Orientation{simulation.initial_angle, 0.0 * altitude_angle[deg]});

    auto movement = std::make_unique<MockMovement>(*gps);

    auto lidar_impl = std::make_unique<MockLidar>(
        lidar,
        *hidden_map,
        *gps);

    auto mapping_algorithm = std::make_unique<MappingAlgorithmImpl>(
        mission);

    auto drone_control = std::make_unique<DroneControlImpl>(
        drone,
        mission,
        *lidar_impl,
        *gps,
        *movement,
        *output_map,
        *mapping_algorithm);

    const std::filesystem::path output_map_file =
        makeOutputMapFile(simulation, output_path);

    auto mission_control = std::make_unique<MissionControlImpl>(
        mission,
        drone,
        *hidden_map,
        *output_map,
        *drone_control,
        output_map_file);

    return std::make_unique<SimulationRunImpl>(
        std::move(hidden_map),
        std::move(output_map),
        std::move(gps),
        std::move(movement),
        std::move(lidar_impl),
        std::move(mapping_algorithm),
        std::move(drone_control),
        std::move(mission_control));
}

} // namespace drone_mapper