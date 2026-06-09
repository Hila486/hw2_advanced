#pragma once

#include <drone_mapper/types/DroneTypes.h>
#include <drone_mapper/types/LidarTypes.h>
#include <drone_mapper/types/MissionTypes.h>
#include <drone_mapper/types/SimulationTypes.h>

#include <filesystem>
#include <string>

namespace drone_mapper {

/**
 * Parses YAML configuration files for the drone mapper simulation.
 * All errors are logged to a file immediately upon occurrence.
 */
class ConfigParser {
public:
    /**
     * Parse a drone configuration YAML file.
     * @param filepath Path to drone_config.yaml
     * @return DroneConfigData with parsed values
     * @throws std::runtime_error on parsing errors
     */
    static types::DroneConfigData parseDroneConfig(const std::filesystem::path& filepath);

    /**
     * Parse a mission configuration YAML file.
     * @param filepath Path to mission_config.yaml
     * @return MissionConfigData with parsed values
     * @throws std::runtime_error on parsing errors
     */
    static types::MissionConfigData parseMissionConfig(const std::filesystem::path& filepath);

    /**
     * Parse a LiDAR configuration YAML file.
     * @param filepath Path to lidar_config.yaml
     * @return LidarConfigData with parsed values
     * @throws std::runtime_error on parsing errors
     */
    static types::LidarConfigData parseLidarConfig(const std::filesystem::path& filepath);

    /**
     * Parse a simulation configuration YAML file.
     * @param filepath Path to simulation_config.yaml
     * @return SimulationConfigData with parsed values
     * @throws std::runtime_error on parsing errors
     */
    static types::SimulationConfigData parseSimulationConfig(const std::filesystem::path& filepath);

    /**
     * Parse a simulation composition YAML file.
     * Creates the cartesian product of all configurations.
     * @param filepath Path to simulation_compositions.yaml
     * @return SimulationCompositionData with all configurations
     * @throws std::runtime_error on parsing errors
     */
    static types::SimulationCompositionData parseSimulationComposition(const std::filesystem::path& filepath);
};

} // namespace drone_mapper
