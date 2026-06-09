#include <drone_mapper/ConfigParser.h>
#include <drone_mapper/ErrorLogger.h>

#include <yaml-cpp/yaml.h>
#include <sstream>
#include <stdexcept>

namespace drone_mapper {

// Helper function to validate and extract values from YAML nodes
template <typename T>
static T getYamlValue(const YAML::Node& node, const std::string& key, const std::string& context) {
    if (!node[key]) {
        std::string error = "Missing required field '" + key + "' in " + context;
        ErrorLogger::logError(error);
        throw std::runtime_error(error);
    }
    try {
        return node[key].as<T>();
    } catch (const YAML::Exception& e) {
        std::string error = "Error parsing '" + key + "' in " + context + ": " + e.what();
        ErrorLogger::logError(error);
        throw std::runtime_error(error);
    }
}

types::DroneConfigData ConfigParser::parseDroneConfig(const std::filesystem::path& filepath) {
    try {
        YAML::Node root = YAML::LoadFile(filepath.string());
        YAML::Node config = root["drone_config"];

        if (!config) {
            std::string error = "No 'drone_config' section found in " + filepath.string();
            ErrorLogger::logError(error);
            throw std::runtime_error(error);
        }

        auto dimensions = getYamlValue<double>(config, "dimensions_cm", "drone_config");
        auto max_rotate = getYamlValue<double>(config, "max_rotate_deg", "drone_config");
        auto max_advance = getYamlValue<double>(config, "max_advance_cm", "drone_config");
        auto max_elevate = getYamlValue<double>(config, "max_elevate_cm", "drone_config");

        return types::DroneConfigData(
            dimensions * cm,
            max_rotate * deg,
            max_advance * cm,
            max_elevate * cm
        );
    } catch (const YAML::Exception& e) {
        std::string error = "YAML parsing error in " + filepath.string() + ": " + e.what();
        ErrorLogger::logError(error);
        throw std::runtime_error(error);
    } catch (const std::exception& e) {
        throw;
    }
}

types::MissionConfigData ConfigParser::parseMissionConfig(const std::filesystem::path& filepath) {
    try {
        YAML::Node root = YAML::LoadFile(filepath.string());
        YAML::Node config = root["mission_config"];

        if (!config) {
            std::string error = "No 'mission_config' section found in " + filepath.string();
            ErrorLogger::logError(error);
            throw std::runtime_error(error);
        }

        auto max_steps = getYamlValue<std::size_t>(config, "max_steps", "mission_config");
        
        YAML::Node boundaries = config["boundaries"];
        auto min_x = getYamlValue<double>(boundaries["x_boundary"], "min_cm", "x_boundary");
        auto max_x = getYamlValue<double>(boundaries["x_boundary"], "max_cm", "x_boundary");
        auto min_y = getYamlValue<double>(boundaries["y_boundary"], "min_cm", "y_boundary");
        auto max_y = getYamlValue<double>(boundaries["y_boundary"], "max_cm", "y_boundary");
        auto min_height = getYamlValue<double>(boundaries["height_boundary"], "min_cm", "height_boundary");
        auto max_height = getYamlValue<double>(boundaries["height_boundary"], "max_cm", "height_boundary");

        auto gps_resolution = getYamlValue<double>(config, "gps_resolution_cm", "mission_config");
        
        // output_mapping_resolution_factor is optional, defaults to 1
        int resolution_factor = 1;
        if (config["output_mapping_resolution_factor"]) {
            resolution_factor = config["output_mapping_resolution_factor"].as<int>();
            if (resolution_factor < 1) {
                std::string warning = "output_mapping_resolution_factor < 1 in " + filepath.string() + ", using default 1";
                ErrorLogger::logWarning(warning);
                resolution_factor = 1;
            }
        }

        types::MappingBounds bounds(
            min_x * cm,
            max_x * cm,
            min_y * cm,
            max_y * cm,
            min_height * cm,
            max_height * cm
        );

        return types::MissionConfigData(max_steps, bounds, gps_resolution * cm, resolution_factor);
    } catch (const YAML::Exception& e) {
        std::string error = "YAML parsing error in " + filepath.string() + ": " + e.what();
        ErrorLogger::logError(error);
        throw std::runtime_error(error);
    } catch (const std::exception& e) {
        throw;
    }
}

types::LidarConfigData ConfigParser::parseLidarConfig(const std::filesystem::path& filepath) {
    try {
        YAML::Node root = YAML::LoadFile(filepath.string());
        YAML::Node config = root["lidar_config"];

        if (!config) {
            std::string error = "No 'lidar_config' section found in " + filepath.string();
            ErrorLogger::logError(error);
            throw std::runtime_error(error);
        }

        auto z_min = getYamlValue<double>(config, "z_min_cm", "lidar_config");
        auto z_max = getYamlValue<double>(config, "z_max_cm", "lidar_config");
        auto d = getYamlValue<double>(config, "d_cm", "lidar_config");
        auto fov_circles = getYamlValue<std::size_t>(config, "fov_circles", "lidar_config");

        return types::LidarConfigData(
            z_min * cm,
            z_max * cm,
            d * cm,
            fov_circles
        );
    } catch (const YAML::Exception& e) {
        std::string error = "YAML parsing error in " + filepath.string() + ": " + e.what();
        ErrorLogger::logError(error);
        throw std::runtime_error(error);
    } catch (const std::exception& e) {
        throw;
    }
}

types::SimulationConfigData ConfigParser::parseSimulationConfig(const std::filesystem::path& filepath) {
    try {
        YAML::Node root = YAML::LoadFile(filepath.string());
        YAML::Node config = root["simulation_config"];

        if (!config) {
            std::string error = "No 'simulation_config' section found in " + filepath.string();
            ErrorLogger::logError(error);
            throw std::runtime_error(error);
        }

        auto map_filename = getYamlValue<std::string>(config, "map_filename", "simulation_config");
        auto map_resolution = getYamlValue<double>(config, "map_resolution_cm", "simulation_config");

        YAML::Node pos = config["initial_drone_position"];
        auto x = getYamlValue<double>(pos, "x_cm", "initial_drone_position");
        auto y = getYamlValue<double>(pos, "y_cm", "initial_drone_position");
        auto height = getYamlValue<double>(pos, "height_cm", "initial_drone_position");
        
        auto angle = getYamlValue<double>(config, "initial_angle_deg", "simulation_config");

        Position3D initial_pos(x * cm, y * cm, height * cm);

        return types::SimulationConfigData(
            map_filename,
            map_resolution * cm,
            initial_pos,
            angle * deg
        );
    } catch (const YAML::Exception& e) {
        std::string error = "YAML parsing error in " + filepath.string() + ": " + e.what();
        ErrorLogger::logError(error);
        throw std::runtime_error(error);
    } catch (const std::exception& e) {
        throw;
    }
}

types::SimulationCompositionData ConfigParser::parseSimulationComposition(const std::filesystem::path& filepath) {
    try {
        YAML::Node root = YAML::LoadFile(filepath.string());
        YAML::Node compositions = root["simulation_compositions"];

        if (!compositions) {
            std::string error = "No 'simulation_compositions' section found in " + filepath.string();
            ErrorLogger::logError(error);
            throw std::runtime_error(error);
        }

        std::vector<types::SimulationConfigData> simulations;
        std::vector<types::MissionConfigData> missions;
        std::vector<types::DroneConfigData> drones;
        std::vector<types::LidarConfigData> lidars;

        // Parse simulation configs
        if (compositions["simulations"]) {
            for (const auto& sim_entry : compositions["simulations"]) {
                auto sim_file = sim_entry["simulation_config"].as<std::string>();
                simulations.push_back(parseSimulationConfig(sim_file));

                // Parse mission configs within this simulation entry
                if (sim_entry["mission_configs"]) {
                    for (const auto& mission_file : sim_entry["mission_configs"]) {
                        missions.push_back(parseMissionConfig(mission_file.as<std::string>()));
                    }
                }
            }
        }

        // Parse drone configs
        if (compositions["drone_configs"]) {
            for (const auto& drone_file : compositions["drone_configs"]) {
                drones.push_back(parseDroneConfig(drone_file.as<std::string>()));
            }
        }

        // Parse lidar configs
        if (compositions["lidar_configs"]) {
            for (const auto& lidar_file : compositions["lidar_configs"]) {
                lidars.push_back(parseLidarConfig(lidar_file.as<std::string>()));
            }
        }

        return types::SimulationCompositionData(filepath, simulations, missions, drones, lidars);
    } catch (const YAML::Exception& e) {
        std::string error = "YAML parsing error in " + filepath.string() + ": " + e.what();
        ErrorLogger::logError(error);
        throw std::runtime_error(error);
    } catch (const std::exception& e) {
        throw;
    }
}

} // namespace drone_mapper
