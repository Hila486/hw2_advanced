#include <iostream>
#include <drone_mapper/ConfigParser.h>
#include <drone_mapper/ErrorLogger.h>

using namespace drone_mapper;

int main() {
    try {
        // Initialize error logger
        ErrorLogger::initialize("logs/test_output.log");
        
        std::cout << "Testing ConfigParser with sample YAML files...\n" << std::endl;

        // Test drone config parsing
        std::cout << "1. Testing DroneConfigData parsing..." << std::endl;
        auto drone_config = ConfigParser::parseDroneConfig("test_configs/drone_config.yaml");
        std::cout << "   ✓ Loaded drone config\n";
        std::cout << "     dimensions: " << drone_config.dimensions.numerical_value_is_an_implementation_detail_ << " cm\n";
        std::cout << "     max_rotate: " << drone_config.max_rotate.numerical_value_is_an_implementation_detail_ << " deg\n" << std::endl;

        // Test mission config parsing
        std::cout << "2. Testing MissionConfigData parsing..." << std::endl;
        auto mission_config = ConfigParser::parseMissionConfig("test_configs/mission_config.yaml");
        std::cout << "   ✓ Loaded mission config\n";
        std::cout << "     max_steps: " << mission_config.max_steps << "\n";
        std::cout << "     gps_resolution: " << mission_config.gps_resolution.numerical_value_is_an_implementation_detail_ << " cm\n" << std::endl;

        // Test lidar config parsing
        std::cout << "3. Testing LidarConfigData parsing..." << std::endl;
        auto lidar_config = ConfigParser::parseLidarConfig("test_configs/lidar_config.yaml");
        std::cout << "   ✓ Loaded lidar config\n";
        std::cout << "     z_min: " << lidar_config.z_min.numerical_value_is_an_implementation_detail_ << " cm\n";
        std::cout << "     fov_circles: " << lidar_config.fov_circles << "\n" << std::endl;

        // Test simulation config parsing
        std::cout << "4. Testing SimulationConfigData parsing..." << std::endl;
        auto sim_config = ConfigParser::parseSimulationConfig("test_configs/simulation_config.yaml");
        std::cout << "   ✓ Loaded simulation config\n";
        std::cout << "     map_filename: " << sim_config.map_filename << "\n";
        std::cout << "     map_resolution: " << sim_config.map_resolution.numerical_value_is_an_implementation_detail_ << " cm\n" << std::endl;

        // Test composition parsing
        std::cout << "5. Testing SimulationCompositionData parsing..." << std::endl;
        auto composition = ConfigParser::parseSimulationComposition("test_configs/simulation_compositions.yaml");
        std::cout << "   ✓ Loaded composition with:\n";
        std::cout << "     - " << composition.simulations.size() << " simulations\n";
        std::cout << "     - " << composition.missions.size() << " missions\n";
        std::cout << "     - " << composition.drones.size() << " drones\n";
        std::cout << "     - " << composition.lidars.size() << " lidars\n";
        std::cout << "     Total combinations: " << (composition.simulations.size() * 
                                                      composition.missions.size() * 
                                                      composition.drones.size() * 
                                                      composition.lidars.size()) << "\n" << std::endl;

        std::cout << "✓ All ConfigParser tests passed!" << std::endl;
        return 0;

    } catch (const std::exception& e) {
        std::cerr << "✗ Error: " << e.what() << std::endl;
        return 1;
    }
}
