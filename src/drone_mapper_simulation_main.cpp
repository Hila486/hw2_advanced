#include <drone_mapper/ConfigParser.h>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>

#include <exception>
#include <filesystem>
#include <iostream>
#include <memory>

namespace {

std::filesystem::path getCompositionFile(int argc, char** argv) {
    if (argc >= 2) {
        return std::filesystem::path{argv[1]};
    }

    return std::filesystem::path{"simulation.yaml"};
}

std::filesystem::path getOutputPath(int argc, char** argv) {
    if (argc >= 3) {
        return std::filesystem::path{argv[2]};
    }

    return std::filesystem::current_path();
}

} // namespace

int main(int argc, char** argv) {
    if (argc > 3) {
        std::cerr << "Usage: ./drone_mapper_simulation [<simulation.yaml>] [<output_path>]\n";
        return 1;
    }

    try {
        const std::filesystem::path composition_file = getCompositionFile(argc, argv);
        const std::filesystem::path output_path = getOutputPath(argc, argv);

        std::cout << "Composition file: " << composition_file << std::endl;
        std::cout << "Output path: " << output_path << std::endl;

        std::filesystem::create_directories(output_path);

        std::cout << "Parsing composition..." << std::endl;

        const drone_mapper::types::SimulationCompositionData composition =
            drone_mapper::ConfigParser::parseSimulationComposition(composition_file);

        std::cout << "Finished parsing composition." << std::endl;
        std::cout << "Simulations: " << composition.simulations.size() << std::endl;
        std::cout << "Missions: " << composition.missions.size() << std::endl;
        std::cout << "Drones: " << composition.drones.size() << std::endl;
        std::cout << "Lidars: " << composition.lidars.size() << std::endl;

        auto run_factory = std::make_unique<drone_mapper::SimulationRunFactoryImpl>();

        drone_mapper::SimulationManager simulation_manager{std::move(run_factory)};

        std::cout << "Starting SimulationManager..." << std::endl;

        const drone_mapper::types::SimulationReport report =
            simulation_manager.run(composition, output_path);

        std::cout << "Finished SimulationManager." << std::endl;

        std::cout << "Simulation completed with "
                  << report.simulations.size()
                  << " score group(s).\n";

        std::cout << "Results written to: "
                  << (output_path / "simulation_output.yaml")
                  << '\n';

        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Simulation failed: " << error.what() << '\n';
        return 1;
    } catch (...) {
        std::cerr << "Simulation failed: unknown error.\n";
        return 1;
    }
}