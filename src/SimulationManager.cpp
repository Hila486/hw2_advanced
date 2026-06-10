#include <drone_mapper/SimulationManager.h>

#include <algorithm>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace drone_mapper {

namespace {

[[nodiscard]] double lengthCm(PhysicalLength length) {
    return length.force_numerical_value_in(cm);
}

[[nodiscard]] std::string utcTimestampNow() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);

    std::tm utc_time{};
#if defined(_WIN32)
    gmtime_s(&utc_time, &now_time);
#else
    gmtime_r(&now_time, &utc_time);
#endif

    std::ostringstream stream;
    stream << std::put_time(&utc_time, "%Y-%m-%dT%H:%M:%SZ");
    return stream.str();
}

[[nodiscard]] std::string yamlEscape(const std::string& text) {
    std::string escaped;
    escaped.reserve(text.size());

    for (const char ch : text) {
        if (ch == '\\' || ch == '"') {
            escaped.push_back('\\');
        }

        escaped.push_back(ch);
    }

    return escaped;
}

[[nodiscard]] std::string quote(const std::string& text) {
    return "\"" + yamlEscape(text) + "\"";
}

[[nodiscard]] std::string missionRunStatusToString(types::MissionRunStatus status) {
    switch (status) {
        case types::MissionRunStatus::Completed:
            return "completed";
        case types::MissionRunStatus::MaxSteps:
            return "max_steps";
        case types::MissionRunStatus::Error:
            return "error";
    }

    return "error";
}

[[nodiscard]] std::string resolutionStatusToString(types::ResolutionRequestStatus status) {
    switch (status) {
        case types::ResolutionRequestStatus::Accepted:
            return "ACCEPTED";
        case types::ResolutionRequestStatus::Ignored:
            return "IGNORED";
        case types::ResolutionRequestStatus::IgnoredTooSmall:
            return "IGNORED_TOO_SMALL";
    }

    return "IGNORED";
}

[[nodiscard]] std::string paddedRunIndex(std::size_t run_index) {
    std::ostringstream stream;
    stream << std::setw(4) << std::setfill('0') << run_index;
    return stream.str();
}

[[nodiscard]] std::string mapStemForPath(const std::filesystem::path& map_filename) {
    const std::string stem = map_filename.stem().string();

    if (stem.empty()) {
        return "unknown_map";
    }

    return stem;
}

[[nodiscard]] std::filesystem::path writeRunErrorLog(
    const std::filesystem::path& output_path,
    std::size_t run_index,
    const types::SimulationConfigData& simulation,
    const std::string& message) {
    const std::filesystem::path error_dir =
        output_path /
        "output_results" /
        ("run_" + paddedRunIndex(run_index) + "_" +
         mapStemForPath(simulation.map_filename) + "_error");

    std::filesystem::create_directories(error_dir);

    const std::filesystem::path error_file = error_dir / "error.log";
    std::ofstream output{error_file};

    if (output) {
        output << "Simulation run failed\n";
        output << "run_index: " << run_index << "\n";
        output << "map_filename: " << simulation.map_filename.string() << "\n";
        output << "message: " << message << "\n";
    }

    return error_file;
}

[[nodiscard]] types::MissionRunResult makeErrorRunResult(
    const std::filesystem::path& output_path,
    std::size_t run_index,
    const types::SimulationConfigData& simulation,
    const std::string& code,
    const std::string& message) {
    types::MissionRunResult result;
    result.status = types::MissionRunStatus::Error;
    result.steps = 0;
    result.score = -1.0;
    result.output_map_file =
        writeRunErrorLog(output_path, run_index, simulation, message);
    result.error.code = code;
    result.error.message = message;

    return result;
}

struct ReportSummary {
    std::size_t total_runs = 0;
    std::size_t scored_runs = 0;
    std::size_t error_runs = 0;
    double average_score = 0.0;
    double min_score = 0.0;
    double max_score = 0.0;
};

[[nodiscard]] ReportSummary calculateSummary(const types::SimulationReport& report) {
    ReportSummary summary;
    double score_sum = 0.0;
    double min_score = std::numeric_limits<double>::max();
    double max_score = std::numeric_limits<double>::lowest();

    for (const types::MissionScoreGroup& group : report.simulations) {
        for (const types::MissionRunResult& run : group.runs) {
            ++summary.total_runs;

            if (run.status == types::MissionRunStatus::Error || run.score < 0.0) {
                ++summary.error_runs;
                continue;
            }

            ++summary.scored_runs;
            score_sum += run.score;
            min_score = std::min(min_score, run.score);
            max_score = std::max(max_score, run.score);
        }
    }

    if (summary.scored_runs > 0) {
        summary.average_score = score_sum / static_cast<double>(summary.scored_runs);
        summary.min_score = min_score;
        summary.max_score = max_score;
    }

    return summary;
}

void writeSimulationOutputYaml(
    const types::SimulationReport& report,
    const std::filesystem::path& output_path) {
    std::filesystem::create_directories(output_path);

    const std::filesystem::path report_file = output_path / "simulation_output.yaml";
    std::ofstream output{report_file};

    if (!output) {
        throw std::runtime_error("Failed to create simulation_output.yaml.");
    }

    const ReportSummary summary = calculateSummary(report);

    output << std::fixed << std::setprecision(2);

    output << "score_report:\n";
    output << "  composition_file: " << quote(report.composition_file.string()) << "\n";
    output << "  generated_at_utc: " << quote(report.generated_at_utc) << "\n";
    output << "  metric: \"output_map_accuracy\"\n";
    output << "  score_range:\n";
    output << "    min: 0\n";
    output << "    max: 100\n";
    output << "  error_score: -1\n";
    output << "\n";

    output << "  summary:\n";
    output << "    total_runs: " << summary.total_runs << "\n";
    output << "    scored_runs: " << summary.scored_runs << "\n";
    output << "    error_runs: " << summary.error_runs << "\n";
    output << "    average_score: " << summary.average_score << "\n";
    output << "    min_score: " << summary.min_score << "\n";
    output << "    max_score: " << summary.max_score << "\n";
    output << "\n";

    output << "  simulations:\n";

    std::size_t group_index = 0;
    for (const types::MissionScoreGroup& group : report.simulations) {
        output << "    - group_index: " << group_index << "\n";
        output << "      resolution_cm: " << lengthCm(group.resolution) << "\n";
        output << "      resolution_request_status: "
               << quote(resolutionStatusToString(group.resolution_request_status)) << "\n";
        output << "      mission:\n";
        output << "        max_steps: " << group.mission.max_steps << "\n";
        output << "        gps_resolution_cm: " << lengthCm(group.mission.gps_resolution) << "\n";
        output << "        output_mapping_resolution_factor: "
               << group.mission.output_mapping_resolution_factor << "\n";
        output << "      runs:\n";

        std::size_t run_index = 0;
        for (const types::MissionRunResult& run : group.runs) {
            output << "        - run_index: " << run_index << "\n";
            output << "          status: " << quote(missionRunStatusToString(run.status)) << "\n";
            output << "          steps: " << run.steps << "\n";
            output << "          score: " << run.score << "\n";

            if (run.status == types::MissionRunStatus::Error) {
                output << "          error_log_file: "
                       << quote(run.output_map_file.string()) << "\n";
            } else {
                output << "          output_map_file: "
                       << quote(run.output_map_file.string()) << "\n";
            }

            if (run.status == types::MissionRunStatus::Error || !run.error.code.empty()) {
                output << "          error_ref:\n";
                output << "            code: " << quote(run.error.code) << "\n";
                output << "            message: " << quote(run.error.message) << "\n";
            }

            ++run_index;
        }

        ++group_index;
    }
}

} // namespace

SimulationManager::SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory)
    : run_factory_(std::move(run_factory)) {
    if (!run_factory_) {
        throw std::invalid_argument("SimulationManager requires a run factory.");
    }
}

types::SimulationReport SimulationManager::run(
    const types::SimulationCompositionData& composition,
    const std::filesystem::path& output_path) {
    std::filesystem::create_directories(output_path / "output_results");

    std::cout << "SimulationManager started." << std::endl;
    std::cout << "Simulations count: " << composition.simulations.size() << std::endl;
    std::cout << "Missions count: " << composition.missions.size() << std::endl;
    std::cout << "Drones count: " << composition.drones.size() << std::endl;
    std::cout << "Lidars count: " << composition.lidars.size() << std::endl;

    std::vector<types::MissionScoreGroup> score_groups;

    std::size_t run_index = 0;

    for (const types::SimulationConfigData& simulation : composition.simulations) {
        std::cout << "Simulation map: " << simulation.map_filename << std::endl;

        for (const types::MissionConfigData& mission : composition.missions) {
            std::cout << "Mission max_steps: " << mission.max_steps << std::endl;

            std::vector<types::MissionRunResult> results;

            for (const types::DroneConfigData& drone : composition.drones) {
                for (const types::LidarConfigData& lidar : composition.lidars) {
                    std::cout << "Creating run #" << run_index << std::endl;

                    try {
                        std::unique_ptr<ISimulationRun> run =
                            run_factory_->create(
                                simulation,
                                mission,
                                drone,
                                lidar,
                                output_path);

                        std::cout << "Created run #" << run_index
                                  << ". Starting run..." << std::endl;

                        types::MissionRunResult result = run->run();

                        std::cout << "Finished run #" << run_index
                                  << ": status=" << static_cast<int>(result.status)
                                  << ", steps=" << result.steps
                                  << ", score=" << result.score
                                  << std::endl;

                        results.push_back(std::move(result));
                    } catch (const std::exception& error) {
                        std::cout << "Run #" << run_index
                                  << " failed: " << error.what() << std::endl;

                        results.push_back(
                            makeErrorRunResult(
                                output_path,
                                run_index,
                                simulation,
                                "RUN_FAILED",
                                error.what()));
                    } catch (...) {
                        std::cout << "Run #" << run_index
                                  << " failed with unknown error." << std::endl;

                        results.push_back(
                            makeErrorRunResult(
                                output_path,
                                run_index,
                                simulation,
                                "RUN_FAILED",
                                "Unknown simulation run error."));
                    }

                    ++run_index;
                }
            }

            score_groups.emplace_back(
                mission,
                simulation.map_resolution,
                types::ResolutionRequestStatus::Ignored,
                std::move(results));
        }
    }

    std::cout << "Creating simulation report..." << std::endl;

    types::SimulationReport report{
        composition.composition_file,
        utcTimestampNow(),
        std::move(score_groups)};

    std::cout << "Writing simulation_output.yaml..." << std::endl;

    writeSimulationOutputYaml(report, output_path);

    std::cout << "SimulationManager finished." << std::endl;

    return report;
}

} // namespace drone_mapper