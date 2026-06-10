#include <drone_mapper/MissionControlImpl.h>

#include <drone_mapper/MapsComparison.h>

#include <exception>
#include <iostream>
#include <string>
#include <utility>

namespace drone_mapper {

namespace {

types::MissionRunResult makeErrorResult(
    std::size_t steps,
    const std::filesystem::path& output_map_file,
    std::string code,
    std::string message) {
    return types::MissionRunResult{
        types::MissionRunStatus::Error,
        steps,
        -1.0,
        output_map_file,
        types::ErrorRef{std::move(code), std::move(message)}};
}

double saveAndCompareMaps(
    const IMap3D& hidden_map,
    IMutableMap3D& output_map,
    const std::filesystem::path& output_map_file) {
    std::cout << "MissionControl: saving output map..." << std::endl;

    output_map.save(output_map_file);

    std::cout << "MissionControl: comparing maps..." << std::endl;

    return MapsComparison::compare(
        hidden_map,
        output_map,
        ResolutionRatio{hidden_map.resolution(), output_map.resolution()});
}

} // namespace

MissionControlImpl::MissionControlImpl(
    types::MissionConfigData mission,
    types::DroneConfigData drone,
    const IMap3D& hidden_map,
    IMutableMap3D& output_map,
    IDroneControl& drone_control,
    std::filesystem::path output_map_file)
    : mission_(std::move(mission)),
      drone_(std::move(drone)),
      hidden_map_(hidden_map),
      output_map_(output_map),
      drone_control_(drone_control),
      output_map_file_(std::move(output_map_file)) {}

types::MissionRunResult MissionControlImpl::runMission() {
    /*
     * MissionControl owns the mission loop.
     * It repeatedly asks DroneControl to perform one logical drone step.
     *
     * DroneControl decides what the drone should do.
     * MissionControl only handles:
     * - max step limit
     * - completed/error status
     * - saving the output map
     * - comparing hidden map vs generated map
     */

    std::cout << "MissionControl: starting mission loop, max_steps="
              << mission_.max_steps << std::endl;

    for (std::size_t step = 0; step < mission_.max_steps; ++step) {
        std::cout << "MissionControl: step " << step
                  << " - before drone_control.step()" << std::endl;

        const types::DroneStepResult step_result = drone_control_.step();

        std::cout << "MissionControl: step " << step
                  << " - after drone_control.step(), status="
                  << static_cast<int>(step_result.status) << std::endl;

        const std::size_t executed_steps = step + 1;

        if (step_result.status == types::DroneStepStatus::Completed) {
            std::cout << "MissionControl: drone completed mission." << std::endl;

            try {
                const double score =
                    saveAndCompareMaps(hidden_map_, output_map_, output_map_file_);

                return types::MissionRunResult{
                    types::MissionRunStatus::Completed,
                    executed_steps,
                    score,
                    output_map_file_,
                    types::ErrorRef{}};
            } catch (const std::exception& error) {
                return makeErrorResult(
                    executed_steps,
                    output_map_file_,
                    "SAVE_OR_COMPARE_FAILED",
                    error.what());
            }
        }

        if (step_result.status == types::DroneStepStatus::Error) {
            std::cout << "MissionControl: drone step returned error: "
                      << step_result.message << std::endl;

            try {
                std::cout << "MissionControl: saving output map after error..." << std::endl;
                output_map_.save(output_map_file_);
            } catch (...) {
                /*
                 * Even if saving fails, the mission already failed.
                 * We still return the original drone-control error.
                 */
            }

            return makeErrorResult(
                executed_steps,
                output_map_file_,
                "DRONE_CONTROL_ERROR",
                step_result.message);
        }
    }

    /*
     * If we reached here, the drone did not finish before max_steps.
     * This is not a crash/error; it is a valid run status.
     */
    std::cout << "MissionControl: max_steps reached. Saving and scoring..." << std::endl;

    try {
        const double score =
            saveAndCompareMaps(hidden_map_, output_map_, output_map_file_);

        std::cout << "MissionControl: max_steps score=" << score << std::endl;

        return types::MissionRunResult{
            types::MissionRunStatus::MaxSteps,
            mission_.max_steps,
            score,
            output_map_file_,
            types::ErrorRef{}};
    } catch (const std::exception& error) {
        return makeErrorResult(
            mission_.max_steps,
            output_map_file_,
            "SAVE_OR_COMPARE_FAILED",
            error.what());
    }
}

} // namespace drone_mapper