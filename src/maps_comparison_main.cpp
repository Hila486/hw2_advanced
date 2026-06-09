#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MapsComparison.h>
#include <drone_mapper/Units.h>

#include <filesystem>
#include <iostream>
#include <string>

namespace {

[[nodiscard]] bool parsePositiveDouble(
    const std::string& text,
    double& value) {
    try {
        std::size_t parsed_chars = 0;
        value = std::stod(text, &parsed_chars);

        return parsed_chars == text.size() && value > 0.0;
    } catch (...) {
        return false;
    }
}

[[nodiscard]] bool parseResolutionRatio(
    const std::string& argument,
    drone_mapper::ResolutionRatio& ratio) {
    std::string value = argument;

    const std::string prefix = "resolution_ratio=";
    if (value.rfind(prefix, 0) == 0) {
        value = value.substr(prefix.size());
    }

    const std::size_t slash_position = value.find('/');
    if (slash_position == std::string::npos) {
        return false;
    }

    const std::string numerator_text = value.substr(0, slash_position);
    const std::string denominator_text = value.substr(slash_position + 1);

    double numerator = 0.0;
    double denominator = 0.0;

    if (!parsePositiveDouble(numerator_text, numerator) ||
        !parsePositiveDouble(denominator_text, denominator)) {
        return false;
    }

    ratio = drone_mapper::ResolutionRatio{
        numerator * drone_mapper::cm,
        denominator * drone_mapper::cm};

    return true;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 3 || argc > 4) {
        std::cout << "-1\n";
        std::cerr << "Usage: maps_comparison <map1> <map2> [resolution_ratio=<res1>/<res2>]\n";
        return 1;
    }

    try {
        drone_mapper::ResolutionRatio ratio{
            1.0 * drone_mapper::cm,
            1.0 * drone_mapper::cm};

        if (argc == 4 && !parseResolutionRatio(argv[3], ratio)) {
            std::cout << "-1\n";
            std::cerr << "Invalid resolution_ratio argument. Expected format: resolution_ratio=<res1>/<res2>\n";
            return 1;
        }

        /*
         * The standalone utility receives only file names, not map resolution.
         * For same-resolution comparison, the actual value is not important,
         * because MapsComparison compares by voxel index.
         */
        const drone_mapper::PhysicalLength assumed_resolution = 1.0 * drone_mapper::cm;

        const drone_mapper::Map3DImpl map1{
            std::filesystem::path{argv[1]},
            assumed_resolution};

        const drone_mapper::Map3DImpl map2{
            std::filesystem::path{argv[2]},
            assumed_resolution};

        const double score = drone_mapper::MapsComparison::compare(map1, map2, ratio);

        std::cout << score << '\n';

        return score < 0.0 ? 1 : 0;
    } catch (const std::exception& error) {
        std::cout << "-1\n";
        std::cerr << error.what() << '\n';
        return 1;
    }
}