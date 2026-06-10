#include <drone_mapper/MockLidar.h>

#include <mp-units/systems/si/math.h>

#include <algorithm>
#include <limits>
#include <cmath>

namespace drone_mapper {

namespace {

[[nodiscard]] std::size_t beams_on_circle(std::size_t circle_index) {
    std::size_t count = 1;
    for (std::size_t i = 0; i < circle_index; ++i) {
        count *= 4;
    }
    return count;
}

[[nodiscard]] HorizontalAngle horizontal_delta(PhysicalLength offset, PhysicalLength distance) {
    return HorizontalAngle{si::atan2(offset, distance)};
}

[[nodiscard]] AltitudeAngle altitude_delta(PhysicalLength offset, PhysicalLength distance) {
    return AltitudeAngle{si::atan2(offset, distance)};
}

} // namespace

MockLidar::MockLidar(types::LidarConfigData config, const IMap3D& map, const IGPS& gps)
    : config_(config), map_(map), gps_(gps) {}

types::LidarScanResult MockLidar::scan(Orientation scan_orientation) const {
    types::LidarScanResult results;
    if (config_.fov_circles == 0) {
        return results;
    }

    const Orientation sensor_heading = gps_.heading();
    const Orientation center_beam_abs{
        scan_orientation.horizontal + sensor_heading.horizontal,
        scan_orientation.altitude + sensor_heading.altitude,
    };

    const BeamTraceResult center_trace = traceBeam(center_beam_abs);

    results.push_back(
        types::LidarHit{
            center_trace.hit,
            center_trace.distance,
            scan_orientation});

    for (std::size_t circle = 1; circle < config_.fov_circles; ++circle) {
        const std::size_t beam_count = beams_on_circle(circle);
        const PhysicalLength radius = static_cast<double>(circle) * config_.d;

        for (std::size_t i = 0; i < beam_count; ++i) {
            const auto theta = (360.0 * static_cast<double>(i) / static_cast<double>(beam_count)) * deg; // explicit static cast for proofing
            const PhysicalLength horizontal_offset = radius * si::cos(theta);
            const PhysicalLength altitude_offset = radius * si::sin(theta);

            const Orientation offset{
                horizontal_delta(horizontal_offset, config_.z_min),
                altitude_delta(altitude_offset, config_.z_min),
            };
            const Orientation relative_beam{
                scan_orientation.horizontal + offset.horizontal,
                scan_orientation.altitude + offset.altitude,
            };
            const Orientation absolute_beam{
                relative_beam.horizontal + sensor_heading.horizontal,
                relative_beam.altitude + sensor_heading.altitude,
            };
            const BeamTraceResult trace = traceBeam(absolute_beam);

            results.push_back(
                types::LidarHit{
                    trace.hit,
                    trace.distance,
                    relative_beam});
        }
    }

    return results;
}

MockLidar::BeamTraceResult MockLidar::traceBeam(const Orientation& beam_orientation) const {
    const Position3D origin = gps_.position();

    constexpr double pi = 3.14159265358979323846;

    const double horizontal_deg =
        beam_orientation.horizontal.force_numerical_value_in(deg);

    const double altitude_deg =
        beam_orientation.altitude.force_numerical_value_in(deg);

    const double horizontal_rad = horizontal_deg * pi / 180.0;
    const double altitude_rad = altitude_deg * pi / 180.0;

    const double cos_altitude = std::cos(altitude_rad);

    const double dir_x = cos_altitude * std::cos(horizontal_rad);
    const double dir_y = cos_altitude * std::sin(horizontal_rad);
    const double dir_z = std::sin(altitude_rad);

    const PhysicalLength step = 0.1 * map_.resolution();

    PhysicalLength last_clear_distance = 0.0 * cm;

    for (PhysicalLength distance = step; distance <= config_.z_max; distance += step) {
        const double distance_cm = distance.force_numerical_value_in(cm);

        const Position3D sample{
            origin.x + dir_x * distance_cm * x_extent[cm],
            origin.y + dir_y * distance_cm * y_extent[cm],
            origin.z + dir_z * distance_cm * z_extent[cm],
        };

        const types::VoxelOccupancy cell = map_.get(sample);

        if (cell == types::VoxelOccupancy::OutOfBounds) {
            return BeamTraceResult{false, last_clear_distance};
        }

        if (cell == types::VoxelOccupancy::Occupied) {
            if (distance < config_.z_min) {
                return BeamTraceResult{true, 0.0 * cm};
            }

            return BeamTraceResult{true, distance};
        }

        last_clear_distance = distance;
    }

    return BeamTraceResult{false, config_.z_max};
}

} // namespace drone_mapper
