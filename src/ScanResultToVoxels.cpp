#include <drone_mapper/ScanResultToVoxels.h>

#include <cmath>
#include <limits>
#include <vector>

namespace drone_mapper {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRaySampleStepCm = 10.0;

[[nodiscard]] double lengthCm(PhysicalLength length) {
    return length.force_numerical_value_in(cm);
}

[[nodiscard]] double horizontalDeg(HorizontalAngle angle) {
    return angle.force_numerical_value_in(deg);
}

[[nodiscard]] double altitudeDeg(AltitudeAngle angle) {
    return angle.force_numerical_value_in(deg);
}

[[nodiscard]] double degreesToRadians(double degrees) {
    return degrees * kPi / 180.0;
}

[[nodiscard]] bool isUsableDistance(double distance_cm) {
    return std::isfinite(distance_cm) && distance_cm >= 0.0;
}

[[nodiscard]] Position3D positionAlongRay(
    const Position3D& drone_position,
    const Orientation& drone_heading,
    const types::LidarHit& lidar_hit,
    double distance_cm) {
    const double absolute_horizontal_deg =
        horizontalDeg(drone_heading.horizontal) +
        horizontalDeg(lidar_hit.angle.horizontal);

    const double absolute_altitude_deg =
        altitudeDeg(drone_heading.altitude) +
        altitudeDeg(lidar_hit.angle.altitude);

    const double horizontal_rad = degreesToRadians(absolute_horizontal_deg);
    const double altitude_rad = degreesToRadians(absolute_altitude_deg);

    const double horizontal_distance_cm =
        distance_cm * std::cos(altitude_rad);

    const double dx_cm =
        horizontal_distance_cm * std::cos(horizontal_rad);

    const double dy_cm =
        horizontal_distance_cm * std::sin(horizontal_rad);

    const double dz_cm =
        distance_cm * std::sin(altitude_rad);

    return Position3D{
        drone_position.x + dx_cm * cm,
        drone_position.y + dy_cm * cm,
        drone_position.z + dz_cm * cm};
}

void addFreeCellsBeforeHit(
    std::vector<types::MappedVoxel>& mapped_voxels,
    const Position3D& drone_position,
    const Orientation& drone_heading,
    const types::LidarHit& lidar_hit,
    double hit_distance_cm) {
    for (double ray_distance_cm = 0.0;
         ray_distance_cm + 0.000001 < hit_distance_cm;
         ray_distance_cm += kRaySampleStepCm) {
        mapped_voxels.push_back(
            types::MappedVoxel{
                positionAlongRay(
                    drone_position,
                    drone_heading,
                    lidar_hit,
                    ray_distance_cm),
                types::VoxelOccupancy::Empty});
    }
}

void addOccupiedHitCell(
    std::vector<types::MappedVoxel>& mapped_voxels,
    const Position3D& drone_position,
    const Orientation& drone_heading,
    const types::LidarHit& lidar_hit,
    double hit_distance_cm) {
    mapped_voxels.push_back(
        types::MappedVoxel{
            positionAlongRay(
                drone_position,
                drone_heading,
                lidar_hit,
                hit_distance_cm),
            types::VoxelOccupancy::Occupied});
}

} // namespace

std::vector<types::MappedVoxel> ScanResultToVoxels::convert(
    const Position3D& drone_position,
    const Orientation& drone_heading,
    const types::LidarScanResult& scan_result) {
    std::vector<types::MappedVoxel> mapped_voxels;

    for (const types::LidarHit& lidar_hit : scan_result) {
        const double distance_cm = lengthCm(lidar_hit.distance);

        if (!isUsableDistance(distance_cm)) {
            continue;
        }

        if (lidar_hit.hit) {
            /*
             * Same as HW1 logic:
             * - if we hit an obstacle, mark the ray before the hit as Empty
             * - then mark the hit point as Occupied
             */
            addFreeCellsBeforeHit(
                mapped_voxels,
                drone_position,
                drone_heading,
                lidar_hit,
                distance_cm);

            addOccupiedHitCell(
                mapped_voxels,
                drone_position,
                drone_heading,
                lidar_hit,
                distance_cm);
        } else {
            /*
             * No obstacle was hit.
             * The returned distance is the clear distance, either until max range
             * or until the beam leaves the map.
             * Mark the whole visible ray as Empty.
             */
            for (double ray_distance_cm = 0.0;
                 ray_distance_cm <= distance_cm + 0.000001;
                 ray_distance_cm += kRaySampleStepCm) {
                mapped_voxels.push_back(
                    types::MappedVoxel{
                        positionAlongRay(
                            drone_position,
                            drone_heading,
                            lidar_hit,
                            ray_distance_cm),
                        types::VoxelOccupancy::Empty});
            }
        }
    }

    return mapped_voxels;
}

} // namespace drone_mapper