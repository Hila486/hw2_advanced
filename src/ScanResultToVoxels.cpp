#include <drone_mapper/ScanResultToVoxels.h>

#include <cmath>
#include <numbers>
#include <vector>

namespace drone_mapper {

namespace {

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
    return degrees * std::numbers::pi / 180.0;
}

[[nodiscard]] bool isUsableHit(const types::LidarHit& hit) {
    if (!hit.hit) {
        return false;
    }

    const double distance_cm = lengthCm(hit.distance);

    /*
     * distance == 0 means the object is closer than lidar z_min.
     * The lidar knows something is too close, but not exactly where,
     * so for now we do not create a voxel from that hit.
     */
    return std::isfinite(distance_cm) && distance_cm > 0.0;
}

[[nodiscard]] Position3D hitToPosition(
    const Position3D& scan_origin,
    const Orientation& drone_heading,
    const types::LidarHit& hit) {
    /*
     * LidarHit::angle is relative to the drone/sensor direction.
     * To place the hit in the world map, convert it to an absolute direction.
     */
    const double horizontal_rad = degreesToRadians(
        horizontalDeg(drone_heading.horizontal) +
        horizontalDeg(hit.angle.horizontal));

    const double altitude_rad = degreesToRadians(
        altitudeDeg(drone_heading.altitude) +
        altitudeDeg(hit.angle.altitude));

    const double distance_cm = lengthCm(hit.distance);

    const double horizontal_distance_cm = distance_cm * std::cos(altitude_rad);

    const double dx_cm = horizontal_distance_cm * std::cos(horizontal_rad);
    const double dy_cm = horizontal_distance_cm * std::sin(horizontal_rad);
    const double dz_cm = distance_cm * std::sin(altitude_rad);

    return Position3D{
        scan_origin.x + dx_cm * cm,
        scan_origin.y + dy_cm * cm,
        scan_origin.z + dz_cm * cm};
}

} // namespace

std::vector<types::MappedVoxel> ScanResultToVoxels::convert(
    const Position3D& scan_origin,
    const Orientation& drone_heading,
    const types::LidarScanResult& scan) {
    std::vector<types::MappedVoxel> mapped_voxels;

    mapped_voxels.reserve(scan.size());

    for (const types::LidarHit& hit : scan) {
        if (!isUsableHit(hit)) {
            continue;
        }

        mapped_voxels.push_back(
            types::MappedVoxel{
                hitToPosition(scan_origin, drone_heading, hit),
                types::VoxelOccupancy::Occupied});
    }

    return mapped_voxels;
}

} // namespace drone_mapper