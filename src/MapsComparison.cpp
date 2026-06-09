#include <drone_mapper/MapsComparison.h>

#include <drone_mapper/Map3DImpl.h>

#include <cmath>
#include <cstddef>

namespace drone_mapper {

namespace {

template <typename Length>
[[nodiscard]] double cmValue(const Length& length) {
    return length.numerical_value_is_an_implementation_detail_;
}

[[nodiscard]] bool isSameResolutionRatio(ResolutionRatio resolution_ratio) {
    const double numerator = cmValue(resolution_ratio.numerator);
    const double denominator = cmValue(resolution_ratio.denominator);

    if (denominator <= 0.0 || numerator <= 0.0) {
        return false;
    }

    return std::abs((numerator / denominator) - 1.0) < 1e-9;
}

} // namespace

double MapsComparison::compare(
    const IMap3D& map1,
    const IMap3D& map2,
    ResolutionRatio resolution_ratio) {
    /*
     * The official API receives IMap3D.
     * We keep that API unchanged, but for this implementation we compare
     * concrete Map3DImpl maps because we need dimensions for iteration.
     */
    const auto* concrete_map1 = dynamic_cast<const Map3DImpl*>(&map1);
    const auto* concrete_map2 = dynamic_cast<const Map3DImpl*>(&map2);

    if (concrete_map1 == nullptr || concrete_map2 == nullptr) {
        return -1.0;
    }

    /*
     * Bonus support for different resolutions is optional.
     * For now, only same-resolution comparison is supported.
     */
    if (!isSameResolutionRatio(resolution_ratio)) {
        return -1.0;
    }

    if (concrete_map1->width() == 0 ||
        concrete_map1->height() == 0 ||
        concrete_map1->depth() == 0) {
        return -1.0;
    }

    if (concrete_map1->width() != concrete_map2->width() ||
        concrete_map1->height() != concrete_map2->height() ||
        concrete_map1->depth() != concrete_map2->depth()) {
        return -1.0;
    }

    std::size_t matching_voxels = 0;
    std::size_t total_voxels = 0;

    for (std::size_t z = 0; z < concrete_map1->depth(); ++z) {
        for (std::size_t y = 0; y < concrete_map1->height(); ++y) {
            for (std::size_t x = 0; x < concrete_map1->width(); ++x) {
                const types::VoxelOccupancy value1 = concrete_map1->getByIndex(x, y, z);
                const types::VoxelOccupancy value2 = concrete_map2->getByIndex(x, y, z);

                if (value1 == value2) {
                    ++matching_voxels;
                }

                ++total_voxels;
            }
        }
    }

    if (total_voxels == 0) {
        return -1.0;
    }

    return 100.0 * static_cast<double>(matching_voxels) /
           static_cast<double>(total_voxels);
}

} // namespace drone_mapper