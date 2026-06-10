#pragma once

#include <drone_mapper/IMappingAlgorithm.h>

#include <cstddef>
#include <set>
#include <vector>

namespace drone_mapper {

class MappingAlgorithmImpl final : public IMappingAlgorithm {
public:
    explicit MappingAlgorithmImpl(types::MissionConfigData mission);

    [[nodiscard]] types::MovementCommand nextMove(
        const types::DroneState& state,
        const types::LidarScanResult& latest_scan) override;

    void applyVoxelUpdates(const std::vector<types::MappedVoxel>& voxels) override;

private:
    enum class VerticalDirection {
        Up,
        Down,
    };

    struct GridCell {
        int x = 0;
        int y = 0;
        int z = 0;

        [[nodiscard]] bool operator<(const GridCell& other) const {
            if (x != other.x) {
                return x < other.x;
            }

            if (y != other.y) {
                return y < other.y;
            }

            return z < other.z;
        }
    };

    [[nodiscard]] GridCell cellFromPosition(const Position3D& position) const;

    types::MissionConfigData mission_;
    std::set<GridCell> visited_cells_;
    std::size_t known_voxel_updates_ = 0;
    VerticalDirection vertical_direction_ = VerticalDirection::Up;
};

} // namespace drone_mapper