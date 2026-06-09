#pragma once

#include <drone_mapper/IMutableMap3D.h>

#include <cstddef>
#include <filesystem>
#include <vector>

namespace drone_mapper {

class Map3DImpl final : public IMutableMap3D {
public:
    Map3DImpl(const std::filesystem::path& path, PhysicalLength resolution);
    Map3DImpl(const types::MappingBounds& bounds, PhysicalLength resolution);

    [[nodiscard]] types::VoxelOccupancy get(const Position3D& pos) const override;
    [[nodiscard]] PhysicalLength resolution() const override;

    [[nodiscard]] std::size_t width() const;
    [[nodiscard]] std::size_t height() const;
    [[nodiscard]] std::size_t depth() const;

    [[nodiscard]] types::VoxelOccupancy getByIndex(
        std::size_t x_index,
        std::size_t y_index,
        std::size_t z_index) const;

    void set(const Position3D& pos, types::VoxelOccupancy value) override;
    void save(const std::filesystem::path& path) const override;

private:
    [[nodiscard]] bool positionToIndices(
        const Position3D& pos,
        std::size_t& x_index,
        std::size_t& y_index,
        std::size_t& z_index) const;

    [[nodiscard]] std::size_t flatIndex(
        std::size_t x_index,
        std::size_t y_index,
        std::size_t z_index) const;

    std::vector<int> values_;

    std::size_t width_ = 0;   // X dimension
    std::size_t height_ = 0;  // Y dimension
    std::size_t depth_ = 0;   // Z / height dimension

    double origin_x_cm_ = 0.0;
    double origin_y_cm_ = 0.0;
    double origin_z_cm_ = 0.0;

    PhysicalLength resolution_;
};

} // namespace drone_mapper