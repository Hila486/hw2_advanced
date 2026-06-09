#include <drone_mapper/Map3DImpl.h>

#include <TinyNPY.h>

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace drone_mapper {

namespace {

template <typename Length>
[[nodiscard]] double cmValue(const Length& length) {
    return length.numerical_value_is_an_implementation_detail_;
}

[[nodiscard]] int toRaw(types::VoxelOccupancy value) {
    return static_cast<int>(value);
}

[[nodiscard]] types::VoxelOccupancy fromRaw(int raw_value) {
    switch (raw_value) {
        case -2:
            return types::VoxelOccupancy::OutOfBounds;
        case -1:
            return types::VoxelOccupancy::Unmapped;
        case 0:
            return types::VoxelOccupancy::Empty;
        case 1:
            return types::VoxelOccupancy::Occupied;
        default:
            return raw_value > 0 ? types::VoxelOccupancy::Occupied
                                 : types::VoxelOccupancy::Unmapped;
    }
}

[[nodiscard]] int normalizeRawValue(int raw_value) {
    switch (raw_value) {
        case -2:
        case -1:
        case 0:
        case 1:
            return raw_value;
        default:
            return raw_value > 0 ? toRaw(types::VoxelOccupancy::Occupied)
                                 : toRaw(types::VoxelOccupancy::Unmapped);
    }
}

template <typename T>
void copyIntegerValuesFromNpy(NpyArray& loaded_map, std::vector<int>& values) {
    const T* data = loaded_map.Data<T>();

    if (data == nullptr) {
        throw std::runtime_error("Failed to access NPY map data.");
    }

    values.resize(loaded_map.NumValue());

    for (std::size_t i = 0; i < loaded_map.NumValue(); ++i) {
        values[i] = normalizeRawValue(static_cast<int>(data[i]));
    }
}

template <typename T>
void copyFloatingValuesFromNpy(NpyArray& loaded_map, std::vector<int>& values) {
    const T* data = loaded_map.Data<T>();

    if (data == nullptr) {
        throw std::runtime_error("Failed to access NPY map data.");
    }

    values.resize(loaded_map.NumValue());

    for (std::size_t i = 0; i < loaded_map.NumValue(); ++i) {
        values[i] = normalizeRawValue(static_cast<int>(std::llround(data[i])));
    }
}

/*
 * TinyNPY does not convert the file type automatically.
 * Therefore we check the .npy dtype and read it using the matching C++ type.
 * Example: '|u1' should be read as std::uint8_t, not as int.
 */
void copyValuesFromNpy(NpyArray& loaded_map, std::vector<int>& values) {
    const char type = static_cast<char>(std::abs(static_cast<int>(loaded_map.Type())));
    const std::size_t word_size = loaded_map.SizeValueBytes();

    if (type == 'i') {
        if (word_size == 1) {
            copyIntegerValuesFromNpy<std::int8_t>(loaded_map, values);
            return;
        }

        if (word_size == 2) {
            copyIntegerValuesFromNpy<std::int16_t>(loaded_map, values);
            return;
        }

        if (word_size == 4) {
            copyIntegerValuesFromNpy<std::int32_t>(loaded_map, values);
            return;
        }

        if (word_size == 8) {
            copyIntegerValuesFromNpy<std::int64_t>(loaded_map, values);
            return;
        }
    }

    if (type == 'u') {
        if (word_size == 1) {
            copyIntegerValuesFromNpy<std::uint8_t>(loaded_map, values);
            return;
        }

        if (word_size == 2) {
            copyIntegerValuesFromNpy<std::uint16_t>(loaded_map, values);
            return;
        }

        if (word_size == 4) {
            copyIntegerValuesFromNpy<std::uint32_t>(loaded_map, values);
            return;
        }

        if (word_size == 8) {
            copyIntegerValuesFromNpy<std::uint64_t>(loaded_map, values);
            return;
        }
    }

    if (type == 'b') {
        if (word_size == 1) {
            copyIntegerValuesFromNpy<std::uint8_t>(loaded_map, values);
            return;
        }
    }

    if (type == 'f') {
        if (word_size == 4) {
            copyFloatingValuesFromNpy<float>(loaded_map, values);
            return;
        }

        if (word_size == 8) {
            copyFloatingValuesFromNpy<double>(loaded_map, values);
            return;
        }
    }

    throw std::runtime_error("Unsupported NPY map data type.");
}

void validateResolution(PhysicalLength resolution) {
    if (cmValue(resolution) <= 0.0) {
        throw std::invalid_argument("Map resolution must be positive.");
    }
}

[[nodiscard]] std::size_t dimensionFromBounds(
    double min_cm,
    double max_cm,
    double resolution_cm,
    const std::string& axis_name) {
    if (max_cm < min_cm) {
        throw std::invalid_argument(
            "Invalid map bounds on axis " + axis_name + ": max is smaller than min.");
    }

    return static_cast<std::size_t>(std::llround((max_cm - min_cm) / resolution_cm)) + 1;
}

} // namespace

Map3DImpl::Map3DImpl(const std::filesystem::path& path, PhysicalLength resolution)
    : resolution_(resolution) {
    validateResolution(resolution_);

    NpyArray loaded_map;
    const char* error = loaded_map.LoadNPY(path.string().c_str());

    if (error != nullptr) {
        throw std::runtime_error(std::string("Failed to load NPY file: ") + error);
    }

    const auto shape = loaded_map.Shape();

    if (shape.size() != 3) {
        throw std::runtime_error("NPY map must be a 3D array.");
    }

    // Convention:
    // shape[0] = depth / z
    // shape[1] = height / y
    // shape[2] = width / x
    depth_ = shape[0];
    height_ = shape[1];
    width_ = shape[2];

    if (width_ == 0 || height_ == 0 || depth_ == 0) {
        throw std::runtime_error("NPY map dimensions must be positive.");
    }

    const std::size_t total_size = width_ * height_ * depth_;

    if (loaded_map.NumValue() != total_size) {
        throw std::runtime_error("NPY map size does not match its shape.");
    }

    copyValuesFromNpy(loaded_map, values_);

    // Maps loaded from file are assumed to start at coordinate 0,0,0.
    origin_x_cm_ = 0.0;
    origin_y_cm_ = 0.0;
    origin_z_cm_ = 0.0;
}

Map3DImpl::Map3DImpl(const types::MappingBounds& bounds, PhysicalLength resolution)
    : resolution_(resolution) {
    validateResolution(resolution_);

    const double resolution_cm = cmValue(resolution_);

    origin_x_cm_ = cmValue(bounds.min_x);
    origin_y_cm_ = cmValue(bounds.min_y);
    origin_z_cm_ = cmValue(bounds.min_height);

    width_ = dimensionFromBounds(
        cmValue(bounds.min_x),
        cmValue(bounds.max_x),
        resolution_cm,
        "x");

    height_ = dimensionFromBounds(
        cmValue(bounds.min_y),
        cmValue(bounds.max_y),
        resolution_cm,
        "y");

    depth_ = dimensionFromBounds(
        cmValue(bounds.min_height),
        cmValue(bounds.max_height),
        resolution_cm,
        "z");

    values_.assign(
        width_ * height_ * depth_,
        toRaw(types::VoxelOccupancy::Unmapped));
}

types::VoxelOccupancy Map3DImpl::get(const Position3D& pos) const {
    std::size_t x_index = 0;
    std::size_t y_index = 0;
    std::size_t z_index = 0;

    if (!positionToIndices(pos, x_index, y_index, z_index)) {
        return types::VoxelOccupancy::OutOfBounds;
    }

    return fromRaw(values_[flatIndex(x_index, y_index, z_index)]);
}

PhysicalLength Map3DImpl::resolution() const {
    return resolution_;
}

std::size_t Map3DImpl::width() const {
    return width_;
}

std::size_t Map3DImpl::height() const {
    return height_;
}

std::size_t Map3DImpl::depth() const {
    return depth_;
}

types::VoxelOccupancy Map3DImpl::getByIndex(
    std::size_t x_index,
    std::size_t y_index,
    std::size_t z_index) const {
    if (x_index >= width_ || y_index >= height_ || z_index >= depth_) {
        return types::VoxelOccupancy::OutOfBounds;
    }

    return fromRaw(values_[flatIndex(x_index, y_index, z_index)]);
}

void Map3DImpl::set(const Position3D& pos, types::VoxelOccupancy value) {
    std::size_t x_index = 0;
    std::size_t y_index = 0;
    std::size_t z_index = 0;

    if (!positionToIndices(pos, x_index, y_index, z_index)) {
        return;
    }

    values_[flatIndex(x_index, y_index, z_index)] = toRaw(value);
}

void Map3DImpl::save(const std::filesystem::path& path) const {
    if (values_.empty() || width_ == 0 || height_ == 0 || depth_ == 0) {
        throw std::runtime_error("Cannot save an empty map.");
    }

    const std::filesystem::path parent_path = path.parent_path();

    if (!parent_path.empty()) {
        std::filesystem::create_directories(parent_path);
    }

    NpyArray::shape_t shape = {depth_, height_, width_};

    const char* error = NpyArray::SaveNPY(path.string(), values_, shape);

    if (error != nullptr) {
        throw std::runtime_error(std::string("Failed to save NPY file: ") + error);
    }
}

bool Map3DImpl::positionToIndices(
    const Position3D& pos,
    std::size_t& x_index,
    std::size_t& y_index,
    std::size_t& z_index) const {
    if (values_.empty() || width_ == 0 || height_ == 0 || depth_ == 0) {
        return false;
    }

    const double resolution_cm = cmValue(resolution_);

    const double x_relative = (cmValue(pos.x) - origin_x_cm_) / resolution_cm;
    const double y_relative = (cmValue(pos.y) - origin_y_cm_) / resolution_cm;
    const double z_relative = (cmValue(pos.z) - origin_z_cm_) / resolution_cm;

    const long long x_candidate = std::llround(x_relative);
    const long long y_candidate = std::llround(y_relative);
    const long long z_candidate = std::llround(z_relative);

    if (x_candidate < 0 || y_candidate < 0 || z_candidate < 0) {
        return false;
    }

    if (x_candidate >= static_cast<long long>(width_) ||
        y_candidate >= static_cast<long long>(height_) ||
        z_candidate >= static_cast<long long>(depth_)) {
        return false;
    }

    x_index = static_cast<std::size_t>(x_candidate);
    y_index = static_cast<std::size_t>(y_candidate);
    z_index = static_cast<std::size_t>(z_candidate);

    return true;
}

std::size_t Map3DImpl::flatIndex(
    std::size_t x_index,
    std::size_t y_index,
    std::size_t z_index) const {
    return z_index * (height_ * width_) + y_index * width_ + x_index;
}

} // namespace drone_mapper