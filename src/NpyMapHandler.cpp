#include <drone_mapper/NpyMapHandler.h>
#include <drone_mapper/ErrorLogger.h>
#include <TinyNPY.h>

#include <algorithm>
#include <cmath>

namespace drone_mapper {

std::vector<types::MappedVoxel> NpyMapHandler::loadMap(const std::filesystem::path& filepath,
                                                        PhysicalLength resolution) {
    try {
        // Load the .npy file
        NpyArray arr;
        const char* err = arr.LoadNPY(filepath.string());
        
        if (err != nullptr) {
            std::string error = "Failed to load map from " + filepath.string() + ": " + std::string(err);
            ErrorLogger::logError(error);
            throw std::runtime_error(error);
        }

        auto shape = arr.Shape();
        if (shape.size() != 3) {
            std::string error = "Map file " + filepath.string() + " is not 3D (expected shape with 3 dimensions)";
            ErrorLogger::logError(error);
            throw std::runtime_error(error);
        }

        size_t depth = shape[0];   // z
        size_t height = shape[1];  // y
        size_t width = shape[2];   // x

        std::vector<types::MappedVoxel> voxels;
        voxels.reserve(depth * height * width);

        // Extract occupancy values from the array
        int* data = arr.Data<int>();
        if (data == nullptr) {
            std::string error = "Failed to access data from map file " + filepath.string();
            ErrorLogger::logError(error);
            throw std::runtime_error(error);
        }
        
        double res_value = resolution.numerical_value_is_an_implementation_detail_;
        
        for (size_t z = 0; z < depth; ++z) {
            for (size_t y = 0; y < height; ++y) {
                for (size_t x = 0; x < width; ++x) {
                    size_t idx = z * (height * width) + y * width + x;
                    int raw_value = data[idx];
                    
                    types::MappedVoxel voxel;
                    voxel.position.x = static_cast<double>(x) * res_value * cm;
                    voxel.position.y = static_cast<double>(y) * res_value * cm;
                    voxel.position.z = static_cast<double>(z) * res_value * cm;
                    
                    // Map values to VoxelOccupancy enum
                    if (raw_value == -2) {
                        voxel.value = types::VoxelOccupancy::OutOfBounds;
                    } else if (raw_value == -1) {
                        voxel.value = types::VoxelOccupancy::Unmapped;
                    } else if (raw_value == 0) {
                        voxel.value = types::VoxelOccupancy::Empty;
                    } else if (raw_value == 1) {
                        voxel.value = types::VoxelOccupancy::Occupied;
                    } else {
                        // Clamp unknown values to Occupied
                        voxel.value = types::VoxelOccupancy::Occupied;
                    }
                    
                    voxels.push_back(voxel);
                }
            }
        }

        return voxels;
    } catch (const std::exception& e) {
        throw;
    }
}


void NpyMapHandler::saveMap(const std::filesystem::path& filepath,
                           const std::vector<types::MappedVoxel>& voxels,
                           PhysicalLength resolution,
                           const types::MappingBounds& bounds) {
    try {
        // Convert quantities to doubles for calculations
        double res_value = resolution.numerical_value_is_an_implementation_detail_;
        double min_x = bounds.min_x.numerical_value_is_an_implementation_detail_;
        double max_x = bounds.max_x.numerical_value_is_an_implementation_detail_;
        double min_y = bounds.min_y.numerical_value_is_an_implementation_detail_;
        double max_y = bounds.max_y.numerical_value_is_an_implementation_detail_;
        double min_height = bounds.min_height.numerical_value_is_an_implementation_detail_;
        double max_height = bounds.max_height.numerical_value_is_an_implementation_detail_;
        
        // Calculate array dimensions
        int x_size = static_cast<int>(std::round((max_x - min_x) / res_value)) + 1;
        int y_size = static_cast<int>(std::round((max_y - min_y) / res_value)) + 1;
        int z_size = static_cast<int>(std::round((max_height - min_height) / res_value)) + 1;

        // Create and initialize array with Unmapped (-1) values
        std::vector<int> map_array(x_size * y_size * z_size, 
                                    static_cast<int>(types::VoxelOccupancy::Unmapped));

        // Fill array with voxel data
        for (const auto& voxel : voxels) {
            auto [x_idx, y_idx, z_idx] = positionToIndices(voxel.position, resolution, bounds);
            
            // Check bounds
            if (x_idx >= 0 && x_idx < x_size && 
                y_idx >= 0 && y_idx < y_size && 
                z_idx >= 0 && z_idx < z_size) {
                
                size_t idx = z_idx * (y_size * x_size) + y_idx * x_size + x_idx;
                map_array[idx] = static_cast<int>(voxel.value);
            }
        }

        // Create parent directories if needed
        std::filesystem::create_directories(filepath.parent_path());

        // Save as .npy file using TinyNPY
        NpyArray::shape_t shape = {static_cast<size_t>(z_size), 
                                    static_cast<size_t>(y_size), 
                                    static_cast<size_t>(x_size)};
        const char* err = NpyArray::SaveNPY(filepath.string(), map_array, shape);
        
        if (err != nullptr) {
            std::string error = "Failed to save map to " + filepath.string() + ": " + std::string(err);
            ErrorLogger::logError(error);
            throw std::runtime_error(error);
        }
    } catch (const std::exception& e) {
        throw;
    }
}

std::tuple<int, int, int> NpyMapHandler::positionToIndices(const Position3D& position,
                                                            PhysicalLength resolution,
                                                            const types::MappingBounds& bounds) {
    double res_value = resolution.numerical_value_is_an_implementation_detail_;
    double min_x = bounds.min_x.numerical_value_is_an_implementation_detail_;
    double min_y = bounds.min_y.numerical_value_is_an_implementation_detail_;
    double min_height = bounds.min_height.numerical_value_is_an_implementation_detail_;
    
    double pos_x = position.x.numerical_value_is_an_implementation_detail_;
    double pos_y = position.y.numerical_value_is_an_implementation_detail_;
    double pos_z = position.z.numerical_value_is_an_implementation_detail_;
    
    int x_idx = static_cast<int>(std::round((pos_x - min_x) / res_value));
    int y_idx = static_cast<int>(std::round((pos_y - min_y) / res_value));
    int z_idx = static_cast<int>(std::round((pos_z - min_height) / res_value));
    
    return std::make_tuple(x_idx, y_idx, z_idx);
}

Position3D NpyMapHandler::indicesToPosition(int x_idx, int y_idx, int z_idx,
                                           PhysicalLength resolution,
                                           const types::MappingBounds& bounds) {
    double res_value = resolution.numerical_value_is_an_implementation_detail_;
    double min_x = bounds.min_x.numerical_value_is_an_implementation_detail_;
    double min_y = bounds.min_y.numerical_value_is_an_implementation_detail_;
    double min_height = bounds.min_height.numerical_value_is_an_implementation_detail_;
    
    double x_val = min_x + x_idx * res_value;
    double y_val = min_y + y_idx * res_value;
    double z_val = min_height + z_idx * res_value;
    
    Position3D pos;
    pos.x = x_val * cm;
    pos.y = y_val * cm;
    pos.z = z_val * cm;
    return pos;
}

} // namespace drone_mapper
