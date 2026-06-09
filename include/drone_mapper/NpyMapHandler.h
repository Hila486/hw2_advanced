#pragma once

#include <drone_mapper/types/MapTypes.h>
#include <drone_mapper/types/MissionTypes.h>
#include <drone_mapper/Units.h>
#include <TinyNPY.h>

#include <filesystem>
#include <vector>
#include <cstdint>

namespace drone_mapper {

/**
 * Handles loading and saving 3D voxel maps in .npy format.
 * Uses tinynpy library to interface with NumPy binary files.
 */
class NpyMapHandler {
public:
    /**
     * Load a 3D voxel map from an .npy file.
     * The file is expected to be a 3D array with shape (depth, height, width)
     * representing (z, height, x) dimensions.
     * 
     * @param filepath Path to the .npy file
     * @param resolution The physical size of each voxel
     * @return Vector of MappedVoxel with positions and occupancy values
     * @throws std::runtime_error on file I/O or format errors
     */
    static std::vector<types::MappedVoxel> loadMap(const std::filesystem::path& filepath,
                                                    PhysicalLength resolution);

    /**
     * Save a 3D voxel map to an .npy file.
     * Creates a 3D array with shape (depth, height, width).
     * 
     * @param filepath Path to save the .npy file
     * @param voxels Vector of mapped voxels to save
     * @param resolution The physical size of each voxel
     * @param bounds The bounding box of the mapped region
     * @throws std::runtime_error on file I/O errors
     */
    static void saveMap(const std::filesystem::path& filepath,
                       const std::vector<types::MappedVoxel>& voxels,
                       PhysicalLength resolution,
                       const types::MappingBounds& bounds);

private:
    /**
     * Convert a 3D voxel position to indices in the .npy array.
     * @param position The 3D position in physical coordinates
     * @param resolution The voxel resolution
     * @param bounds The mapping bounds
     * @return Tuple of (x_idx, y_idx, z_idx) indices
     */
    static std::tuple<int, int, int> positionToIndices(const Position3D& position,
                                                        PhysicalLength resolution,
                                                        const types::MappingBounds& bounds);

    /**
     * Convert array indices back to a 3D position in physical coordinates.
     * @param x_idx, y_idx, z_idx Array indices
     * @param resolution The voxel resolution
     * @param bounds The mapping bounds
     * @return Position3D in physical coordinates
     */
    static Position3D indicesToPosition(int x_idx, int y_idx, int z_idx,
                                       PhysicalLength resolution,
                                       const types::MappingBounds& bounds);
};

} // namespace drone_mapper
