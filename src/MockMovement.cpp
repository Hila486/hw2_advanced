#include <drone_mapper/MockMovement.h>

#include <mp-units/systems/si/math.h>

namespace drone_mapper {

MockMovement::MockMovement(MockGPS& gps) : gps_(gps) {}

types::MovementResult MockMovement::rotate(types::RotationDirection direction, HorizontalAngle angle) {
    const Orientation current = gps_.heading();
    const HorizontalAngle signed_angle =
        (direction == types::RotationDirection::Left) ? angle : -angle;
    gps_.setHeading(Orientation{current.horizontal + signed_angle, current.altitude});
    return types::MovementResult{true, {}};
}

types::MovementResult MockMovement::advance(PhysicalLength distance) {
    // Move the drone forward in the horizontal plane based on its heading
    const Position3D current_pos = gps_.position();
    const Orientation heading = gps_.heading();
    
    // Calculate horizontal displacement using trigonometry
    const HorizontalAngle heading_angle = heading.horizontal;
    auto dx = si::cos(heading_angle) * distance;
    auto dy = si::sin(heading_angle) * distance;
    
    double dx_val = dx.numerical_value_is_an_implementation_detail_;
    double dy_val = dy.numerical_value_is_an_implementation_detail_;
    
    Position3D new_pos(
        current_pos.x + dx_val * cm,
        current_pos.y + dy_val * cm,
        current_pos.z
    );
    gps_.setPosition(new_pos);
    
    return types::MovementResult{true, {}};
}

types::MovementResult MockMovement::elevate(PhysicalLength distance) {
    // Move the drone vertically
    const Position3D current_pos = gps_.position();
    double z_offset = distance.numerical_value_is_an_implementation_detail_;
    Position3D new_pos(
        current_pos.x,
        current_pos.y,
        current_pos.z + z_offset * cm
    );
    gps_.setPosition(new_pos);
    
    return types::MovementResult{true, {}};
}

} // namespace drone_mapper
