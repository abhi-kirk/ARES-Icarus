#pragma once

namespace icarus {

// Shared layout for 3-DOF and 6-DOF plants. The 3-DOF dynamics leave the
// attitude and rate slots at zero. FUEL stays last so guidance that only
// reads position and velocity keeps working unchanged.
enum StateIndex {
    POS_X = 0,   // Horizontal position (x)
    POS_Y = 1,   // Horizontal position (y)
    POS_Z = 2,   // Vertical position / altitude (z, up is positive)

    VEL_X = 3,   // Velocity in x
    VEL_Y = 4,   // Velocity in y
    VEL_Z = 5,   // Velocity in z (negative = descending)

    ROLL  = 6,   // Euler roll  φ about body x (rad)
    PITCH = 7,   // Euler pitch θ about body y (rad)
    YAW   = 8,   // Euler yaw   ψ about body z (rad)

    WX = 9,      // Body angular rate ωx (rad/s)
    WY = 10,     // Body angular rate ωy (rad/s)
    WZ = 11,     // Body angular rate ωz (rad/s)

    FUEL = 12,   // Fuel mass

    STATE_SIZE = 13
};

}  // namespace icarus
