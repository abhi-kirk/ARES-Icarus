#pragma once

namespace icarus {

// Shared layout for the 3-DOF and 6-DOF plants. The 3-DOF dynamics leave
// the attitude derivative at zero. Fuel stays last so guidance that only
// reads position and velocity keeps working. Attitude is a quaternion
// (scalar first). Samples still carry roll, pitch, and yaw, computed
// from this quaternion when the sample is written.

enum StateIndex {
    POS_X = 0,   // Horizontal position (x)
    POS_Y = 1,   // Horizontal position (y)
    POS_Z = 2,   // Vertical position / altitude (z, up is positive)

    VEL_X = 3,   // Velocity in x
    VEL_Y = 4,   // Velocity in y
    VEL_Z = 5,   // Velocity in z (negative = descending)

    QW = 6,      // Quaternion scalar
    QX = 7,      // Quaternion vector x
    QY = 8,      // Quaternion vector y
    QZ = 9,      // Quaternion vector z

    WX = 10,     // Body angular rate ωx (rad/s)
    WY = 11,     // Body angular rate ωy (rad/s)
    WZ = 12,     // Body angular rate ωz (rad/s)

    FUEL = 13,   // Fuel mass

    STATE_SIZE = 14
};

}  // namespace icarus
