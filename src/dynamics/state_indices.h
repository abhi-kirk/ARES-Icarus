#pragma once

namespace icarus {

enum StateIndex {
    POS_X = 0,   // Horizontal position (x)
    POS_Y = 1,   // Horizontal position (y)
    POS_Z = 2,   // Vertical position / altitude (z, up is positive)

    VEL_X = 3,   // Velocity in x
    VEL_Y = 4,   // Velocity in y
    VEL_Z = 5,   // Velocity in z (negative = descending)

    FUEL = 6,  // Fuel mass

    STATE_SIZE = 7  // Total number of state variables
};

}  // namespace icarus