#pragma once

namespace chromatic {
    enum struct UNIT {INCH, DEGREE, RADIAN};
    enum struct COLOR {RED, BLUE, DARK, BRIGHT, OTHER};
    enum struct SIGN {NEGATIVE = -1, ZERO = 0, POSITIVE = 1};
    enum struct DIR {EITHER = 0, CLOCKWISE = -1, CW = -1, COUNTERCLOCKWISE = 1, CCW = 1};
 }