#pragma once

namespace chromatic {
    enum struct UNIT {INCH, DEGREE, RADIAN};
    enum struct COLOR {RED, BLUE, DARK, BRIGHT, OTHER};
    enum struct SIGN {NEGATIVE = -1, ZERO = 0, POSITIVE = 1};
    enum struct DIR {EITHER = 0, CLOCKWISE = -1, CW = -1, RIGHT = -1, COUNTERCLOCKWISE = 1, CCW = 1, LEFT = 1};
    enum struct FACE {FORWARDS = 1, FWD = 1, BACKWARDS = -1, BACK = -1};
 }
