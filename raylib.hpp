#pragma once

namespace raylib {
#include <raylib.h>
#include <rlgl.h>
#undef LIGHTGRAY
#undef GRAY
#undef DARKGRAY
#undef YELLOW
#undef GOLD
#undef ORANGE
#undef PINK
#undef RED
#undef MAROON
#undef GREEN
#undef LIME
#undef DARKGREEN
#undef SKYBLUE
#undef BLUE
#undef DARKBLUE
#undef PURPLE
#undef VIOLET
#undef DARKPURPLE
#undef BEIGE
#undef BROWN
#undef DARKBROWN
#undef WHITE
#undef BLACK
#undef BLANK
#undef MAGENTA
#undef RAYWHITE

static constexpr Color LIGHTGRAY = { 200, 200, 200, 255 }; // Light Gray
static constexpr Color GRAY = { 130, 130, 130, 255 };      // Gray
static constexpr Color DARKGRAY = { 80, 80, 80, 255 };     // Dark Gray
static constexpr Color YELLOW = { 253, 249, 0, 255 };      // Yellow
static constexpr Color GOLD = { 255, 203, 0, 255 };        // Gold
static constexpr Color ORANGE = { 255, 161, 0, 255 };      // Orange
static constexpr Color PINK = { 255, 109, 194, 255 };      // Pink
static constexpr Color RED = { 230, 41, 55, 255 };         // Red
static constexpr Color MAROON = { 190, 33, 55, 255 };      // Maroon
static constexpr Color GREEN = { 0, 228, 48, 255 };        // Green
static constexpr Color LIME = { 0, 158, 47, 255 };         // Lime
static constexpr Color DARKGREEN = { 0, 117, 44, 255 };    // Dark Green
static constexpr Color SKYBLUE = { 102, 191, 255, 255 };   // Sky Blue
static constexpr Color BLUE = { 0, 121, 241, 255 };        // Blue
static constexpr Color DARKBLUE = { 0, 82, 172, 255 };     // Dark Blue
static constexpr Color PURPLE = { 200, 122, 255, 255 };    // Purple
static constexpr Color VIOLET = { 135, 60, 190, 255 };     // Violet
static constexpr Color DARKPURPLE = { 112, 31, 126, 255 }; // Dark Purple
static constexpr Color BEIGE = { 211, 176, 131, 255 };     // Beige
static constexpr Color BROWN = { 127, 106, 79, 255 };      // Brown
static constexpr Color DARKBROWN = { 76, 63, 47, 255 };    // Dark Brown
static constexpr Color WHITE = { 255, 255, 255, 255 };     // White
static constexpr Color BLACK = { 0, 0, 0, 255 };           // Black
static constexpr Color BLANK = { 0, 0, 0, 0 };             // Blank (Transparent)
static constexpr Color MAGENTA = { 255, 0, 255, 255 };     // Magenta
static constexpr Color RAYWHITE = { 245, 245, 245, 255 };  // My own White (raylib logo)

} // namespace raylib