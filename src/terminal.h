#ifndef VISION_TERMINAL_H
#define VISION_TERMINAL_H

#include <iostream>

#define CSI (char)(0x9B)

namespace terminal {

static void initAlternateScreen() {
    std::cout << "\e[?1049h"<< std::flush;
}

static void exitAlternateScreen() {
    std::cout << "\e[?1049l"<< std::flush;
}

static void goToCorner() {
    std::cout << "\e[1;1H" << std::flush;
}

static void clear() {
    std::cout << "\e[2J" << std::flush;
    goToCorner();
}
}

#endif
