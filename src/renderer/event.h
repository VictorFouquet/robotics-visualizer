#pragma once

#include <tuple>


struct AppEvent
{
    std::pair<float, float> clickCoord = { -1.f, -1.f };
    int keyCode = -1;
};
