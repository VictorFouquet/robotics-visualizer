#pragma once

#include "uiComponent.h"

class GUI
{
public:
    GUI() = default;
    ~GUI() = default;

    inline std::vector<UIComponent> getComponents() { return m_components; }
    inline void appendComponent(UIComponent component) { m_components.push_back(component); }
private:
    std::vector<UIComponent> m_components = {};
};
