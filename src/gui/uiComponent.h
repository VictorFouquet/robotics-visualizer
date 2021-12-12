#pragma once

#include <vector>
#include <string>
#include <functional>
#include <iostream>

#include "frame.h"
#include "rectangle.h"
#include "event.h"


class UIComponent
{
public: 
    static const int CONTAINER = 1 << 0;
    static const int BUTTON    = 1 << 1;

    static const int CLICKABLE = 0 | BUTTON;

public:
    UIComponent(int width, int height, int posX, int posY);
    ~UIComponent() = default;

public:
    inline int getWidth() { return m_width; }
    inline int getHeight() { return m_height; }
    inline std::vector<int> getPosition() { return { m_posX, m_posY }; }
    
    inline void appendChild(UIComponent component) { m_children.push_back(component); }
    
    inline void addType(int type) { m_type |= type; }
    
    inline int getType() { return m_type; }

protected:
    int m_width, m_height, m_posX, m_posY;
    std::vector<UIComponent> m_children = {};

    int m_type = 0;
};
