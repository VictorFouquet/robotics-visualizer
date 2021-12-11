#pragma once

#include <vector>


class UIComponent
{
public:
    UIComponent(int width, int height, int posX, int posY);
    ~UIComponent() = default;

public:
    inline int getWidth() { return m_width; }
    inline int getHeight() { return m_height; }
    inline std::vector<int> getPosition() { return { m_posX, m_posY }; }
    
    inline void appendChild(UIComponent component) { m_children.push_back(component); }

protected:
    int m_width, m_height, m_posX, m_posY;
    std::vector<UIComponent> m_children = {};
};
