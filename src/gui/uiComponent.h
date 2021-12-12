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
    UIComponent() = default;
    UIComponent(int width, int height, int posX, int posY);
    ~UIComponent() = default;

public:
    inline int getWidth() { return m_width; }
    inline int getHeight() { return m_height; }
    inline std::vector<int> getPosition() { return { m_posX, m_posY }; }
    
    void render(Frame* frame);

    inline void appendChild(UIComponent component) { m_children.push_back(component); }
    
    inline void addType(int type) { m_type |= type; }
    
    inline void setInnerText(std::string text) { m_innerText = text; }
    inline void setBackgroundColor(int r, int g, int b, int a) { m_backgroundColor = { r, g, b, a }; }
    inline void setBorderColor(int r, int g, int b, int a) { m_borderColor = { r, g, b, a }; }
    inline void setCallback(std::function<void()> f) { m_callback = f; }
    
    inline int getType() { return m_type; }
    inline std::string getInnerText() { return m_innerText; }
    inline std::vector<int> getBackgroundColor(int r, int g, int b) { return m_backgroundColor; }
    inline std::vector<int> getBorderColor(int r, int g, int b) { return m_backgroundColor; }
    inline std::vector<UIComponent> getChildren() { return m_children; }


    void onEvent(AppEvent e);
    inline void callback() { m_callback(); }

protected:
    int m_width, m_height, m_posX, m_posY;
    std::string m_innerText;
    std::vector<int> m_backgroundColor{ 0, 0, 0, 0 };
    std::vector<int> m_borderColor{ 0, 0, 0, 0 };
    std::vector<UIComponent> m_children = {};
    std::function<void()> m_callback;

    int m_type = 0;
};
