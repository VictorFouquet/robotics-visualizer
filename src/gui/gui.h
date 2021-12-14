#pragma once

#include <string>

#include "uiComponent.h"


class GUI
{
public:
    GUI() = default;
    ~GUI() = default;

    void init(int width, int height);
    
    void onEvent(AppEvent e);

    inline UIComponent getMainContainer() { return m_mainContainer; }
    // inline std::vector<UIComponent> getComponents() { return m_components; }    
    inline std::vector<UIComponent> getComponents() { return m_mainContainer.getChildren(); }

    // inline void appendComponent(UIComponent component) { m_mainContainer.appendChild(component); }
    inline void appendComponent(UIComponent component) { m_mainContainer.appendChild(component); }

    inline std::vector<UIComponent*> getButtons(); // { return m_buttons; }
    std::vector<UIComponent> getButtons(UIComponent rootComponent, std::vector<UIComponent> buttons = {});

    UIComponent createButton(std::string text, int x, int y, int w, int h);
    UIComponent createContainer(int x, int y, int w, int h);
private:
    std::vector<UIComponent> m_components = {};
    UIComponent m_mainContainer;
    std::vector<UIComponent> m_buttons = {};
    std::vector<UIComponent> m_containers = {};
};
