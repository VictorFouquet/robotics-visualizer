#include "gui.h"
#include <iostream>


void GUI::init(int w, int h)
{
    m_mainContainer = createContainer(0, 0, w, h);
}

void GUI::onEvent(AppEvent e) 
{
    for (UIComponent* btn : getButtons())
        btn->onEvent(e);
}

UIComponent GUI::createButton(std::string text, int x, int y, int w, int h) 
{
    UIComponent btn(w, h, x, y);
    btn.setInnerText(text);
    btn.addType(UIComponent::BUTTON);

    m_buttons.push_back(btn);

    return btn;
}

std::vector<UIComponent> GUI::getButtons(UIComponent rootComponent, std::vector<UIComponent> buttons) 
{
    for (UIComponent component : rootComponent.getChildren())
    {
        if (component.isButton())
            buttons.push_back(component);
        else if (component.isContainer())
            buttons = getButtons(component, buttons);
    }

    return buttons;
}

std::vector<UIComponent*> GUI::getButtons() 
{
    std::vector<UIComponent*> btns = {};
    for (auto component : m_buttons)
        btns.push_back(&component);
    
    return btns;
}

UIComponent GUI::createContainer(int x, int y, int w, int h) 
{
    UIComponent ctn(w, h, x, y);
    ctn.addType(UIComponent::CONTAINER);

    m_containers.push_back(ctn);

    return ctn;
}
