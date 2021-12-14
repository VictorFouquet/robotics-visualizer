#include <iostream>

#include "uiComponent.h"
#include "frame.h"
#include <iostream>


UIComponent::UIComponent(int width, int height, int posX, int posY)
    : m_width(width), m_height(height), m_posX(posX), m_posY(posY)
{
    
}

void UIComponent::render(Frame* frame) 
{
    frame->addRectangle(m_width, m_height, m_posX, m_posY, 5, 15, 30);
    if (m_innerText.size() > 0)
    {
        int textWidth, textHeight;
        TTF_Font* font = TTF_OpenFont("roboto.ttf", 12);
        if(TTF_SizeText(font, m_innerText.c_str(), &textWidth, &textHeight))
        {

        } 
        else
            frame->addMessage(
                m_innerText, "roboto.ttf",
                m_posX + (m_width - textWidth) / 2, 
                m_posY + (m_height - textHeight) / 2, 12
            );

    }
    
    if (m_borderColor[3] > 0)
    {
        frame->addLine(m_posX, m_posY, m_posX + m_width, m_posY);
        frame->addLine( m_posX + m_width, m_posY, m_posX + m_width, m_posY + m_height);
        frame->addLine(m_posX + m_width, m_posY + m_height, m_posX, m_posY + m_height);
        frame->addLine(m_posX, m_posY + m_height, m_posX, m_posY);
    }
    for (auto component : m_children)
    {
        component.render(frame);
    }
}

void UIComponent::onEvent(AppEvent e) 
{
    bool cliked = (bool)(m_type & CLICKABLE);
    if 
    (
        (bool)(m_type & CLICKABLE) &&
        e.clickCoord.first > m_posX && e.clickCoord.first < m_posX + m_width &&
        e.clickCoord.second > m_posY && e.clickCoord.second < m_posY + m_height
    )
        m_clicked = true;
    else
        m_clicked = false;
}
