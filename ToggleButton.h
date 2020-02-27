#pragma once


class ToggleButton
{
    public:
    bool State;
    bool LastButton;
    ToggleButton()
    {
        State = false;
        LastButton = false;
    }

    bool ButtonPress(bool button) {
        if(button == true && LastButton == false) {
            State = !State;
        }
        LastButton = button;
        return State;
    }

};
