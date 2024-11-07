package org.firstinspires.ftc.teamcode;


public class ToggleButton {
    private boolean previousState = false;
    private boolean value = false;
    public boolean toggle(boolean button) {
        if (button != previousState && button) {
            value = !value;
        }
        previousState = button;
        return value;
    }
    
    public void forceState(boolean state) {
        value = state;
    }
}