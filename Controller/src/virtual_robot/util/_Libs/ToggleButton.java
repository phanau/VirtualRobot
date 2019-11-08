package virtual_robot.util._Libs;

public class ToggleButton {

    boolean m_buttonValue;	// last sampled physical button value
    int m_toggleValue;	// current value of the toggled output
    int m_numStates;	// number of states the toggle moves through
    boolean m_transitionOnDown;
    boolean m_transitionOnUp;

    public ToggleButton(boolean button, int numStates, int initvalue)
    {
        this.m_buttonValue = button;
        this.m_toggleValue = initvalue;
        this.m_numStates = numStates;
        this.m_transitionOnDown = true;
        this.m_transitionOnUp = false;
    }

    // change the enabled transitions from the defaults (down, not up)
    public void setTransitions(boolean up, boolean down)
    {
        m_transitionOnUp = up;
        m_transitionOnDown = down;
    }

    // look at the current state of the controlling button and update the toggle if it has changed.
    // return true iff the state has changed, since that's when the caller might want to do something.
    public boolean process(boolean button)
    {
        // if button did one of the enabled transitions, cycle the state of the value to the next one
        boolean changed = m_transitionOnDown && (!m_buttonValue && button) ||
                m_transitionOnUp && (m_buttonValue && !button);
        if (changed)
            m_toggleValue = (m_toggleValue+1) % m_numStates;

        // remember the current state of the physical button
        m_buttonValue = button;

        return changed;
    }

    // return the current state of the toggle.
    public int value()
    {
        return m_toggleValue;
    }

}