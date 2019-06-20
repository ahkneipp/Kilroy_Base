package frc.HardwareInterfaces;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * Class used to solve a common problem with buttons in teleop where
 * the button only needs to return true for one loop
 *
 * Apparently this can be done with Momentary Switch, but it was not
 * documented or intuitize to figure out without having to declare
 * other variables/ work arounds
 *
 * NOTE: Mr. Brown does not want this to be used at the moment
 * if it can be avoided because he would prefer MomentarySwitch
 *
 * @author Cole; 2019 Build season
 */
public class QuickSwitch
{

// the button this QuickSwitch uses to determine if it is on
JoystickButton button;

// the value of this QuickSwitch
boolean isOn = false;

boolean wasPreviouslyOn = false;

/**
 * Constructor for QuickSwitch
 *
 * Creates a private JoystickButton object based on the given
 * joystick and button number that is used by this QuickSwitch
 *
 * @param joystick
 *                         the joystick the button will be on
 * @param buttonNumber
 *                         the number for the new button
 */
public QuickSwitch (Joystick joystick, int buttonNumber)
{
    this.button = new JoystickButton(joystick, buttonNumber);
}

/**
 * Constructor for QuickSwitch
 *
 * Creates a QuickSwitch that uses that given button
 *
 * @param button
 */
public QuickSwitch (JoystickButton button)
{
    this.button = button;
}

/**
 * Returns true if the button was just pressed, false otherwise.
 * Will only return true the first time teleop periodic calls this
 * method, false otherwise (no matter how long the associated button
 * is held, this method will only return true once)
 *
 * Should be called every loop of Teleop periodic (in place of the
 * associated button) in order to work properly
 */
public boolean getCurrentValue ()
{
    this.update();
    return isOn;
}

/**
 * Updates the isOn value of this QuickSwitch, setting
 * it to true if and only the button has just been
 * pressed. If it is being held, the value isOn will
 * only be set to true the first time update is called
 * while the button is being held. In all othe
 * circumstances, isOn will be set to false
 */
private void update ()
{
    if (this.button.get() == true) // if the button is being held
        {
        // if the button has just been pressed down,
        // set isOn to true. Otherwise, set isOn to false
        if (this.wasPreviouslyOn == false)
            this.isOn = true;
        else
            this.isOn = false;

        // signals that the button was not just pressed anymore
        wasPreviouslyOn = true;
        }
    else // if the button is not being held
        {
        // reset isOn and wasPreviosulyOn
        this.isOn = false;
        this.wasPreviouslyOn = false;
        }
}

}
