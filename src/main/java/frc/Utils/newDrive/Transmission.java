package frc.Utils.newDrive;

import edu.wpi.first.wpilibj.SpeedController;

/**
 * Transmission.java
 * 
 * @author Ryan McGee
 * @written 7/10/2019
 * 
 * The highest level of the drive system, Transmission controls the number crunching
 * that is used in all the drive systems. This includes software gears and joystick
 * deadbands.
 * It also includes the basic motor controller methods that are not dependant on the
 * location of the motor (as in controlling all motors at once)
 * 
 */
public abstract class Transmission
{

/**
 * Creates the Transmission object.
 * 
 * @param speedControllers
 *                             All speed controllers that deal with driving the
 *                             robot.
 */
public Transmission(SpeedController... speedControllers)
{
    this.speedControllers = speedControllers;
}


// ================================ SOFTWARE GEARS ================================
/**
 * Sets every gear ratio. Make sure that the lowest gear starts at 0, and the
 * highest gear is at the max, to make sure the up-shifting and down-shifting
 * works properly.
 *
 * @param ratios
 *                   Percent multiplied by the transmission.drive functions
 */
public void setAllGearPercentages (double... ratios)
{
    this.gearRatios = ratios;
}

/**
 * Shift gears using a up-shift and down-shift button. Also makes sure that
 * holding the button will not trigger multiple shifts.
 *
 * @param upShiftButton
 *                            The button that should change to the next higher
 *                            gear
 * @param downShiftButton
 *                            The button that should change to the next lowest
 *                            gear
 */
public void shiftGears (boolean upShiftButton, boolean downShiftButton)
{
    // Makes sure that if the button is held down, it doesn't constantly
    // cycle through gears.
    if (downShiftButton && !this.downShiftButtonStatus)
        {
        downShift();
        }
    else if (upShiftButton && !this.upShiftButtonStatus)
        {
        upShift();
        }

    this.upShiftButtonStatus = upShiftButton;
    this.downShiftButtonStatus = downShiftButton;
}

/**
 * Adds one to the current gear of the robot, allowing the user to drive faster.
 */
public void upShift ()
{
    if (currentGear < gearRatios.length - 1)
        currentGear++;
}

/**
 * Removes one from the current gear of the robot, allowing the user to drive
 * slower.
 */
public void downShift ()
{
    if (currentGear > 0)
        currentGear--;
}

/**
 * @return The gear number that is active
 */
public int getCurrentGear ()
{
    return this.currentGear;
}

/**
 * @return The percentage corresponding to the current gear
 */
public double getCurrentGearRatio ()
{
    return this.gearRatios[currentGear];
}

/**
 * Sets the percent multiplied by Transmission.
 *
 * @param gear
 *                  Which gear should be changed: 0 is lowest, increasing.
 * @param value
 *                  Percent decimal form: between 0 and 1.0
 */
public void setGearPercentage (int gear, double value)
{
    if (value < 1 && value > 0 && gear < this.gearRatios.length && gear >= 0)
        {
        this.gearRatios[gear] = value;
        }
}


// ================================ DEADBAND CONTROLS ================================

/**
 * Turns on the deadband for use in teleop.
 */
public void enableJoystickDeadband ()
{
    this.currentJoystickDeadband = lastEnabledDeadband;
}

/**
 * Stores the current deadband, and set the new one to 0, 
 * effectively disabling it. Using setJoystickDeadband will
 * re-enable it.
 */
public void disableJoystickDeadband()
{
    this.lastEnabledDeadband = currentJoystickDeadband;
    this.currentJoystickDeadband = 0;
}

/**
 * Sets the minimum value the joysticks must output in order
 * for the robot to start moving.
 *
 * @param deadband
 *                     Percentage value, ranging from 0.0 to 1.0, in decimals.
 */
public void setJoystickDeadband (double deadband)
{
    this.lastEnabledDeadband = deadband;
    this.enableJoystickDeadband();
}

/**
 * Uses the formula for mapping one set of values to the other: y = mx + b
 *
 * m = 1 / (1 - deadband) b = deadband * -m x = joystick input y = motor output
 *
 * Therefore, motor output = (1 / (1 - deadband)) * joystick input + (1 - (1 /
 * (1 - deadband)))
 *
 * If this equation does not make much sense, try graphing it first as the
 * original x = y, and then the scaled output starting at the deadband, and use
 * the slope formula.
 *
 * @param input
 *                      The joystick value to be scaled, between -1.0 and 1.0
 * @return The scaled value, if between -1 and -deadband or deadband and 1, or 0
 *         if between -deadband and deadband.
 */
public double scaleJoystickForDeadband (double input)
{
    double deadbandSlope = 1.0 / (1.0 - currentJoystickDeadband);
    double constant = -this.currentJoystickDeadband * deadbandSlope;

    if (input > this.currentJoystickDeadband)
        return (deadbandSlope * input) + constant;
    else if (input < -this.currentJoystickDeadband)
        return -((-deadbandSlope * input) + constant);

    // Set to 0 if it is between the deadbands.
    return 0.0;
}

// ================================ MOTOR CONTROLLING ================================

/**
 * Sets all the motors to 0% that are affiliated with the drive train.
 */
public void stop()
{
    for (SpeedController sc : this.speedControllers)
        sc.stopMotor();
}

// ================================ VARIABLES ================================

// All the gear ratios, from smallest [0] to largest [length-1]
private double[] gearRatios = DEFAULT_GEAR_RATIOS;

// Will default to the lowest gear available
private int currentGear = 0;

// Makes sure when upshifting, it only changes once per click
private boolean upShiftButtonStatus = false;

// Makes sure when downshifting, it only changes once per click
private boolean downShiftButtonStatus = false;

// Stored value for if someone disables the deadband
private double lastEnabledDeadband = DEFAULT_JOYSTICK_DEADBAND;

// The motors will start turning only once the joystick is past this
// deadband.
private double currentJoystickDeadband = lastEnabledDeadband;

private final SpeedController[] speedControllers;

// ================================ CONSTANTS ================================
/** the default deadband applied to all joysticks used in drive methods */
public static final double DEFAULT_JOYSTICK_DEADBAND = .2;

/**Default ratio that is used if setAllGearPercentages is not called */
public static final double[] DEFAULT_GEAR_RATIOS = {.6, .8, 1.0};

}