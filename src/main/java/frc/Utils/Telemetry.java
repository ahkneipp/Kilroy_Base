package frc.Utils;

import frc.Hardware.Hardware;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Dedicated to printing information to both Console and Smartdashboard
 *
 * @author Craig Kimball
 * @written 6/7/2018
 *
 */

public class Telemetry
{
/**
 * constructor for
 *
 */
public Telemetry ()
{
    this.init();
}

/**
 * creates the Telemetry object. Used for the set function of the
 * timeBetweenPrints
 *
 * @param newTimeBetweenPrints
 *                                 - passing in a time between prints overriding
 *                                 the default
 */
public Telemetry (double newTimeBetweenPrints)
{
    this.setTimeBetweenPrints(newTimeBetweenPrints);
    this.init();
}

/**
 * Returns last time print statements were printed
 *
 * @return lastTimePrinted
 */
private double getLastTimePrinted ()
{
    return (lastTimePrinted);
}

/**
 * function dedicated to returning the current time between prints
 *
 * @return timeBetweenPrints
 */
public double getTimeBetweenPrints ()
{
    return (timeBetweenPrints);

}

/**
 * Initializes telemetry object by setting lastTimePrinted to the system's
 * current time
 */
private void init ()
{
    // getting the initial time
    lastTimePrinted = System.currentTimeMillis();
}

/**
 * @param newLastTimePrinted
 *                               sets lastTimePrinted to what you want
 * @return
 */
private double setLastTimePrinted (double newLastTimePrinted)
{
    lastTimePrinted = newLastTimePrinted;
    return (this.getLastTimePrinted());
}

/**
 * function dedicated to setting a new value for time between each print
 *
 * @param newTimeBetweenPrints
 *                                 - new time between each printed string
 * @return - the updated Time between prints
 */
public double setTimeBetweenPrints (double newTimeBetweenPrints)
{
    timeBetweenPrints = newTimeBetweenPrints;
    return (this.getTimeBetweenPrints());
}

/**
 * prints to console
 *
 * @param String
 *                   - stringToPrint - string that will be displayed on the
 *                   console
 *
 */
public void printToConsole (String stringToPrint)
{
    // clock determining change

    if ((System.currentTimeMillis() - lastTimePrinted) >= this
            .getTimeBetweenPrints())
        {
        if (Hardware.driverStation.isFMSAttached() == false)
            System.out.println(stringToPrint);

        // resets the clock
        this.setLastTimePrinted(System.currentTimeMillis());
        } // if
} // end printToConsole()

/**
 * prints to shuffleboard / smartboard
 *
 * @param String
 *                   - dashboardKey - the name of the shuffleboard item to place
 *                   the value into
 * @param String
 *                   - dashboardValue - the value to display
 *
 */
public void printToShuffleboard (String dashboardKey,
        String dashboardValue)
{
    // clock determining change

    if ((System.currentTimeMillis() - lastTimePrinted) >= this
            .getTimeBetweenPrints())
        {
        SmartDashboard.putString(dashboardKey, dashboardValue);

        SmartDashboard.updateValues();
        } // if
} // end printToShuffleboard()

// in milliseconds
private double timeBetweenPrints = 2000;

// initial state
private double lastTimePrinted = 0.0;

}
