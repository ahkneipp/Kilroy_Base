/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;
import frc.Hardware.Hardware;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot
{
private static final String kDefaultAuto = "Default";

// private static final String kCustomAuto = "My Auto";
private String m_autoSelected;
// private final SendableChooser<String> m_chooser = new SendableChooser<>();

/**
 * ------------------------------------------------------- This function is run
 * when the robot is first started up and should be used for any initialization
 * code for the robot.
 *
 * @author Bob Brown
 * @written Jan 2, 2011 -------------------------------------------------------
 */
@Override
public void robotInit ()
{
    // ---------------------------------------
    // start setup - tell the user we are beginning
    // setup
    // ---------------------------------------
    System.out.println("Started robotInit()");

    // =========================================================
    // User code goes below here
    // =========================================================
    Hardware.initialize();

    Hardware.setHardwareSettings();


    // User code goes above here
    // =========================================================
    // ---------------------------------------
    // done setup - tell the user we are complete
    // setup
    // ---------------------------------------
    // Teleop.printStatements();

    System.out.println(
            Hardware.whichRobot
                    + " is started.  All hardware items created.");

} // end robotInit()

/**
 * This function is called every robot packet, no matter the mode. Use this for
 * items like diagnostics that you want ran during disabled, autonomous,
 * teleoperated and test.
 *
 * <p>
 * This runs after the mode specific periodic functions, but before LiveWindow
 * and SmartDashboard integrated updating.
 */
@Override
public void robotPeriodic ()
{
} // end robotPeriodic()

/**
 * ------------------------------------------------------- Initialization code
 * for autonomous mode should go here. Will be called once when the robot enters
 * autonomous mode.
 *
 * @author Bob Brown
 * @written Jan 2, 2011
 *
 *          -------------------------------------------------------
 */
@Override
public void autonomousInit ()
{
    // ---------------------------------------
    // start setup - tell the user we are beginning
    // setup
    // ---------------------------------------
    System.out.println("Started AutonousInit().");

    // =========================================================
    // User code goes below here
    // =========================================================
    // m_autoSelected = SmartDashboard.getString("Auto Selector",
    // kDefaultAuto);
    Autonomous.init();


    // =========================================================
    // User code goes above here
    // =========================================================
    // ---------------------------------------
    // done setup - tell the user we are complete
    // setup
    // ---------------------------------------
    System.out.println("Completed AutonousInit().");
} // end autonomousInit()

/**
 * ------------------------------------------------------- Non-User Periodic
 * code for autonomous mode should go here. Will be called periodically at a
 * regular rate while the robot is in autonomous mode. This in turn calls the
 * Autonomous class's Periodic function, which is where the user code should be
 * placed.
 *
 * @author Bob Brown
 * @written Jan 2, 2011
 *
 *          -------------------------------------------------------
 */
@Override
public void autonomousPeriodic ()
{
    // =========================================================
    // User code goes below here
    // =========================================================
    Autonomous.periodic();

    // =========================================================
    // User code goes above here
    // =========================================================
}// end autonomousPeriodic()

/**
 * ------------------------------------------------------- Initialization code
 * for disabled mode should go here. Will be called once when the robot enters
 * disabled mode.
 *
 * @author Bob Brown
 * @written Jan 2, 2011 -------------------------------------------------------
 */
@Override
public void disabledInit ()
{
    // ---------------------------------------
    // start setup - tell the user we are beginning
    // setup
    // ---------------------------------------
    System.out.println("Started DisabledInit().");
    // =========================================================
    // User code goes below here
    // =========================================================

    // =========================================================
    // User code goes above here
    // =========================================================
    // ---------------------------------------
    // done setup - tell the user we are complete
    // setup
    // ---------------------------------------
    System.out.println("Completed DisabledInit().");
} // end disabledInit()

/**
 * ------------------------------------------------------- Periodic code for
 * disabled mode should go here. Will be called periodically at a regular rate
 * while the robot is in disabled mode. Code that can be "triggered" by a
 * joystick button can go here. This can set up configuration things at the
 * driver's station for instance before a match.
 *
 * @author Bob Brown
 * @written Jan 2, 2011 -------------------------------------------------------
 */
@Override
public void disabledPeriodic ()
{
    // =========================================================
    // User code goes below here
    // =========================================================

    // =========================================================
    // User code goes above here
    // =========================================================
} // end disabledPeriodic()

/**
 * ------------------------------------------------------ Non-User
 * initialization code for teleop mode should go here. Will be called once when
 * the robot enters teleop mode, and will call the Teleop class's Init function,
 * where the User code should be placed.
 *
 * @author Bob Brown
 * @written Jan 2, 2011 -------------------------------------------------------
 */
@Override
public void teleopInit ()
{
    // ---------------------------------------
    // start setup - tell the user we are beginning
    // setup
    // ---------------------------------------
    System.out.println("Started teleopInit().");
    // =========================================================
    // User code goes below here
    // =========================================================
    Teleop.init();

    // =========================================================
    // User code goes above here
    // =========================================================
    // ---------------------------------------
    // done setup - tell the user we are complete
    // setup
    // ---------------------------------------
    System.out.println("Completed TeleopInit().");
} // end teleopInit()

/**
 * ------------------------------------------------------- Non-User Periodic
 * code for teleop mode should go here. Will be called periodically at a regular
 * rate while the robot is in teleop mode, and will in turn call the Teleop
 * class's Periodic function.
 *
 * @author Bob Brown
 * @written Jan 2, 2011 -------------------------------------------------------
 */
@Override
public void teleopPeriodic ()
{
    // -------------------------------------
    // Call the Teleop class's Periodic function,
    // which contains the user code.
    // -------------------------------------

    // =========================================================
    // User code goes below here
    // =========================================================
   Teleop.periodic();

    // =========================================================
    // User code goes above here
    // =========================================================

} // end teleopPeriodic()

/**
 * ------------------------------------------------------- Initialization code
 * for test mode should go here. Will be called once when the robot enters test
 * mode.
 *
 * @author Bob Brown
 * @written Jan 2, 2015 ------------------------------------------------------
 */
@Override
public void testInit ()
{
    // =========================================================
    // User code goes below here
    // =========================================================
    this.teleopInit();
    // =========================================================
    // User code goes above here
    // =========================================================

} // end testInit()

/**
 * This function is called periodically during test mode.
 */
@Override
public void testPeriodic ()
{
} // end testPeriodic()

// ==========================================
// TUNEABLES
// ==========================================

} // end robot class
