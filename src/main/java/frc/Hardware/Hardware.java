// ====================================================================
// FILE NAME: Hardware.java (Team 339 - Kilroy)
//
// CREATED ON: Jan 2, 2011
// CREATED BY: Bob Brown
// MODIFIED ON:
// MODIFIED BY:
// ABSTRACT:
// This file contains all of the global definitions for the
// hardware objects in the systemr
//
// NOTE: Please do not release this code without permission from
// Team 339.
// ====================================================================
package frc.Hardware;

import frc.HardwareInterfaces.DriveWithCamera;
import frc.HardwareInterfaces.DoubleSolenoid;
import frc.HardwareInterfaces.DoubleThrowSwitch;
import frc.HardwareInterfaces.KilroyEncoder;
import frc.HardwareInterfaces.KilroySPIGyro;
import frc.HardwareInterfaces.LVMaxSonarEZ;
import frc.HardwareInterfaces.LightSensor;
import frc.HardwareInterfaces.MomentarySwitch;
import frc.HardwareInterfaces.QuickSwitch;
import frc.HardwareInterfaces.RobotPotentiometer;
import frc.HardwareInterfaces.SingleThrowSwitch;
import frc.HardwareInterfaces.SixPositionSwitch;
import frc.Utils.drive.Drive;
import frc.Utils.drive.DrivePID;
import frc.Utils.drive.Drive.BrakeType;
import frc.robot.Teleop;
import frc.vision.VisionProcessor;
import frc.vision.VisionProcessor.CameraModel;
import frc.HardwareInterfaces.Transmission.TankTransmission;
import frc.Utils.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

/**
 * ------------------------------------------------------- puts all of the
 * hardware declarations into one place. In addition, it makes them available to
 * both autonomous and teleop.
 *
 * @class HardwareDeclarations
 * @author Bob Brown
 * @written Jan 2, 2011 -------------------------------------------------------
 */

public class Hardware
{
// ------------------------------------
// Public Constants
// ------------------------------------
public enum RobotYear
    {
    KILROY_2018, KILROY_2019, TEST_BOARD
    }

public static final RobotYear whichRobot = RobotYear.KILROY_2019;

// -------------------------------------
// Private Constants
// -------------------------------------

// ---------------------------------------
// Hardware Tunables
// ---------------------------------------
public static boolean demoMode = false;

public static int DEMO_MAX_FORK = 40;

// **********************************************************
// DIGITAL I/O CLASSES
// **********************************************************

// ====================================
// PWM classes
// ====================================

// ------------------------------------
// Jaguar classes
// ------------------------------------

// ------------------------------------
// Talon classes
// ------------------------------------

// ------------------------------------
// Victor Classes
// ------------------------------------

// ------------------------------------
// Servo classes
// ------------------------------------

// ====================================
// CAN classes
// ====================================
public static PowerDistributionPanel pdp = null;

public static SpeedController armMotor = null;

public static SpeedController liftMotor = null;

/** The right front drive motor */
public static SpeedController rightFrontCANMotor = null;

/** The left front drive motor */
public static SpeedController leftFrontCANMotor = null;

/** The right rear drive motor */
public static SpeedController rightRearCANMotor = null;

/** The left rear drive motor */
public static SpeedController leftRearCANMotor = null;

public static SpeedController armRoller = null;

// ====================================
// Relay classes
// ====================================
public static Relay ringLightRelay = null;

// ====================================
// Digital Inputs
// ====================================
// ------------------------------------
// Single and double throw switches
// ------------------------------------
public static SingleThrowSwitch leftAutoSwitch = null;

public static SingleThrowSwitch rightAutoSwitch = null;

public static DoubleThrowSwitch autoCenterSwitch = null;

public static SingleThrowSwitch levelOneSwitch = null;

public static SingleThrowSwitch levelTwoSwitch = null;

public static SingleThrowSwitch demoModeSwitch = null;

public static DoubleThrowSwitch autoDisableSwitch = null;

public static SixPositionSwitch autoSixPosSwitch = null;


// ------------------------------------
// Gear Tooth Sensors
// ------------------------------------

// ------------------------------------
// Encoders
// ------------------------------------
public static KilroyEncoder leftFrontDriveEncoder = null;

public static KilroyEncoder rightFrontDriveEncoder = null;

public static KilroyEncoder leftRearDriveEncoder = null;

public static KilroyEncoder rightRearDriveEncoder = null;

public static KilroyEncoder liftingEncoder = null;

// -----------------------
// Wiring diagram
// -----------------------
// Orange - Red PWM 1
// Yellow - White PWM 1 Signal
// Brown - Black PWM 1 (or PWM 2)
// Blue - White PWM 2 Signal
// For the AMT103 Encoders UNVERIFIED
// B - White PWM 2
// 5V - Red PWM 1 or 2
// A - White PWM 1
// X - index channel, unused
// G - Black PWM 1 or 2
// see http://www.cui.com/product/resource/amt10-v.pdf page 4 for Resolution
// (DIP Switch) Settings (currently all are off)

// -------------------------------------
// Red Light/IR Sensor class
// -------------------------------------

// ====================================
// I2C Classes
// ====================================

// **********************************************************
// SOLENOID I/O CLASSES
// **********************************************************
// ====================================
// Compressor class - runs the compressor
// ====================================

// ====================================
// Pneumatic Control Module
// ====================================

// ====================================
// Solenoids
// ====================================
// ------------------------------------
// Double Solenoids
// ------------------------------------

// ------------------------------------
// Single Solenoids
// ------------------------------------

// **********************************************************
// ANALOG I/O CLASSES
// **********************************************************
// ====================================
// Analog classes
// ====================================
// ------------------------------------
// Gyro class
// ------------------------------------
// P/N ADW22307

// --------------------------------------
// Potentiometers
// --------------------------------------

// -------------------------------------
// Sonar/Ultrasonic
// -------------------------------------

// =====================================
// SPI Bus
// =====================================

// -------------------------------------
// Analog Interfaces
// -------------------------------------
// if you are getting a null pointer exception from the gyro,
// try setting the parameter you are passing into this declaration
// to false. The null pointer exception is probably because
// there is not a gyro on the robot, and passing in a false will
// tell the robot we do not have a gyro without requiring us to
// comment out the gyro declaration.

// **********************************************************
// roboRIO CONNECTIONS CLASSES
// **********************************************************
// -------------------------------------
// Axis/USB Camera class
// -------------------------------------

// -------------------------------------
// declare the USB camera server and the
// USB camera it serves at the same time
// -------------------------------------

// -------------------------------------
// declare the USB camera server and the
// USB camera it serves at the same time
// -------------------------------------

// **********************************************************
// DRIVER STATION CLASSES
// **********************************************************
// ------------------------------------
// DriverStations class
// ------------------------------------

// ------------------------------------
// Joystick classes
// ------------------------------------

// ------------------------------------
// Buttons classes and Quick Switches
// ------------------------------------
// ----- Left Operator -----


// ----- Right Operator -----


// ------------------------------------
// Momentary Switches
// ------------------------------------

// --------------------------------------------------

// ----------Left Driver---------------

// ----------Right Driver--------------

// **********************************************************
// Kilroy's Ancillary classes
// **********************************************************

// -------------------------------------
// PID tuneables
// -------------------------------------

// -------------------------------------
// PID classes
// -------------------------------------

// ------------------------------------
// Utility classes
// ------------------------------------
public static Timer autoTimer = null;

public static Timer telopTimer = null;

public static Telemetry telemetry = null;

// ------------------------------------
// Transmission class
// ------------------------------------
public static TankTransmission transmission = null;

// ------------------------------------
// Drive system
// ------------------------------------
public static Drive drive = null;

public static DrivePID drivePID = null;

public static DriveWithCamera driveWithCamera = null;

// -------------------
// Assembly classes (e.g. forklift)
// -------------------

// ====================================
// Methods
// ====================================

/**
 * This initializes the hardware for the robot depending on which year we
 * are using
 */
public static void initialize ()
{
} // end initialize()

public static void commonInitialization ()
{

    // **********************************************************
    // DIGITAL I/O CLASSES
    // **********************************************************

    // ====================================
    // PWM classes
    // ====================================

    // ----- Jaguar classes -----
    // ----- Talon classes -----
    // ----- Victor classes ----
    // ----- Servo classes -----

    // ====================================
    // CAN classes
    // ====================================
    pdp = new PowerDistributionPanel(2);

    // ====================================
    // Relay classes
    // ====================================
    ringLightRelay = new Relay(0);

    // ====================================
    // Digital Inputs
    // ====================================
    // -------------------------------------
    // Single and double throw switches
    // -------------------------------------
    leftAutoSwitch = new SingleThrowSwitch(24);

    rightAutoSwitch = new SingleThrowSwitch(25);

    autoCenterSwitch = new DoubleThrowSwitch(leftAutoSwitch,
            rightAutoSwitch);

    levelOneSwitch = new SingleThrowSwitch(22);

    levelTwoSwitch = new SingleThrowSwitch(23);

    demoModeSwitch = new SingleThrowSwitch(0);

    autoDisableSwitch = new DoubleThrowSwitch(levelOneSwitch,
            levelTwoSwitch);

    autoSixPosSwitch = new SixPositionSwitch(13, 14, 15, 16, 17, 18);

    // Gear Tooth Sensors

    // Encoders
    // -------------------------------------
    // Red Light/IR Sensor class
    // -------------------------------------

    // ====================================
    // I2C Classes
    // ====================================

    // **********************************************************
    // SOLENOID I/O CLASSES
    // **********************************************************
    // ====================================
    // Compressor class - runs the compressor
    // ====================================

    // ====================================
    // Pneumatic Control Module
    // ====================================

    // ====================================
    // Solenoids
    // ====================================

    // Double Solenoids

    // Single Solenoids

    // **********************************************************
    // ANALOG I/O CLASSES
    // **********************************************************
    // ====================================
    // Analog classes
    // ====================================

    // Gyro class

    // P/N ADW22307

    // Potentiometers

    // Sonar/Ultrasonic

    // =====================================
    // SPI Bus
    // =====================================

    // Analog Interfaces

    // **********************************************************
    // roboRIO CONNECTIONS CLASSES
    // **********************************************************

    // Axis/USB Camera class

    // -------------------------------------
    // declare the USB camera server and the
    // USB camera it serves at the same time
    // -------------------------------------
    // TODO: put somewhere useful
    // Camera settings: 320-240, 20fps, 87 ????

    // **********************************************************
    // DRIVER STATION CLASSES
    // **********************************************************

    // DriverStations class

    // Joystick classes

    // Buttons classes
    // ----- Left Operator -----

    // left trigger

    // cargoShipCargoButton = new QuickSwitch(leftOperator, 11);

    // ----- Right Operator -----

    // right trigger

    // ----------Left Driver---------------


    // Momentary Switches


    // ----------Right Driver--------------

    // Momentary Switches

    // ----------Right Driver--------------

    // **********************************************************
    // Kilroy's Ancillary classes
    // **********************************************************
    // PID tuneables
    // PID classes
    // Utility classes
    autoTimer = new Timer();

    telopTimer = new Timer();

    telemetry = new Telemetry(10000);

    // Transmission class
    transmission = new TankTransmission(
            new SpeedControllerGroup(leftFrontCANMotor,
                    leftRearCANMotor),
            new SpeedControllerGroup(rightFrontCANMotor,
                    rightRearCANMotor));

    // ------------------------------------
    // Drive system
    // ------------------------------------
//    drive = new Drive(transmission,
//            leftFrontDriveEncoder, rightFrontDriveEncoder,
//            gyro);

//    drivePID = new DrivePID(transmission,
//            leftFrontDriveEncoder, rightFrontDriveEncoder,
//            leftFrontDriveEncoder, rightFrontDriveEncoder, gyro);

//   driveWithCamera = new DriveWithCamera(
//            transmission, null, null, frontUltraSonic,
//            frontUltraSonic, gyro, axisCamera);

    // Assembly classes (e.g. forklift)

} // end of commonInitialization()

} // end class
