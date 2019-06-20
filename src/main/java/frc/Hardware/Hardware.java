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
public static LightSensor armBallDetector = null;

public static LightSensor leftBackIR = null;

// the IR on the back right part of the robot
public static LightSensor rightBackIR = null;

// ====================================
// I2C Classes
// ====================================

// **********************************************************
// SOLENOID I/O CLASSES
// **********************************************************
// ====================================
// Compressor class - runs the compressor
// ====================================
public static Compressor compressor = null;

// ====================================
// Pneumatic Control Module
// ====================================

// ====================================
// Solenoids
// ====================================
// ------------------------------------
// Double Solenoids
// ------------------------------------
public static DoubleSolenoid armIntakeSolenoid = null;

public static DoubleSolenoid driveSolenoid = null;

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
public static RobotPotentiometer delayPot = null;

public static RobotPotentiometer armPot = null;

// -------------------------------------
// Sonar/Ultrasonic
// -------------------------------------
public static LVMaxSonarEZ frontUltraSonic = null;

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
public static KilroySPIGyro gyro = null;

// **********************************************************
// roboRIO CONNECTIONS CLASSES
// **********************************************************
// -------------------------------------
// Axis/USB Camera class
// -------------------------------------
public static VisionProcessor axisCamera = null;

public static String axisCameraIp = null;

// -------------------------------------
// declare the USB camera server and the
// USB camera it serves at the same time
// -------------------------------------
public static UsbCamera USBCam = null;

public static UsbCamera USBCamII = null;

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
public static DriverStation driverStation = null;

// ------------------------------------
// Joystick classes
// ------------------------------------
public static Joystick leftDriver = null;

public static Joystick rightDriver = null;

public static Joystick leftOperator = null;

public static Joystick rightOperator = null;

// ------------------------------------
// Buttons classes and Quick Switches
// ------------------------------------
// ----- Left Operator -----


public static MomentarySwitch alignAndStopButton = null;

public static JoystickButton intakeTriggerLeft = null;

public static JoystickButton intakeTriggerRight = null;

public static JoystickButton outtakeButtonLeft = null;

public static JoystickButton outtakeButtonRight = null;

public static QuickSwitch armSolenoidToggleButton = null;

public static JoystickButton intakeOverride = null;

public static JoystickButton deployOverride = null;

public static QuickSwitch playerStationCargoButton = null;

public static QuickSwitch playerStationButton = null;

// public static QuickSwitch cargoShipCargoButton = null;

public static JoystickButton visionHeightDownButton = null;

public static JoystickButton visionHeightUpButton = null;

public static JoystickButton upshiftButton = null;

public static JoystickButton poweredManipulatorForClimbButton = null;

public static QuickSwitch toggleIgnoreMakeBreakButton = null;


// ----- Right Operator -----

public static JoystickButton pictureButtonOne = null;

public static JoystickButton pictureButtonTwo = null;

public static JoystickButton chooseCargoRocketHeights = null;

public static JoystickButton forkliftOverride = null;

public static QuickSwitch nextHigherLiftHeightButton = null;

public static QuickSwitch nextLowerLiftHeightButton = null;

public static MomentarySwitch alignVisionButton = null;

public static JoystickButton downshiftButton = null;

// ------------------------------------
// Momentary Switches
// ------------------------------------
public static JoystickButton driveStraightButton = null;

public static MomentarySwitch solenoidButtonOne = null;

public static MomentarySwitch solenoidButtonTwo = null;

public static MomentarySwitch armHackButton = null;

public static JoystickButton liftHackButton = null;

public static MomentarySwitch descendButton = null;

public static MomentarySwitch ringLightButton = null;

// --------------------------------------------------

public static JoystickButton climbOneButton = null;

public static JoystickButton climbTwoButton = null;

// ----------Left Driver---------------
public static JoystickButton resetForkliftEncoderButton1 = null;

public static JoystickButton cancelOneButton = null;

public static JoystickButton cancelAutoLeftDriver = null;


// ----------Right Driver--------------
public static JoystickButton resetForkliftEncoderButton2 = null;

public static JoystickButton cancelTwoButton = null;

public static JoystickButton cancelAutoRightDriver = null;

public static JoystickButton retrievalButton = null;
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

public static Timer takePictureTimer = null;

public static Timer telopTimer = null;

public static Timer ringLightTimer = null;

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
public static GamePieceManipulator manipulator = null;

public static Forklift lift = null;

public static ClimbToLevelTwo climber = null;

public static AlignPerpendicularToTape alignByTape = null;

public static DepositGamePiece depositGamePiece = null;

public static RetrieveHatch retriever = null;

// ====================================
// Methods
// ====================================

/**
 * This initializes the hardware for the robot depending on which year we
 * are using
 */
public static void initialize ()
{
    // ---------------------------
    // any hardware declarations that
    // are exactly the same between 2018
    // and 2019. Make them only once
    // ---------------------------


    switch (whichRobot)
        {
        case KILROY_2018:
            axisCameraIp = "10.13.39.11";
            robotInitialize2018();
            break;

        default:
        case KILROY_2019:
            axisCameraIp = "10.3.39.11";
            robotInitialize2019();
            break;

        case TEST_BOARD:
            break;
        } // end switch
          // ------------------------
          // The function calls in commonKilroyAncillary
          // must follow all other hardware declarations
          // -------------------------
    commonInitialization();
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
    armBallDetector = new LightSensor(21);

    leftBackIR = new LightSensor(8);


    rightBackIR = new LightSensor(9);

    // ====================================
    // I2C Classes
    // ====================================

    // **********************************************************
    // SOLENOID I/O CLASSES
    // **********************************************************
    // ====================================
    // Compressor class - runs the compressor
    // ====================================
    compressor = new Compressor();

    // ====================================
    // Pneumatic Control Module
    // ====================================

    // ====================================
    // Solenoids
    // ====================================

    // Double Solenoids

    driveSolenoid = new DoubleSolenoid(2,
            3);

    armIntakeSolenoid = new DoubleSolenoid(4, 5);



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

    delayPot = new RobotPotentiometer(2, 300);

    armPot = new RobotPotentiometer(1, 300);

    // Sonar/Ultrasonic
    frontUltraSonic = new LVMaxSonarEZ(3);

    // =====================================
    // SPI Bus
    // =====================================

    // Analog Interfaces
    gyro = new KilroySPIGyro(true);

    // **********************************************************
    // roboRIO CONNECTIONS CLASSES
    // **********************************************************

    // Axis/USB Camera class

    axisCamera = new VisionProcessor(axisCameraIp,
            CameraModel.AXIS_M1013,
            ringLightRelay);
    // -------------------------------------
    // declare the USB camera server and the
    // USB camera it serves at the same time
    // -------------------------------------
    // TODO: put somewhere useful
    // Camera settings: 320-240, 20fps, 87 ????
    USBCam = CameraServer.getInstance().startAutomaticCapture(0);


    USBCamII = CameraServer.getInstance().startAutomaticCapture(1);

    // **********************************************************
    // DRIVER STATION CLASSES
    // **********************************************************

    // DriverStations class

    driverStation = DriverStation.getInstance();

    // Joystick classes
    leftDriver = new Joystick(0);

    rightDriver = new Joystick(1);

    leftOperator = new Joystick(2);

    rightOperator = new Joystick(3);

    // Buttons classes
    // ----- Left Operator -----

    cancelOneButton = new JoystickButton(leftOperator, 10);

    // left trigger
    intakeTriggerLeft = new JoystickButton(leftOperator, 1);

    outtakeButtonLeft = new JoystickButton(leftOperator, 2);

    armSolenoidToggleButton = new QuickSwitch(leftOperator, 3);

    alignAndStopButton = new MomentarySwitch(leftOperator, 4, false);

    playerStationCargoButton = new QuickSwitch(leftOperator, 6);

    playerStationButton = new QuickSwitch(leftOperator, 7);

    // cargoShipCargoButton = new QuickSwitch(leftOperator, 11);

    visionHeightUpButton = new JoystickButton(leftOperator, 9);//

    visionHeightDownButton = new JoystickButton(leftOperator, 8);

    alignVisionButton = new MomentarySwitch(leftOperator, 5, false);

    toggleIgnoreMakeBreakButton = new QuickSwitch(leftOperator, 11);

    // ----- Right Operator -----

    pictureButtonOne = new JoystickButton(rightOperator, 8);

    pictureButtonTwo = new JoystickButton(rightOperator, 9);

    cancelTwoButton = new JoystickButton(rightOperator, 10);

    // right trigger

    intakeTriggerRight = new JoystickButton(rightOperator, 1);

    outtakeButtonRight = new JoystickButton(rightOperator, 2);

    chooseCargoRocketHeights = new JoystickButton(rightOperator, 4);

    forkliftOverride = new JoystickButton(rightOperator, 5);

    intakeOverride = forkliftOverride;

    deployOverride = forkliftOverride;

    nextHigherLiftHeightButton = new QuickSwitch(rightOperator, 6);

    nextLowerLiftHeightButton = new QuickSwitch(rightOperator,
            7);

    // ----------Left Driver---------------


    // Momentary Switches

    upshiftButton = new JoystickButton(leftDriver, 1);

    // ----------Right Driver--------------

    driveStraightButton = new JoystickButton(rightDriver, 2);

    solenoidButtonOne = new MomentarySwitch(leftDriver, 7, false);

    solenoidButtonTwo = new MomentarySwitch(leftDriver, 8, false);

    retrievalButton = new JoystickButton(leftDriver, 9);// fix button
                                                        // yeeeeeeeeeeee

    climbOneButton = new JoystickButton(leftDriver, 11);

    climbTwoButton = new JoystickButton(rightDriver, 11);

    cancelAutoRightDriver = new JoystickButton(rightDriver, 10);

    cancelAutoLeftDriver = new JoystickButton(leftDriver, 10);

    downshiftButton = new JoystickButton(rightDriver, 1);

    // Momentary Switches

    // ----------Right Driver--------------


    poweredManipulatorForClimbButton = new JoystickButton(rightDriver,
            6);


    resetForkliftEncoderButton1 = new JoystickButton(rightDriver, 7);

    resetForkliftEncoderButton2 = new JoystickButton(rightDriver, 8);



    // hacks
    // armHackButton = new MomentarySwitch(rightDriver, 6, false);

    // liftHackButton = new JoystickButton(rightDriver, 5);

    // descendButton = new MomentarySwitch(leftOperator, 5, false);

    // ringLightButton = new MomentarySwitch(leftOperator, 6, false);

    // **********************************************************
    // Kilroy's Ancillary classes
    // **********************************************************
    // PID tuneables
    // PID classes
    // Utility classes
    autoTimer = new Timer();

    takePictureTimer = new Timer();

    telopTimer = new Timer();

    ringLightTimer = new Timer();



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
    drive = new Drive(transmission,
            leftFrontDriveEncoder, rightFrontDriveEncoder,
            gyro);

    drivePID = new DrivePID(transmission,
            leftFrontDriveEncoder, rightFrontDriveEncoder,
            leftFrontDriveEncoder, rightFrontDriveEncoder, gyro);

    driveWithCamera = new DriveWithCamera(
            transmission, null, null, frontUltraSonic,
            frontUltraSonic, gyro, axisCamera);

    // Assembly classes (e.g. forklift)
    manipulator = new GamePieceManipulator(
            armMotor, armPot/* armEncoder */,
            armRoller,
            armBallDetector,
            armIntakeSolenoid);

    lift = new Forklift(liftMotor, liftingEncoder, manipulator);

    alignByTape = new AlignPerpendicularToTape(leftBackIR, rightBackIR,
            drive);

    depositGamePiece = new DepositGamePiece(drive, lift, manipulator);



    climber = new ClimbToLevelTwo(
            driveSolenoid, armMotor, armPot,
            drive, lift, frontUltraSonic);


    retriever = new RetrieveHatch();

} // end of commonInitialization()

/**
 * This initializes all of the components in Hardware
 */
public static void robotInitialize2018 ()
{
    // **********************************************************
    // DIGITAL I/O CLASSES
    // **********************************************************

    // ====================================
    // PWM classes
    // ====================================

    // ----- Jaguar classes -----
    // ----- Talon classes -----
    // ----- Victor classes -----

    armMotor = new VictorSP(4);

    // ----- Servo classes -----

    // ====================================
    // CAN classes
    // ====================================
    liftMotor = new WPI_TalonSRX(23);

    rightFrontCANMotor = new WPI_TalonSRX(14);

    leftFrontCANMotor = new WPI_TalonSRX(11);

    rightRearCANMotor = new WPI_TalonSRX(15);

    leftRearCANMotor = new WPI_TalonSRX(13);

    armRoller = new WPI_TalonSRX(10);

    // ====================================
    // Relay classes
    // ====================================

    // ====================================
    // Digital Inputs
    // ====================================
    // -------------------------------------
    // Single and double throw switches
    // -------------------------------------

    // Gear Tooth Sensors

    // Encoders
    leftFrontDriveEncoder = new KilroyEncoder(4, 5);

    rightFrontDriveEncoder = new KilroyEncoder(6, 7);

    liftingEncoder = new KilroyEncoder(10, 11);

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

    // **********************************************************
    // DRIVER STATION CLASSES
    // **********************************************************

    // DriverStations class

    // Joystick classes

    // Buttons classes
    // ----- Left Operator -----

    // left trigger

    // ----- Right Operator -----

    // Momentary Switches

}  // end of robotInitialize2018()

/**
 * This initializes all of the components in Hardware
 */
public static void robotInitialize2019 ()
{
    // **********************************************************
    // DIGITAL I/O CLASSES
    // **********************************************************

    // ====================================
    // PWM classes
    // ====================================

    // ----- Jaguar classes -----
    // ----- Talon classes -----
    // ----- Victor classes -----
    // ----- Servo classes -----

    // ====================================
    // CAN classes
    // ====================================
    armMotor = new WPI_TalonSRX(24);

    liftMotor = new WPI_TalonSRX(23);

    rightFrontCANMotor = new CANSparkMax(14, MotorType.kBrushless);

    leftFrontCANMotor = new CANSparkMax(11, MotorType.kBrushless);

    rightRearCANMotor = new CANSparkMax(15, MotorType.kBrushless);

    leftRearCANMotor = new CANSparkMax(13, MotorType.kBrushless);

    armRoller = new WPI_TalonSRX(10);

    // ====================================
    // Relay classes
    // ====================================

    // ====================================
    // Digital Inputs
    // ====================================
    // -------------------------------------
    // Single and double throw switches
    // -------------------------------------

    // Gear Tooth Sensors

    // Encoders
    leftFrontDriveEncoder = new KilroyEncoder(
            (CANSparkMax) leftFrontCANMotor);

    rightFrontDriveEncoder = new KilroyEncoder(
            (CANSparkMax) rightFrontCANMotor);

    leftRearDriveEncoder = new KilroyEncoder(
            (CANSparkMax) leftRearCANMotor);

    rightRearDriveEncoder = new KilroyEncoder(
            (CANSparkMax) rightRearCANMotor);

    liftingEncoder = new KilroyEncoder((BaseMotorController) liftMotor);

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
    axisCamera = new VisionProcessor(axisCameraIp,
            CameraModel.AXIS_M1013,
            ringLightRelay);

    // -------------------------------------
    // declare the USB camera server and the
    // USB camera it serves at the same time
    // -------------------------------------

    // **********************************************************
    // DRIVER STATION CLASSES
    // **********************************************************

    // DriverStations class
    // DriverStations class

    // Joystick classes

    // Buttons classes

    // ----- Left Operator -----

    // left trigger

    // ----- Right Operator -----

    // Momentary Switches

} // end robotInitialize2019()

/**
 * it's a switch statement for the current robot, a robot we don't have,
 * and a robot Mr. Brown said not to use.
 *
 * @author Patrick
 */
public static void setHardwareSettings ()
{
    if (Hardware.demoModeSwitch.isOn())
        {
        Hardware.demoMode = true;
        }
    switch (whichRobot)
        {
        case KILROY_2018:
            setHardwareSettings2018();
            break;

        default:
        case KILROY_2019:
            setHardwareSettings2019();
            break;

        case TEST_BOARD:
            break;
        }
    commonHardwareSettings();
} // end setHardwareSettings()

/**
 * This sets up the settings and resets for the hardware objects so we
 * don't have to write them all in robotInit. it's to keep it not cluttered.
 *
 * @author Patrick
 */
public static void commonHardwareSettings ()
{
    // ------------------------------
    // starts the compressor
    // ------------------------------
    Hardware.compressor.setClosedLoopControl(true);

    // ------------------------------
    // must be calibrated before we can
    // use the gyro
    // ------------------------------
    Hardware.gyro.calibrate();
    Hardware.gyro.reset();

    // ------------------------------
    // camera setup
    // ------------------------------
    // written by Meghan Brown 2019
    // makes the camera work --- can we get this in colour somehow?
    // Hardware.USBCam.setResolution(320, 240);
    // Hardware.USBCam.setFPS(20);
    // Hardware.USBCam.setPixelFormat(VideoMode.PixelFormat.kYUYV);
    // Hardware.USBCam.setWhiteBalanceManual(2500);

    // Hardware.USBCamII.setResolution(320, 240);
    // Hardware.USBCamII.setFPS(20);
    // Hardware.USBCamII.setPixelFormat(VideoMode.PixelFormat.kYUYV);
    // Hardware.USBCamII.setWhiteBalanceManual(2500);

    // Hardware.USBCamUp.setResolution(320, 240);
    // Hardware.USBCamUp.setFPS(20);
    // Hardware.USBCamUp.setPixelFormat(VideoMode.PixelFormat.kYUYV);

    // --------------------------------------
    // reset the MotorSafetyHelpers for each
    // of the drive motors
    // --------------------------------------


    Hardware.transmission
            .setJoystickDeadband(JOYSTICK_DEADBAND);

} // end commonHardwareSettings()

/**
 * This sets up the settings and resets for the hardware objects so we
 * don't have to write them all in robotInit. it's to keep it not cluttered.
 *
 * @author Patrick
 */
public static void setHardwareSettings2018 ()
{
    // Hardware.drive.setBrakeIterations(4);
    // Hardware.drive.setBrakeDeadband(20, BrakeType.AFTER_DRIVE);
    // Hardware.drive.setDebugOnStatus(Drive.debugType.DEBUG_BRAKING);
    // ----------------------------
    // motor initialization
    // ----------------------------
    Hardware.rightFrontCANMotor.setInverted(true);
    Hardware.rightRearCANMotor.setInverted(true);
    Hardware.leftFrontCANMotor.setInverted(false);
    Hardware.leftRearCANMotor.setInverted(false);

    // ---------------------------
    // Encoder Initialization
    // ---------------------------
    Hardware.rightFrontDriveEncoder.setReverseDirection(false);
    Hardware.leftFrontDriveEncoder.setReverseDirection(false);
    Hardware.liftingEncoder.setReverseDirection(true);

    // -------------------------------------
    // Resets encoder values
    // -------------------------------------
    Hardware.rightFrontDriveEncoder.reset();
    Hardware.leftFrontDriveEncoder.reset();
    Hardware.liftingEncoder.reset();

    // -------------------------------------
    // Manually sets encoders Distance per Pulse
    // -------------------------------------
    Hardware.leftFrontDriveEncoder
            .setDistancePerPulse(KILROY_XIX_DRIVE_ENCODER_DPP);
    Hardware.rightFrontDriveEncoder
            .setDistancePerPulse(KILROY_XIX_DRIVE_ENCODER_DPP);
    Hardware.liftingEncoder
            .setDistancePerPulse(KILROY_XIX_LIFT_ENCODER_DPP);

    Hardware.lift.initiliazeConstantsFor2018();
    Hardware.manipulator.initiliazeConstantsFor2018();
} // end setHardwareSettings2018()

/**
 * This sets up the settings and resets for the hardware objects so we
 * don't have to write them all in robotInit. it's to keep it not cluttered.
 *
 * @author Patrick
 */
public static void setHardwareSettings2019 ()
{



    // Hardware.drive.setBrakeIterations(20);
    // Hardware.drive.setBrakeDeadband(50, BrakeType.AFTER_DRIVE);
    // ----------------------------
    // motor initialization
    // ----------------------------
    Hardware.rightFrontCANMotor.setInverted(true);
    Hardware.rightRearCANMotor.setInverted(true);
    Hardware.leftFrontCANMotor.setInverted(false);
    Hardware.leftRearCANMotor.setInverted(false);
    Hardware.armMotor.setInverted(true);


    // ---------------------------
    // Encoder Initialization
    // ---------------------------
    // Hardware.rightFrontDriveEncoder.setReverseDirection(false);
    // Hardware.leftFrontDriveEncoder.setReverseDirection(false);
    // Hardware.rightRearDriveEncoder.setReverseDirection(false);
    // Hardware.leftRearDriveEncoder.setReverseDirection(false);
    // Hardware.liftingEncoder.setReverseDirection(false);

    // -------------------------------------
    // Manually sets encoders Distance per Pulse
    // -------------------------------------
    Hardware.leftFrontDriveEncoder
            .setDistancePerPulse(KILROY_XX_DRIVE_ENCODER_DPP);
    Hardware.rightFrontDriveEncoder
            .setDistancePerPulse(KILROY_XX_DRIVE_ENCODER_DPP);
    Hardware.leftFrontDriveEncoder
            .setDistancePerPulse(KILROY_XX_DRIVE_ENCODER_DPP);
    Hardware.rightFrontDriveEncoder
            .setDistancePerPulse(KILROY_XX_DRIVE_ENCODER_DPP);
    Hardware.liftingEncoder
            .setDistancePerPulse(KILROY_XX_LIFT_ENCODER_DPP);

    // -------------------------------------
    // Resets encoder values
    // -------------------------------------
    Hardware.rightFrontDriveEncoder.reset();
    Hardware.leftFrontDriveEncoder.reset();
    Hardware.rightRearDriveEncoder.reset();
    Hardware.leftRearDriveEncoder.reset();
    Hardware.liftingEncoder.reset();


    Hardware.drive
            .setMaxBrakeIterations(KILROY_XX_MAX_BRAKE_ITERATIONS);

    Hardware.drive.setBrakePower(.5, BrakeType.AFTER_DRIVE);

    // Solenoid stuff

    armIntakeSolenoid.setForward(true);
} // end setHardwareSettings2019()

private static final double KILROY_XIX_DRIVE_ENCODER_DPP = 0.0346;

private static final double KILROY_XIX_LIFT_ENCODER_DPP = 0.02;

private static final double KILROY_XX_DRIVE_ENCODER_DPP = 1.68;

// This value was tested on 15 Feb 2019; it gives values within
// .1 inch of what we want
private static final double KILROY_XX_LIFT_ENCODER_DPP = 0.0094;


private static final int KILROY_XX_MAX_BRAKE_ITERATIONS = 5;

private static final double JOYSTICK_DEADBAND = 0.2;

} // end class
