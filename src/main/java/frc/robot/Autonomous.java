/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
// ====================================================================
// FILE NAME: Autonomous.java (Team 339 - Kilroy)
//
// CREATED ON: Jan 13, 2015
// CREATED BY: Nathanial Lydick
// MODIFIED ON:
// MODIFIED BY:
// ABSTRACT:
// This file is where almost all code for Kilroy will be
// written. Some of these functions are functions that should
// override methods in the base class (IterativeRobot). The
// functions are as follows:
// -----------------------------------------------------
// Init() - Initialization code for autonomous mode
// should go here. Will be called each time the robot enters
// autonomous mode.
// -----------------------------------------------------
// Periodic() - Periodic code for autonomous mode should
// go here. Will be called periodically at a regular rate while
// the robot is in autonomous mode.
// -----------------------------------------------------
//
// NOTE: Please do not release this code without permission from
// Team 339.
// ====================================================================
package frc.robot;

import frc.Hardware.Hardware;
import frc.HardwareInterfaces.DriveWithCamera;
import frc.HardwareInterfaces.LightSensor;
import frc.HardwareInterfaces.DriveWithCamera.Side;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Utils.drive.Drive;
import frc.Utils.drive.Drive.BrakeType;
import edu.wpi.first.cameraserver.CameraServer;


/**
 * An Autonomous class. This class <b>beautifully</b> uses state machines in
 * order to periodically execute instructions during the Autonomous period.
 *
 * This class contains all of the user code for the Autonomous part of the
 * match, namely, the Init and Periodic code
 *
 *
 * @author Michael Andrzej Klaczynski
 * @written at the eleventh stroke of midnight, the 28th of January, Year of our
 *          LORD 2016. Rewritten ever thereafter.
 *
 * @author Nathanial Lydick
 * @written Jan 13, 2015
 */
public class Autonomous
{

/**
 * User Initialization code for autonomous mode should go here. Will run once
 * when the autonomous first starts, and will be followed immediately by
 * periodic().
 */
public static void init ()
{

    switch (Hardware.whichRobot)
        {
        case KILROY_2018:
            Hardware.ringLightRelay.set(Value.kOff);
            Hardware.drive.setGearPercentage(FIRST_GEAR_NUMBER,
                    FIRST_AUTO_GEAR_RATIO_KILROY_XIX);
            Hardware.drive.setGearPercentage(SECOND_GEAR_NUMBER,
                    SECOND_AUTO_GEAR_RATIO_KILROY_XIX);
            // sets the gear to 0 at the beginning.
            Hardware.drive.setGear(0);
            // ------------------------------------------------------
            DRIVE_SPEED = KILROY_XIX_DRIVE_SPEED;
            TURN_SPEED = KILROY_XIX_TURN_SPEED;
            LEAVE_LEVEL_ONE_SPEED = KILROY_XIX_LEAVE_LEVEL_ONE_SPEED;

            break;

        default:
        case KILROY_2019:
            Hardware.ringLightRelay.set(Value.kOn);
            // Hardware.axisCamera.processImage();
            Hardware.drive.setGearPercentage(FIRST_GEAR_NUMBER,
                    FIRST_AUTO_GEAR_RATIO_KILROY_XX);
            Hardware.drive.setGearPercentage(SECOND_GEAR_NUMBER,
                    SECOND_AUTO_GEAR_RATIO_KILROY_XX);
            // sets the gear to 0 at the beginning.
            Hardware.drive.setGear(0);
            // ---------------------------------------------------------
            DRIVE_SPEED = KILROY_XX_DRIVE_SPEED;
            TURN_SPEED = KILROY_XIX_TURN_SPEED;
            LEAVE_LEVEL_ONE_SPEED = KILROY_XX_LEAVE_LEVEL_ONE_SPEED;
            // Hardware.manipulator.initiliazeArmPositonAverage();
            break;

        case TEST_BOARD:
            break;

        } // end switch
    // --------------------------------------
    // reset the MotorSafetyHelpers for each
    // of the drive motors
    // --------------------------------------
    // Hardware.leftDriveMotor.setSafetyEnabled(false);
    // Hardware.rightDriveMotor.setSafetyEnabled(false);

    // Hardware.leftFrontDriveEncoder.reset();
    // Hardware.rightFrontDriveEncoder.reset();
    Hardware.gyro.reset();

    // Hardware.axisCamera.setRelayValue(Value.kOff);

    Hardware.liftingEncoder.reset();

    if (Hardware.autoDisableSwitch.isOn() == true)
        {
        autoLevel = Level.DISABLE;
        autoState = State.FINISH;
        }

} // end Init

/**
 * State of autonomous as a whole; mainly for init, delay, finish, and choosing
 * which autonomous path is being used
 */
public static enum State
    {
    INIT, DELAY, CHOOSE_PATH, CROSS_AUTOLINE, DEPOSIT_STRAIGHT_CARGO_HATCH, DEPOSIT_ROCKET_HATCH, DEPOSIT_SIDE_CARGO_BALL, BLIND_ROCKET_HATCH, JANKY_DEPOSIT_STRAIGHT, FINISH
    }

/**
 * Starting position and which side of the field the robot is going to
 */

public static enum Position
    {
    LEFT, RIGHT, CENTER, NULL
    }

public static enum Level
    {
    LEVEL_ONE, LEVEL_TWO, DISABLE, NULL
    }

// variable that controls the state of autonomous as a whole (init, delay
// which path is being used, etc.)
public static State autoState = State.INIT;

// variable that controls the starting position/side (Left, Right, or Center) of
// the robot

public static Position autoPosition = Position.NULL;

// variable that controls the level the robot is starting on (Level 1 or level 2
// or disabled)

public static Level autoLevel = Level.NULL;

/**
 * User Periodic code for autonomous mode should go here. Will be called
 * periodically at a regular rate while the robot is in autonomous mode.
 *
 * @author Nathanial Lydick
 * @written Jan 13, 2015
 *
 *          FYI: drive.stop cuts power to the motors, causing the robot to
 *          coast. drive.brake results in a more complete stop.
 *          Meghan Brown; 10 February 2019
 *
 */
public static boolean canceledAuto = false;

public static void periodic ()
{


    if (Hardware.cancelAutoLeftDriver.get() == true
            && Hardware.cancelAutoRightDriver.get() == true)
        {
        endAutoPath();
        canceledAuto = true;

        // Hardware.USBCam = CameraServer.getInstance()
        // .startAutomaticCapture(0);
        autoState = State.FINISH;

        }
    // Teleop.printStatements();
    // Hardware.lift.update();
    // System.out.println(autoState + "yeeeeeeeee");
    SmartDashboard.putNumber("Delay", Hardware.delayPot.get(0, 5));


    switch (autoState)
        {
        case INIT:
            setPositionAndLevel();
            Hardware.autoTimer.start();
            autoState = State.DELAY;
            break;
        case DELAY:

            // Delay using the potentiometer, from 0 to 5 seconds
            // once finished, stop the timer and go to the next state
            if (Hardware.autoTimer.get() >= Hardware.delayPot.get(0.0,
                    5.0))
                {
                autoState = State.CHOOSE_PATH;
                Hardware.autoTimer.stop();
                }
            break;

        case CHOOSE_PATH:
            choosePath();
            break;

        case CROSS_AUTOLINE:
            if (crossAutoline() == true)
                {
                autoState = State.FINISH;
                }
            break;

        case DEPOSIT_STRAIGHT_CARGO_HATCH:
            if (depositCargoHatch() == true)
                {
                autoState = State.FINISH;
                }
            break;

        case DEPOSIT_ROCKET_HATCH:
            // System.out.println("rocket hatch");
            usingVision = true;
            if (depositRocketHatch() == true)
                {
                autoState = State.FINISH;
                }
            break;

        case DEPOSIT_SIDE_CARGO_BALL:
            if (depositSideCargoBall() == true)
                {
                autoState = State.FINISH;
                }
            break;

        case BLIND_ROCKET_HATCH:
            usingVision = false;
            if (depositRocketHatch() == true)
                {
                autoState = State.FINISH;
                }

            break;
        case JANKY_DEPOSIT_STRAIGHT:
            usingVision = true;
            usingVisionOnStraight = true;
            if (jankyDepositCargoHatch() == true)
                {
                autoState = State.FINISH;
                }
            break;

        case FINISH:
            driverControl();
            break;

        default:
            autoState = State.FINISH;
            break;
        }

}

// ---------------------------------
// Methods
// ---------------------------------

/**
 *
 */
// choosePath() selects which autonomous path is used, based on the position of
// the auto path switch on the robot. Said switch is a six position switch.
private static void choosePath ()
{
    switch (Hardware.autoSixPosSwitch.getPosition())
        {
        case 0:
            // crossing the autoline
            autoState = State.CROSS_AUTOLINE;
            break;

        case 1:
            autoState = State.DEPOSIT_STRAIGHT_CARGO_HATCH;
            break;

        case 2:
            autoState = State.DEPOSIT_ROCKET_HATCH;
            break;

        case 3:
            autoState = State.DEPOSIT_SIDE_CARGO_BALL;
            break;

        case 4:
            autoState = State.BLIND_ROCKET_HATCH;
            break;

        case 5:
            autoState = State.JANKY_DEPOSIT_STRAIGHT;
            break;
        default:
            // end of autonomous
            System.out.println(
                    "WE DONE MESSED UP THE SIX POSITION SWITCH");
            autoState = State.FINISH;
            break;
        }

}

/**
 * sets the enums of both the autoPosition and autoLevel based on the
 * coresponding switches
 *
 */

// setPositionAndLevel gets the level we are descending from, and which part of
// the HAB we are in. (Level 1 or 2; left, right or centre respectively.) This
// info is based on the position of the autoLevel and autoPosition switches.
private static void setPositionAndLevel ()
{
    // getting the position; left, right, or centre
    if (Hardware.autoCenterSwitch.getPosition() == LEFT)
        {
        autoPosition = Position.LEFT;
        // System.out.println("position and level set");
        }
    else
        if (Hardware.autoCenterSwitch.getPosition() == RIGHT)
            {
            autoPosition = Position.RIGHT;
            }
        else
            if (Hardware.autoCenterSwitch.isOn() == true)
                {
                autoPosition = Position.CENTER;
                }

    // sets the autoLevel enum to the correct level, or disabled, based on the
    // state of the levelSwitch
    // hardcoded, change when switch is fixed
    // autoLevel = Level.LEVEL_TWO;
    // System.out.println("woooooooooooo");
    // getting the level.
    if (Hardware.levelOneSwitch.isOn() == true)
        {
        autoLevel = Level.LEVEL_ONE;
        }
    else
        if (Hardware.levelTwoSwitch.isOn() == true)
            {
            autoLevel = Level.LEVEL_TWO;
            }


}

// =====================================================================
// Path Methods
// =====================================================================

// crosses the autoline during autonomous and nothing else. Has a path that
// descends from level 2 as well.
public static enum CrossAutoState
    {
    INIT, L2_DESCEND, ONWARD, BRAKE, FINISH
    }

private static CrossAutoState cross = CrossAutoState.INIT;

private static boolean crossAutoline ()
{
    // System.out.println("CrossAutoState = " + cross);
    switch (cross)
        {
        case INIT:
            // initial state for crossing the autoline
            Hardware.leftFrontDriveEncoder.reset();
            Hardware.rightFrontDriveEncoder.reset();
            // System.out.println("Initialising");
            switch (autoPosition)
                {
                case LEFT:
                    distanceToCrossAutoline = 60;
                    break;
                case CENTER:
                    distanceToCrossAutoline = 90;
                    break;
                case RIGHT:
                    distanceToCrossAutoline = 120;
                    break;
                case NULL:
                    break;
                }
            Hardware.gyro.reset();
            cross = CrossAutoState.L2_DESCEND;
            break;

        case L2_DESCEND:

            // only run if going off of level 2
            // System.out.println("Manoevering, clear the datum!");
            if (autoLevel == Level.LEVEL_TWO)
                {
                if (descendFromLevelTwo(usingAlignByWall,
                        goingBackwards, turningAroundAfter) == true)
                    {
                    cross = CrossAutoState.ONWARD;
                    }
                // Hardware.leftFrontDriveEncoder.reset();
                // Hardware.rightFrontDriveEncoder.reset();
                }
            else
                if (autoLevel == Level.LEVEL_ONE)
                    {
                    cross = CrossAutoState.ONWARD;
                    }
            break;

        case ONWARD:
            // a.k.a. drive straight
            // 17 February 2019
            // Hardware.depositGamePiece.prepToDepositHatch();
            // Hardware.manipulator.printDeployDebugInfo();
            // System.out.println("*distant screaming*");
            Hardware.gyro.reset();
            if (Hardware.drive.driveStraightInches(
                    distanceToCrossAutoline
                            - Hardware.drive.getBrakeStoppingDistance(),
                    LEAVE_LEVEL_ONE_SPEED, ACCELERATION_TIME,
                    true) == true)
                {

                cross = CrossAutoState.BRAKE;
                }
            break;

        case BRAKE:
            // don't ever use drive.stop to break - leaves you coasting for
            // another foot and a half.
            // System.out.println("SLAM THE BRAKES!");
            if ((Hardware.drive
                    .brake(BrakeType.AFTER_DRIVE)) == true)
                {
                // cross = CrossAutoState.PosArmRL1;
                cross = CrossAutoState.FINISH;
                }
            break;

        case FINISH:
            return true;
        // end of crossing the autoline
        // TODO figure out how to make the below line WORK
        // HardwareInterfaces.Transmission.TransmissionBase.stop();
        // System.out.println(
        // "Finite Incatem");

        }
    return false;
}

// depositCargoHatch() goes straight ahead to the front end of the cargo ship.
private static enum DepositCargoHatchState
    {
    INIT, DESCEND, STRAIGHT_DEPOSIT_DRIVE_1, STRAIGHT_DEPOSIT_TURN_1_RIGHT_SIDE, STRAIGHT_DEPOSIT_TURN_1_LEFT_SIDE, STRAIGHT_DEPOSIT_DRIVE_2, STRAIGHT_DEPOSIT_TURN_2_RIGHT_SIDE, STRAIGHT_DEPOSIT_TURN_2_LEFT_SIDE, STRAIGHT_DEPOSIT_DRIVE_3, STRAIGHT_DEPOSIT_ALIGN_TO_CARGO, STRAIGHT_DEPOSIT_DEPOSIT_HATCH, DRIVE_2_CENTER, FINISHED
    }

private static DepositCargoHatchState depositCargoHatchState = DepositCargoHatchState.INIT;

private static boolean depositCargoHatch ()
{
    System.out.println(
            "depositCargoHatchState:" + depositCargoHatchState);
    switch (depositCargoHatchState)
        {
        case INIT:

            // if on level two decend, turn base on start position
            if (autoLevel == Level.LEVEL_TWO)
                {
                depositCargoHatchState = DepositCargoHatchState.DESCEND;
                }
            else
                {
                depositCargoHatchState = DepositCargoHatchState.STRAIGHT_DEPOSIT_DRIVE_1;
                }
            break;
        case DESCEND: // driving off of level 2
        // if (descendFromLevelTwo(usingAlignByWall))
            {
            // turn based on start position
            depositCargoHatchState = DepositCargoHatchState.STRAIGHT_DEPOSIT_DRIVE_1;
            }
            break;

        case STRAIGHT_DEPOSIT_DRIVE_1: // first leg forward
            if (Hardware.drive.driveStraightInches(
                    distanceToCrossAutoline, LEAVE_LEVEL_ONE_SPEED,
                    ACCELERATION_TIME,
                    USING_GYRO))
                {
                if (autoPosition == Position.RIGHT)
                    {
                    depositCargoHatchState = DepositCargoHatchState.STRAIGHT_DEPOSIT_TURN_1_RIGHT_SIDE;
                    }
                else
                    if (autoPosition == Position.LEFT)
                        {
                        depositCargoHatchState = DepositCargoHatchState.STRAIGHT_DEPOSIT_TURN_1_LEFT_SIDE;
                        }
                    else
                        {
                        if (!usingVisionOnStraight)
                            {
                            depositCargoHatchState = DepositCargoHatchState.DRIVE_2_CENTER;
                            }
                        else
                            {
                            depositCargoHatchState = DepositCargoHatchState.STRAIGHT_DEPOSIT_ALIGN_TO_CARGO;
                            }
                        }
                }
            break;
        case DRIVE_2_CENTER:
            Hardware.depositGamePiece.prepToDepositHatch();
            if (Hardware.drive.driveStraightInches(
                    DISTANCE_TO_SHIP_CENTER, DRIVE_SPEED,
                    ACCELERATION_TIME, USING_GYRO))
                {
                depositCargoHatchState = DepositCargoHatchState.STRAIGHT_DEPOSIT_DEPOSIT_HATCH;
                }
            break;
        case STRAIGHT_DEPOSIT_TURN_1_RIGHT_SIDE: // first turn, when
                                                 // autoPosition is set to RIGHT
            if (Hardware.drive.turnDegrees(TURN_LEFT90,
                    TURN_SPEED,
                    ACCELERATION_TIME, USING_GYRO))
                {
                depositCargoHatchState = DepositCargoHatchState.STRAIGHT_DEPOSIT_DRIVE_2;

                }
            break;
        case STRAIGHT_DEPOSIT_TURN_1_LEFT_SIDE: // first turn, when autoPosition
                                                // is set to LEFT
            if (Hardware.drive.turnDegrees(TURN_RIGHT90,
                    TURN_SPEED,
                    ACCELERATION_TIME, USING_GYRO))
                {
                depositCargoHatchState = DepositCargoHatchState.STRAIGHT_DEPOSIT_DRIVE_2;
                }
            break;
        case STRAIGHT_DEPOSIT_DRIVE_2:
            if (Hardware.drive.driveStraightInches(
                    DRIVE_STRAIGHT_DEPOSIT_1, DRIVE_SPEED,
                    ACCELERATION_TIME,
                    USING_GYRO))
                {
                if (autoPosition == Position.RIGHT)
                    {
                    depositCargoHatchState = DepositCargoHatchState.STRAIGHT_DEPOSIT_TURN_2_RIGHT_SIDE;
                    }
                else
                    {
                    depositCargoHatchState = DepositCargoHatchState.STRAIGHT_DEPOSIT_TURN_2_LEFT_SIDE;
                    }
                }
            break;
        case STRAIGHT_DEPOSIT_TURN_2_RIGHT_SIDE:
            if (Hardware.drive.turnDegrees(TURN_RIGHT90,
                    TURN_SPEED,
                    ACCELERATION_TIME, USING_GYRO))
                {
                if (usingVisionOnStraight == true)
                    {
                    depositCargoHatchState = DepositCargoHatchState.STRAIGHT_DEPOSIT_ALIGN_TO_CARGO;
                    }
                else
                    depositCargoHatchState = DepositCargoHatchState.STRAIGHT_DEPOSIT_DRIVE_3;
                }
            break;
        case STRAIGHT_DEPOSIT_TURN_2_LEFT_SIDE:
            if (Hardware.drive.turnDegrees(TURN_LEFT90,
                    TURN_SPEED,
                    ACCELERATION_TIME, USING_GYRO))
                {
                if (usingVisionOnStraight == true)
                    {
                    depositCargoHatchState = DepositCargoHatchState.STRAIGHT_DEPOSIT_ALIGN_TO_CARGO;
                    }
                else
                    depositCargoHatchState = DepositCargoHatchState.STRAIGHT_DEPOSIT_DRIVE_3;
                }
            break;
        case STRAIGHT_DEPOSIT_DRIVE_3:

            Hardware.depositGamePiece.prepToDepositHatch();
            if (Hardware.drive.driveStraightInches(
                    DRIVE_STRAIGHT_DEPOSIT_2, DRIVE_SPEED,
                    ACCELERATION_TIME,
                    USING_GYRO)
                    || Hardware.frontUltraSonic
                            .getDistanceFromNearestBumper() < 20)
                {

                depositCargoHatchState = DepositCargoHatchState.STRAIGHT_DEPOSIT_DEPOSIT_HATCH;
                }


            break;
        case STRAIGHT_DEPOSIT_ALIGN_TO_CARGO:
            // System.out.println(
            // "Ultrasosnic" + Hardware.frontUltraSonic
            // .getDistanceFromNearestBumper());
            Hardware.depositGamePiece.prepToDepositHatch();
            // maybe align with vision
            if (Hardware.driveWithCamera
                    .driveToTarget(DRIVE_WITH_CAMERA_SPEED, false))
                {
                depositCargoHatchState = DepositCargoHatchState.STRAIGHT_DEPOSIT_DEPOSIT_HATCH;
                }



            break;
        case STRAIGHT_DEPOSIT_DEPOSIT_HATCH:
            // System.out.println("Deposit");
            if (Hardware.depositGamePiece.depositHatch(true))
                {
                depositCargoHatchState = DepositCargoHatchState.FINISHED;
                }
            break;
        case FINISHED:
            return true;

        }
    return false;
}

// depositRocketHatch goes to the rocket to put on a hatch. The ROCKET, not the
// CARGO SHIP
private static enum RocketHatchState
    {
    STANDBY, DESCEND, DRIVE_FORWARD_TO_TURN, BREAKIE_AFTER_DRIVIE, TURN_TOWARDS_FIELD_WALL, DRIVE_TOWARDS_FIELD_WALL, DELAY_BEFORE_TURN_ALONG_FIELD_WALL, TURN_ALONG_FIELD_WALL, ALIGN_PERPENDICULAR_TO_TAPE, DRIVE_TO_ROCKET_TAPE, ALIGN_TO_ROCKET, PREP_TO_DEPOSIT_HATCH, DEPOSIT_HATCH, FINISH, DRIVE_BY_CAMERA
    }

private static RocketHatchState rocketHatchState = RocketHatchState.STANDBY;

// states for the camera nested switch statement
private static enum DriveWithCameraStates
    {
    INIT, DRIVE, FIND_SIDE, TURN_RIGHT, TURN_LEFT, ALIGN
    }

private static DriveWithCameraStates driveWithCameraStates = DriveWithCameraStates.INIT;

private static boolean depositRocketHatch ()
{
    Hardware.drive.setBrakeDeadband(20, BrakeType.AFTER_DRIVE);
    // System.out.println(rocketHatchState);
    if (rocketHatchState == RocketHatchState.STANDBY)
        {
        rocketHatchState = RocketHatchState.DESCEND;
        }
    switch (rocketHatchState)
        {
        case STANDBY:

            break;

        case DESCEND:

            if (autoLevel == Level.LEVEL_TWO)
                {
                // System.out.println("AAAAAAAAHHHHHHHHHHHHHHHHHHHH");
                if (descendFromLevelTwo(usingAlignByWall,
                        goingBackwards, turningAroundAfter) == true)
                    {
                    if (usingVision == true)
                        {
                        System.out.println(
                                "WERE ACTUALLY TRYING TO DRIVE BY CAMERA");
                        rocketHatchState = RocketHatchState.DRIVE_BY_CAMERA;
                        }
                    else
                        {
                        // Hardware.axisCamera.setRelayValue(Value.kOff);
                        autoTimer.reset();
                        autoTimer.start();
                        rocketHatchState = RocketHatchState.BREAKIE_AFTER_DRIVIE;
                        }
                    }
                }
            else
                if (usingVision == true)
                    {
                    rocketHatchState = RocketHatchState.DRIVE_BY_CAMERA;
                    }
                else
                    {
                    // Hardware.axisCamera.setRelayValue(Value.kOff);
                    autoTimer.reset();
                    autoTimer.start();
                    rocketHatchState = RocketHatchState.DRIVE_FORWARD_TO_TURN;
                    }
            break;
        // TODO @ANE
        // =================================================================
        // DRIVE BY NONVISION this is where the dumb kids code
        // =================================================================
        case DRIVE_FORWARD_TO_TURN:
            if (Hardware.drive.driveStraightInches(
                    DISTANCE_TO_DRIVE_TO_FIRST_TURN_ROCKET
                            - Hardware.drive.getBrakeStoppingDistance(),
                    LEAVE_LEVEL_ONE_SPEED, .3
                    /* ACCELERATION_TIME */,
                    true) == true)
                {
                // Hardware.drive.resetEncoders();
                rocketHatchState = RocketHatchState.BREAKIE_AFTER_DRIVIE;
                }
            else
                {
                // System.out.println("DRIVE STRAIGHT POWER : "
                // + Hardware.leftFrontCANMotor.get() + " "
                // + Hardware.rightFrontCANMotor.get());
                }
            break;

        case BREAKIE_AFTER_DRIVIE:

            // System.out.println(
            // "left : " + Hardware.leftFrontCANMotor.get());
            // System.out.println(
            // "right : " + Hardware.rightFrontCANMotor.get());
            if (Hardware.drive.brake(BrakeType.AFTER_DRIVE) == true)
                {
                rocketHatchState = RocketHatchState.TURN_TOWARDS_FIELD_WALL;
                }

            break;// ironic I know

        case TURN_TOWARDS_FIELD_WALL:
            // turn for if we are on the right side of the field
            if (autoPosition == Position.RIGHT
                    && Hardware.drive.turnDegrees(TURN_RIGHT90,
                            TURN_SPEED, ACCELERATION_TIME, true))
                {
                rocketHatchState = RocketHatchState.DRIVE_TOWARDS_FIELD_WALL;
                }
            // turn for if we are on the left side of th field
            else
                if (autoPosition == Position.LEFT
                        && Hardware.drive.turnDegrees(TURN_LEFT90,
                                TURN_SPEED, ACCELERATION_TIME, true))
                    {
                    rocketHatchState = RocketHatchState.DRIVE_TOWARDS_FIELD_WALL;
                    }
            break;
        case DRIVE_TOWARDS_FIELD_WALL:

            if (Hardware.drive.driveStraightInches(
                    35 - Hardware.drive.getBrakeStoppingDistance(),
                    DRIVE_SPEED,
                    ACCELERATION_TIME, USING_GYRO))
                {

                rocketHatchState = RocketHatchState.DELAY_BEFORE_TURN_ALONG_FIELD_WALL;

                }
            // System.out
            // .println("ultrasonic - " + Hardware.frontUltraSonic
            // .getDistanceFromNearestBumper());
            // if (Hardware.frontUltraSonic
            // .getDistanceFromNearestBumper() > DISTANCE_NEEDED_TO_TURN)
            // {
            // Hardware.drive.driveStraight(DRIVE_SPEED,
            // ACCELERATION_TIME, false);
            // System.out.println("speeeeeedddd : "
            // + Hardware.leftFrontCANMotor.get());
            // System.out.println("RRRRRRRR : "
            // + Hardware.rightFrontCANMotor.get());
            // }
            // else
            // {

            // Hardware.drive.stop();
            // Hardware.gyro.reset();
            // autoTimer.reset();
            // autoTimer.start();
            // rocketHatchState =
            // RocketHatchState.DELAY_BEFORE_TURN_ALONG_FIELD_WALL;
            // }

            break;

        case DELAY_BEFORE_TURN_ALONG_FIELD_WALL:
            if (Hardware.drive.brake(BrakeType.AFTER_DRIVE) == true)
            /* && autoTimer.get() >= TIME_TO_DELAY_B4_TURN )_ */
                {
                rocketHatchState = RocketHatchState.TURN_ALONG_FIELD_WALL;
                }
            break;


        case TURN_ALONG_FIELD_WALL:
            // System.out.println(" gyro " + Hardware.gyro.getAngle());
            // turn for if we are on the right side of the field
            if (autoPosition == Position.RIGHT
                    && Hardware.drive
                            .turnDegrees(
                                    -DEGREES_TO_TURN_ALONG_FIELD_WALL,
                                    TURN_SPEED, ACCELERATION_TIME,
                                    true))
                {
                // currently bypasses the align state
                rocketHatchState = RocketHatchState.ALIGN_PERPENDICULAR_TO_TAPE;
                }
            // turn for if we are on the left side of th field
            else
                if (autoPosition == Position.LEFT
                        && Hardware.drive.turnDegrees(
                                DEGREES_TO_TURN_ALONG_FIELD_WALL,
                                TURN_SPEED, ACCELERATION_TIME, true))
                    {
                    // currently bypasses the align state
                    rocketHatchState = RocketHatchState.ALIGN_PERPENDICULAR_TO_TAPE;
                    }
            break;

        case ALIGN_PERPENDICULAR_TO_TAPE:
            // if (alignPerpendicularToTape() == true)
            // {
            // Hardware.lift
            // .setLiftPosition(Forklift.LOWER_ROCKET_HATCH);
            rocketHatchState = RocketHatchState.DRIVE_TO_ROCKET_TAPE;
            // }

            break;

        case DRIVE_TO_ROCKET_TAPE:

            if (Hardware.drive.driveStraightInches(80, DRIVE_SPEED,
                    ACCELERATION_TIME, USING_GYRO))
                {
                rocketHatchState = RocketHatchState.FINISH;
                }
            // if (redlight1 == true || redlight2 == true || redlight3 ==
            // true
            // ||
            // redlight4 == true ||redlight5 == true)

            // if (Hardware.frontUltraSonic
            // .getDistanceFromNearestBumper() < DISTANCE_NEEDED_TO_TURN)
            // {
            // Hardware.drive.stop();
            // rocketHatchState = RocketHatchState.ALIGN_TO_ROCKET;
            // }
            // else
            // {
            // // Hardware.lift
            // // .setLiftPosition(Forklift.LOWER_ROCKET_HATCH);
            // Hardware.drive.driveStraight(DRIVE_SPEED,
            // ACCELERATION_TIME,
            // false);
            // }

            break;

        // =================================================================
        // DRIVE BY VISION CODE this is where the dumb kidz code
        // =================================================================

        case DRIVE_BY_CAMERA:


            switch (driveWithCameraStates)
                {
                case INIT:
                    // System.out.println("the dumb kidz code");

                    driveWithCameraStates = DriveWithCameraStates.DRIVE;
                    break;
                case DRIVE:
                    if (Hardware.drive.driveStraightInches(
                            DISTANCE_TO_CROSS_AUTOLINE_CAMERA,
                            LEAVE_LEVEL_ONE_SPEED,
                            ACCELERATION_TIME, USING_GYRO))
                        {


                        if (autoPosition == Position.RIGHT)
                            {
                            driveWithCameraStates = DriveWithCameraStates.TURN_RIGHT;
                            }
                        else
                            if (autoPosition == Position.LEFT)
                                {
                                driveWithCameraStates = DriveWithCameraStates.TURN_LEFT;
                                }
                            else
                                {
                                System.out.println(
                                        "the code did not receive the side switch value");
                                driveWithCameraStates = DriveWithCameraStates.FIND_SIDE;
                                }


                        }
                    break;
                case FIND_SIDE:
                    // System.out.println("find side: "
                    // + Hardware.driveWithCamera.getTargetSide());
                    if (Hardware.driveWithCamera
                            .getTargetSide() == Side.RIGHT)
                        {
                        driveWithCameraStates = DriveWithCameraStates.TURN_LEFT;
                        }
                    else
                        if (Hardware.driveWithCamera
                                .getTargetSide() == Side.LEFT)
                            {
                            driveWithCameraStates = DriveWithCameraStates.TURN_RIGHT;
                            }
                        else
                            {
                            if (Hardware.drive.driveStraightInches(
                                    DISTANCE_TO_CROSS_AUTOLINE_CAMERA,
                                    .4,
                                    ACCELERATION_TIME, USING_GYRO))
                                rocketHatchState = RocketHatchState.FINISH;
                            }
                    break;
                case TURN_RIGHT:
                    // System.out.println("right");[]

                    if (Hardware.drive.turnDegrees(
                            TURN_FOR_CAMERA_DEGREES, CAMERA_TURN_SPEED,
                            CAMERA_ACCELERATION, USING_GYRO))
                        {
                        driveWithCameraStates = DriveWithCameraStates.ALIGN;
                        }
                    break;
                case TURN_LEFT:
                    // System.out.println(
                    // "gyro degrees" + Hardware.gyro.getAngle());
                    // System.out.println("left");
                    if (Hardware.drive.turnDegrees(
                            -TURN_FOR_CAMERA_DEGREES, CAMERA_TURN_SPEED,
                            CAMERA_ACCELERATION, USING_GYRO))
                        {
                        driveWithCameraStates = DriveWithCameraStates.ALIGN;
                        }
                    break;
                case ALIGN:
                    // Hardware.axisCamera.saveImage(ImageType.PROCESSED);
                    // Hardware.axisCamera.saveImage(ImageType.RAW);
                    // align with the camera
                    if (Hardware.driveWithCamera
                            .driveToTarget(DRIVE_WITH_CAMERA_SPEED,
                                    false))
                        {


                        rocketHatchState = RocketHatchState.DEPOSIT_HATCH;
                        }

                    Hardware.depositGamePiece.prepToDepositHatch();


                    break;
                }

            break;
        // =================================================================
        // END OF SEPCIALIZED DRIVING CODE
        // =================================================================
        case ALIGN_TO_ROCKET:
        // if (alignParallelToTape() == true)
            {
            rocketHatchState = RocketHatchState.PREP_TO_DEPOSIT_HATCH;
            }
            break;

        case PREP_TO_DEPOSIT_HATCH:
            if (/* prepToDeposit() == */ true)
                {
                rocketHatchState = RocketHatchState.DEPOSIT_HATCH;
                }
            break;


        case DEPOSIT_HATCH:
            if (Hardware.depositGamePiece.depositHatch(true))
                {
                rocketHatchState = RocketHatchState.FINISH;
                }
            break;
        case FINISH:

            return true;
        default:
            break;
        }

    return false;
}


// depositSideCargoHatch() deposits on the SIDE of the cargo ship.
/**
 * Enum for representing the states used in the depositSideCargoHatch path
 */
private static enum SideCargoBallState
    {
    INIT, LEAVE_LEVEL_2, LEAVE_LEVEL_1, TURN_TOWARDS_ROCKET, DRIVE_TOWARDS_ROCKET, TURN_PARALLEL_TO_ROCKET, DRIVE_PARALLEL_TO_ROCKET, TURN_TOWARDS_CARGO_SHIP, PREP_MANIPULATOR_AND_LIFT, VISION_DRIVE_TOWARDS_CARGO_SHIP, DEPOSIT_BALL, BACK_UP, FINISHED
    } // and we need to deploy the manipulator somewhere in here

/**
 * Variable for keeping track of the state used in the depositSideCargoBall
 * path
 */
private static SideCargoBallState sideCargoBallState = SideCargoBallState.INIT;// TODO

private static boolean depositSideCargoBall ()
// welcome to the Gates of Hell

{
    // System.out.println("sideState" + sideCargoBallState);
    // // System.out.println("level one?" + Hardware.levelOneSwitch.isOn());
    // System.out.println(
    // "yay the robot is doing something orrect for once");
    // System.out.println("auto disable switch: "
    // + Hardware.autoDisableSwitch.isOn());
    // TODO
    switch (sideCargoBallState)
        {
        case INIT:
            if (autoLevel == Level.LEVEL_TWO)
                sideCargoBallState = SideCargoBallState.LEAVE_LEVEL_2;
            else
                sideCargoBallState = SideCargoBallState.LEAVE_LEVEL_1;
            break;
        case LEAVE_LEVEL_2:
            if (descendFromLevelTwo(usingAlignByWall,
                    goingBackwards, turningAroundAfter) == true)
                {
                sideCargoBallState = SideCargoBallState.LEAVE_LEVEL_1;
                }
            break;
        case LEAVE_LEVEL_1:
            if (Hardware.drive.driveStraightInches(
                    DISTANCE_TO_CROSS_AUTOLINE_CAMERA,
                    LEAVE_LEVEL_ONE_SPEED,
                    ACCELERATION_TIME, USING_GYRO) == true)
                {
                sideCargoBallState = SideCargoBallState.TURN_TOWARDS_ROCKET;
                }
            break;

        case TURN_TOWARDS_ROCKET:
            // System.out.println("gyro " + Hardware.gyro.getAngle());
            if (autoPosition == Position.LEFT)
                {
                System.out.println("left turn towards rocket");
                if (Hardware.drive.turnDegrees(-TURN_FOR_CAMERA_DEGREES,
                        TURN_SPEED,
                        ACCELERATION_TIME, USING_GYRO) == true)
                    {
                    sideCargoBallState = SideCargoBallState.DRIVE_TOWARDS_ROCKET;
                    break;
                    }
                }
            else
                if (autoPosition == Position.RIGHT)
                    {
                    System.out.println("right turn towards rocket");
                    if (Hardware.drive.turnDegrees(
                            TURN_FOR_CAMERA_DEGREES,
                            TURN_SPEED,
                            ACCELERATION_TIME, USING_GYRO) == true)
                        {
                        sideCargoBallState = SideCargoBallState.DRIVE_TOWARDS_ROCKET;
                        break;
                        }
                    }
            break;
        case DRIVE_TOWARDS_ROCKET:
            // System.out.println(
            // "left enc " + Hardware.leftFrontDriveEncoder
            // .getDistance());
            // System.out.println("right enc "
            // + Hardware.rightFrontDriveEncoder.getDistance());
            if (Hardware.drive.driveStraightInches(
                    DISTANCE_TO_DRIVE_TOWARDS_ROCKET_WITH_SIDE_BALL,
                    DRIVE_SPEED, ACCELERATION_TIME, USING_GYRO))
                {
                sideCargoBallState = SideCargoBallState.TURN_PARALLEL_TO_ROCKET;
                }
            break;

        case TURN_PARALLEL_TO_ROCKET:
            // System.out.println("gyro " + Hardware.gyro.getAngle());
            if (autoPosition == Position.LEFT)
                {
                System.out
                        .println("right turn towards parallel rocket");
                if (Hardware.drive.turnDegrees(
                        TURN_FOR_CAMERA_DEGREES, CAMERA_TURN_SPEED,
                        CAMERA_ACCELERATION, USING_GYRO) == true)
                    {
                    sideCargoBallState = SideCargoBallState.DRIVE_PARALLEL_TO_ROCKET;
                    break;
                    }
                }
            else
                if (autoPosition == Position.RIGHT)
                    {
                    System.out.println(
                            "left turn parallel towards rocket");
                    if (Hardware.drive.turnDegrees(
                            -TURN_FOR_CAMERA_DEGREES, CAMERA_TURN_SPEED,
                            CAMERA_ACCELERATION, USING_GYRO) == true)
                        {
                        sideCargoBallState = SideCargoBallState.DRIVE_PARALLEL_TO_ROCKET;
                        break;
                        }
                    }
            break;
        case DRIVE_PARALLEL_TO_ROCKET:
            // System.out.println(
            // "left enc " + Hardware.leftFrontDriveEncoder
            // .getDistance());
            // System.out.println("right enc "
            // + Hardware.rightFrontDriveEncoder.getDistance());
            if (Hardware.drive.driveStraightInches(
                    DISTANCE_TO_DRIVE_PARALLEL_TO_ROCKET, DRIVE_SPEED,
                    ACCELERATION_TIME, USING_GYRO) == true)
                {
                sideCargoBallState = SideCargoBallState.TURN_TOWARDS_CARGO_SHIP;
                }
            break;

        case TURN_TOWARDS_CARGO_SHIP:
            // System.out.println("gyro " + Hardware.gyro.getAngle());
            if (autoPosition == Position.LEFT)
                {

                if (Hardware.drive.turnDegrees(TURN_FOR_CAMERA_DEGREES,
                        TURN_SPEED,
                        ACCELERATION_TIME, USING_GYRO) == true)
                    {
                    sideCargoBallState = SideCargoBallState.PREP_MANIPULATOR_AND_LIFT;
                    break;
                    }
                }
            else
                if (autoPosition == Position.RIGHT)
                    {
                    if (Hardware.drive.turnDegrees(
                            -TURN_FOR_CAMERA_DEGREES,
                            TURN_SPEED,
                            ACCELERATION_TIME, USING_GYRO) == true)
                        {
                        sideCargoBallState = SideCargoBallState.PREP_MANIPULATOR_AND_LIFT;
                        break;
                        }
                    }

            break;

        case PREP_MANIPULATOR_AND_LIFT:

            // if (Hardware.lift.setLiftPosition(CARGO_HEIGHT_FOR_CARGO_SHIP) ==
            // true)
            // {
            // prepBoolean = true;
            // }
            // if (prepBoolean == true &&
            // Hardware.manipulator.moveArmToPositionPrecise(25.0) == true)
            // {
            // sideCargoBallState =
            // SideCargoBallState.VISION_DRIVE_TOWARDS_CARGO_SHIP;
            // }
            sideCargoBallState = SideCargoBallState.VISION_DRIVE_TOWARDS_CARGO_SHIP;
            break;

        case VISION_DRIVE_TOWARDS_CARGO_SHIP:
            if (Hardware.driveWithCamera
                    .driveToTargetClose(.2, true))// TODO constant nad vision
                                                  // function
                {
                sideCargoBallState = SideCargoBallState.DEPOSIT_BALL;
                // //TODO

                }
            break;
        case DEPOSIT_BALL:

            if (Hardware.depositGamePiece.depositCargo(true))
                {
                sideCargoBallState = SideCargoBallState.FINISHED;
                }
            break;

        case BACK_UP:
            if (Hardware.drive.driveStraightInches(DISTANCE_TO_BACK_UP,
                    -DRIVE_SPEED, ACCELERATION_TIME,
                    USING_GYRO) == true)
                {
                sideCargoBallState = SideCargoBallState.FINISHED;
                }
            break;
        default:
        case FINISHED:
            Hardware.drive.stop();
            break;
        }
    return false;
} // end depositSideCargoHatch()

private static void driverControl ()
{
    // if (Hardware.leftDriver.getRawButton(5) == true)
    // {
    // Hardware.leftFrontCANMotor.set(.5);
    // } else
    // {
    // Hardware.leftFrontCANMotor.set(0);
    // }
    Teleop.periodic();

} // end driverControl()

public static enum DescentState
    {
    STANDBY, INIT, DRIVE_FAST, DELAY_INIT_AFTER_DRIVE_FAST, DELAY_AFTER_DRIVE_FAST, LANDING_SETUP, BACKWARDS_TIMER_INIT, DRIVE_BACKWARDS_TO_ALIGN, DELAY_INIT_B4_FINISH, DELAY_B4_FINISH, PREP_TO_TURN_180, TURN_180, FINISH
    }


// Cole wrote this method, January 2019



public static boolean driveOffStraightLevel1 ()
{
    return driveOffStraightLevel1(Hardware.leftBackIR,
            Hardware.rightBackIR, Hardware.drive);
}

// TODO this needs to be tested


public static boolean driveOffStraightLevel1 (LightSensor backIR1,
        LightSensor backIR2, Drive drive)
{
    double driveSpeed = .7; // arbitrary number to be tested
    // for now, we are not using the gyro
    boolean usingGyro = USING_GYRO_FOR_DRIVE_STARIGHT;

    if (backIR1.isOn() == true || backIR2.isOn() == true)
        {
        drive.driveStraight(driveSpeed, ACCELERATION_TIME, usingGyro);
        return false;
        }
    else
        {
        drive.stop();
        return true;
        } // else
} // end driveOffStraightLevel1()


public static DescentState descentState = DescentState.STANDBY;

// TODO placeholder
public static boolean reorientAfterLevel2Drop ()
{

    return false;
}

public static boolean descendFromLevelTwo (boolean usingAlignByWall,
        boolean goingBackwards, boolean turn180AtEnd)
{

    if (descentState == DescentState.STANDBY)
        {
        descentState = DescentState.INIT;
        } // if


    // System.out.println(descentState);
    // System.out.println(descentTimer.get());
    switch (descentState)
        {

        case STANDBY:

            break;
        case INIT:
            descentTimer.reset();
            descentTimer.start();
            descentState = DescentState.DRIVE_FAST;
            break;

        case DRIVE_FAST:
            if (goingBackwards == false)
                {
                if (descentTimer.get() >= TIME_TO_DRIVE_OFF_PLATFORM)
                    {
                    Hardware.drive.stop();
                    descentTimer.stop();
                    descentState = DescentState.DELAY_INIT_AFTER_DRIVE_FAST;
                    }
                else
                    {
                    Hardware.drive.driveStraight(
                            SPEED_TO_DRIVE_OFF_PLATFORM,
                            0.0,
                            true);
                    }
                break;
                }
            else
                if (goingBackwards == true)
                    {
                    if (descentTimer
                            .get() >= TIME_TO_DRIVE_OFF_PLATFORM_BACKWARDS)
                        {
                        Hardware.drive.stop();
                        descentTimer.stop();
                        descentState = DescentState.DELAY_INIT_AFTER_DRIVE_FAST;
                        }
                    else
                        {
                        Hardware.drive.driveStraight(
                                REVERSE_SPEED_TO_DRIVE_OFF_PLATFORM,
                                0.0,
                                true);
                        }
                    break;
                    }
            break;

        case DELAY_INIT_AFTER_DRIVE_FAST:
            descentTimer.reset();
            descentTimer.start();
            descentState = DescentState.DELAY_AFTER_DRIVE_FAST;
            break;

        case DELAY_AFTER_DRIVE_FAST:
            if (descentTimer.get() > TIME_TO_DELAY_AFTER_DRIVE_FAST)
                {
                descentState = DescentState.LANDING_SETUP;
                }
            break;

        case LANDING_SETUP:

            // if (Hardware.testRedLight.isOn() == true
            // && usingAlignByWall == true)
            // {
            // descentState = DescentState.BACKWARDS_TIMER_INIT;
            // } else if (Hardware.testRedLight.isOn()
            // && usingAlignByWall == false)
            // {
            // descentState = DescentState.FINISH;
            // }
            descentState = DescentState.BACKWARDS_TIMER_INIT;
            break;

        case BACKWARDS_TIMER_INIT:
            descentTimer.reset();
            descentTimer.start();
            descentState = DescentState.DRIVE_BACKWARDS_TO_ALIGN;
            break;

        case DRIVE_BACKWARDS_TO_ALIGN:
            if (goingBackwards == false)
                {
                if (descentTimer
                        .get() >= TIME_TO_DRIVE_BACKWARDS_TO_ALIGN)
                    {
                    descentTimer.stop();
                    Hardware.drive.stop();
                    descentState = DescentState.DELAY_INIT_B4_FINISH;
                    }
                else
                    {
                    Hardware.transmission.driveRaw(
                            DRIVE_BACKWARDS_TO_ALIGN_SPEED,
                            DRIVE_BACKWARDS_TO_ALIGN_SPEED);
                    }
                break;
                }
            else
                if (goingBackwards == true)
                    {
                    if (descentTimer
                            .get() >= TIME_TO_DRIVE_FORWARDS_TO_ALIGN)
                        {
                        descentTimer.stop();
                        Hardware.drive.stop();
                        descentState = DescentState.DELAY_INIT_B4_FINISH;
                        }
                    else
                        {
                        Hardware.transmission.driveRaw(
                                DRIVE_FORWARDS_TO_ALIGN_SPEED,
                                DRIVE_FORWARDS_TO_ALIGN_SPEED);
                        }
                    break;
                    }
        case DELAY_INIT_B4_FINISH:
            descentTimer.reset();
            descentTimer.start();
            descentState = DescentState.DELAY_B4_FINISH;
            break;

        case DELAY_B4_FINISH:
            if (descentTimer.get() > TIME_TO_DELAY_B4_FINISH)
                {
                if (turn180AtEnd == true)
                    {
                    descentState = DescentState.PREP_TO_TURN_180;
                    }
                else
                    {
                    descentState = DescentState.FINISH;
                    }

                }
            break;

        case PREP_TO_TURN_180:
            if (Hardware.drive.driveStraightInches(
                    distanceToCrossAutoline + 20, -DRIVE_SPEED,
                    ACCELERATION_TIME, true) == true)
                {
                descentState = DescentState.TURN_180;
                }
            break;

        case TURN_180:
            if (Hardware.drive.turnDegrees(TURN_180, TURN_SPEED,
                    ACCELERATION_TIME, true))
                {
                descentState = DescentState.FINISH;
                }
            break;

        case FINISH:
            // System.out.println("YEEt! XD");
            return true;

        default:
            break;

        } // end switch
    return false;
} // end descendFromLevelTwo()


private static enum JankyDepositCargoHatchState
    {
    INIT, DESCEND, STRAIGHT_DEPOSIT_DRIVE_1, STRAIGHT_DEPOSIT_TURN_1_RIGHT_SIDE, STRAIGHT_DEPOSIT_TURN_1_LEFT_SIDE, STRAIGHT_DEPOSIT_ALIGN_TO_CARGO, STRAIGHT_DEPOSIT_DEPOSIT_HATCH, FINISHED
    }

private static JankyDepositCargoHatchState jankyDepositCargoHatchState = JankyDepositCargoHatchState.INIT;

private static boolean jankyDepositCargoHatch ()
{
    // System.out.println(
    // "depositCargoHatchState:" + depositCargoHatchState);

    switch (jankyDepositCargoHatchState)
        {
        case INIT:

            // if on level two decend, turn base on start position
            if (autoLevel == Level.LEVEL_TWO)
                {
                jankyDepositCargoHatchState = JankyDepositCargoHatchState.DESCEND;
                }
            else
                {
                jankyDepositCargoHatchState = JankyDepositCargoHatchState.STRAIGHT_DEPOSIT_DRIVE_1;
                }
            break;
        case DESCEND: // driving off of level 2
            if (descendFromLevelTwo(true, false, false))
                {
                // turn based on start position
                jankyDepositCargoHatchState = JankyDepositCargoHatchState.STRAIGHT_DEPOSIT_DRIVE_1;
                }
            break;

        case STRAIGHT_DEPOSIT_DRIVE_1: // first leg forward
            if (Hardware.drive.driveStraightInches(
                    distanceToCrossAutoline, LEAVE_LEVEL_ONE_SPEED,
                    ACCELERATION_TIME,
                    USING_GYRO))
                {
                if (autoPosition == Position.RIGHT)
                    {
                    jankyDepositCargoHatchState = JankyDepositCargoHatchState.STRAIGHT_DEPOSIT_TURN_1_RIGHT_SIDE;
                    }
                else
                    if (autoPosition == Position.LEFT)
                        {
                        jankyDepositCargoHatchState = JankyDepositCargoHatchState.STRAIGHT_DEPOSIT_TURN_1_LEFT_SIDE;
                        }
                    else
                        {

                        jankyDepositCargoHatchState = JankyDepositCargoHatchState.STRAIGHT_DEPOSIT_ALIGN_TO_CARGO;

                        }
                }
            break;

        case STRAIGHT_DEPOSIT_TURN_1_RIGHT_SIDE: // first turn, when
                                                 // autoPosition is set to RIGHT
            if (Hardware.drive.turnDegrees(-30,
                    TURN_SPEED,
                    ACCELERATION_TIME, USING_GYRO))
                {
                // Hardware.manipulator
                // .moveArmToPosition(105);
                jankyDepositCargoHatchState = JankyDepositCargoHatchState.STRAIGHT_DEPOSIT_ALIGN_TO_CARGO;

                }
            break;
        case STRAIGHT_DEPOSIT_TURN_1_LEFT_SIDE: // first turn, when autoPosition
                                                // is set to LEFT
            if (Hardware.drive.turnDegrees(30,
                    TURN_SPEED,
                    ACCELERATION_TIME, USING_GYRO))
                {
                // Hardware.manipulator
                // .moveArmToPosition(105);
                jankyDepositCargoHatchState = JankyDepositCargoHatchState.STRAIGHT_DEPOSIT_ALIGN_TO_CARGO;
                }
            break;

        case STRAIGHT_DEPOSIT_ALIGN_TO_CARGO:
            // System.out.println(
            // "Ultrasosnic" + Hardware.frontUltraSonic
            // .getDistanceFromNearestBumper());

            Hardware.depositGamePiece.prepToDepositHatch();
            // Hardware.depositGamePiece.prepToDepositHatch();
            // maybe align with vision
            if (Hardware.driveWithCamera
                    .driveToTarget(DRIVE_WITH_CAMERA_SPEED, false))
                {
                jankyDepositCargoHatchState = JankyDepositCargoHatchState.STRAIGHT_DEPOSIT_DEPOSIT_HATCH;
                }
            break;
        case STRAIGHT_DEPOSIT_DEPOSIT_HATCH:
            // System.out.println("Deposit");
            // TODO C.R. this is calling a faulty function,
            // but passing in the true should help avoid the
            // faulty section
            if (Hardware.depositGamePiece.depositHatch(true))
                {
                jankyDepositCargoHatchState = JankyDepositCargoHatchState.FINISHED;
                }
            break;
        case FINISHED:
            return true;

        }
    return false;
}

public static void endAutoPath ()
{
    Hardware.driveWithCamera.state = DriveWithCamera.DriveWithCameraState.INIT;
    sideCargoBallState = SideCargoBallState.FINISHED;
    depositCargoHatchState = DepositCargoHatchState.FINISHED;
    rocketHatchState = RocketHatchState.FINISH;
    descentState = DescentState.FINISH;
    cross = CrossAutoState.FINISH;

} // end endAutoPath()


// =========================================================================
// TUNEABLES
// =========================================================================


public static boolean turningAroundAfter = false;

public static boolean goingBackwards = false;

private static boolean usingAlignByWall = true;

// ----------------------------------------------
// use vision for rocket autopath
private static boolean usingVision = true;

// use vision for the put hatch straght auto path
private static boolean usingVisionOnStraight = true;;

private static boolean descendInit = false;

public static Timer descentTimer = new Timer();

public static int distanceToCrossAutoline = 60;
/*
 * ==============================================================
 * Constants
 * ==============================================================
 */




// General constants

public static double DRIVE_SPEED;

public static double TURN_SPEED;

public static double LEAVE_LEVEL_ONE_SPEED;
// turn stuff

public static final double TURN_BY_GYRO_SPEED = .5;

public static final int TURN_RIGHT90 = 90;

public static final int TURN_LEFT90 = -90;

public static final int TURN_180 = 180;

// whether or not, by default, we are using the gyro for driveStraight
// in our autonomous code
public static final boolean USING_GYRO_FOR_DRIVE_STARIGHT = true;

public static final boolean USING_GYRO = true;

public static Timer autoTimer = new Timer();

/**
 * Acceleration time that we generally pass into the drive class's driveStraight
 * function; .6 is the value we used for 2018's robot
 */

public static final double ACCELERATION_TIME = .6;

public static final Relay.Value LEFT = Relay.Value.kForward;

public static final Relay.Value RIGHT = Relay.Value.kReverse;

public static final Relay.Value LEVEL_ONE = Relay.Value.kForward;

public static final Relay.Value LEVEL_TWO = Relay.Value.kReverse;


// cross autoline constants


// straight cargo hatch constants
public static final double DISTANCE_TO_SHIP_CENTER = 100;

// rocket hatch contstants- no vision

public static final double DISTANCE_TO_DRIVE_TO_FIRST_TURN_ROCKET = distanceToCrossAutoline;// 60;

public static final int DISTANCE_NEEDED_TO_TURN = 35;// change @ANE

public static final double TIME_TO_DELAY_B4_TURN = .2;

public static final int DEGREES_TO_TURN_ALONG_FIELD_WALL = 50;



// rocket hatch vision constants
public static final double CAMERA_TURN_SPEED = .5;

public static final double CAMERA_ACCELERATION = .2;


public static final double DRIVE_WITH_CAMERA_SPEED = .3;

public static final int TURN_FOR_CAMERA_DEGREES = 67;


// changed to correct-ish number 2 February 2019
public static final int DISTANCE_TO_CROSS_AUTOLINE_CAMERA = 60;


// public static final int LEFT_DISTANCE_CROSS_AUTOLINE = 60;
// public static final int CENTER_DISTANCE_CROSS_AUTOLINE = 90;
// public static final int RIGHT_DISTANCE_CROSS_AUTOLINE = 120;


// side cargo hatch constants

public static final double DRIVE_STRAIGHT_DEPOSIT_1 = 37;

public static final double DRIVE_STRAIGHT_DEPOSIT_2 = 170;

public static final double DISTANCE_TO_DRIVE_TOWARDS_ROCKET_WITH_SIDE_BALL = 60.0;// TODO

public static final double DISTANCE_TO_DRIVE_PARALLEL_TO_ROCKET = 120.0;

public static final double DISTANCE_TO_BACK_UP = 0.0;

// descent Stuff

public static final double TIME_TO_DELAY_AFTER_DRIVE_FAST = 1;

public static final double TIME_TO_DELAY_B4_FINISH = .1;

public static final double DRIVE_BACKWARDS_TO_ALIGN_SPEED = -.4;

public static final double SPEED_TO_DRIVE_OFF_PLATFORM = .75; // @ANE

// TODO @CR .5?
public static final double TIME_TO_DRIVE_OFF_PLATFORM = .5; // @ANE changed
                                                            // from
                                                            // .4 @ 4/11/18

public static final double TIME_TO_DRIVE_BACKWARDS_TO_ALIGN = .4;

// reverse descent Stuff


public static final double TIME_TO_DRIVE_OFF_PLATFORM_BACKWARDS = .5;

public static final double TIME_TO_DRIVE_FORWARDS_TO_ALIGN = .35;

public static final double REVERSE_SPEED_TO_DRIVE_OFF_PLATFORM = -.75;

public static final double DRIVE_FORWARDS_TO_ALIGN_SPEED = .5;



// Auto INIT stuff

public static final int FIRST_GEAR_NUMBER = 0;

public static final int SECOND_GEAR_NUMBER = 1;

public static final double FIRST_AUTO_GEAR_RATIO_KILROY_XIX = 1.0;

public static final double SECOND_AUTO_GEAR_RATIO_KILROY_XIX = 1.0;

public static final double FIRST_AUTO_GEAR_RATIO_KILROY_XX = 1.0;

public static final double SECOND_AUTO_GEAR_RATIO_KILROY_XX = 1.0;

public static final double KILROY_XIX_DRIVE_SPEED = .6;

public static final double KILROY_XX_DRIVE_SPEED = .4;

public static final double KILROY_XIX_TURN_SPEED = .5;

public static final double KILROY_XX_TURN_SPEED = .7;// .5;

public static final double KILROY_XIX_LEAVE_LEVEL_ONE_SPEED = .25;

public static final double KILROY_XX_LEAVE_LEVEL_ONE_SPEED = .25;




}
