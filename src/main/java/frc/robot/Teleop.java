/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
// ====================================================================
// FILE NAME: Teleop.java (Team 339 - Kilroy)
//
// CREATED ON: Jan 13, 2015
// CREATED BY: Nathanial Lydick
// MODIFIED ON: June 20, 2019
// MODIFIED BY: Ryan McGee
// ABSTRACT:
// This file is where almost all code for Kilroy will be
// written. All of these functions are functions that should
// override methods in the base class (IterativeRobot). The
// functions are as follows:
// -----------------------------------------------------
// Init() - Initialization code for teleop mode
// should go here. Will be called each time the robot enters
// teleop mode.
// -----------------------------------------------------
// Periodic() - Periodic code for teleop mode should
// go here. Will be called periodically at a regular rate while
// the robot is in teleop mode.
// -----------------------------------------------------
//
// ====================================================================
package frc.robot;

import frc.Hardware.Hardware;

/**
 * This class contains all of the user code for the Autonomous part of the
 * match, namely, the Init and Periodic code
 *
 * @author Nathanial Lydick
 * @written Jan 13, 2015
 */
public class Teleop
{

/**
 * User Initialization code for teleop mode should go here. Will be called once
 * when the robot enters teleop mode.
 *
 * @author Nathanial Lydick
 * @written Jan 13, 2015
 */
public static void init ()
{

} // end Init


/**
 * User Periodic code for teleop mode should go here. Will be called
 * periodically at a regular rate while the robot is in teleop mode.
 *
 * @author Nathanial Lydick
 * @written Jan 13, 2015
 */

public static void periodic ()
{
    // =============== AUTOMATED SUBSYSTEMS ===============

    // ================= OPERATOR CONTROLS ================

    // ================== DRIVER CONTROLS =================

    Hardware.drive.drive(Hardware.leftDriver, Hardware.rightDriver);


} // end Periodic()

public static void printStatements()
{
    // ========== INPUTS ==========

    // ---------- DIGITAL ----------

    // Encoder Distances
    // Hardware.telemetry.printToConsole("L. Encoder Dist: " + Hardware.leftEncoder.getDistance());
    // Hardware.telemetry.printToConsole("R. Encoder Dist: " + Hardware.rightEncoder.getDistance());
    
    // Encoder Raw Values
    // Hardware.telemetry.printToConsole("L. Encoder Raw: " + Hardware.leftEncoder.get());
    // Hardware.telemetry.printToConsole("R. Encoder Raw: " + Hardware.rightEncoder.get());

    // Switch Values
    // Hardware.telemetry.printToConsole("Six Pos Sw: " + Hardware.autoSixPosSwitch.getPosition());
    // Hardware.telemetry.printToConsole("Auto Disable Sw: " + Hardware.autoDisableSwitch.isOn());

    // ---------- ANALOG -----------

    // Hardware.telemetry.printToConsole("Gyro: " + Hardware.gyro.getAngle());

    // Hardware.telemetry.printToConsole("Delay Pot: " + Hardware.delayPot.get());

    // ----------- CAN -------------

    // -------- SUBSYSTEMS ---------

    // ---------- OTHER ------------

    // Left Driver
    // Hardware.telemetry.printToConsole("Left Driver X: " + Hardware.leftDriver.getX());
    // Hardware.telemetry.printToConsole("Left Driver Y: " + Hardware.leftDriver.getY());
    // Hardware.telemetry.printToConsole("Left Driver Z: " + Hardware.leftDriver.getZ());

    // Right Driver
    // Hardware.telemetry.printToConsole("Right Driver X: " + Hardware.rightDriver.getX());
    // Hardware.telemetry.printToConsole("Right Driver Y: " + Hardware.rightDriver.getY());
    // Hardware.telemetry.printToConsole("Right Driver Z: " + Hardware.rightDriver.getZ());

    // Left Operator
    // Hardware.telemetry.printToConsole("Left Op X: " + Hardware.leftOperator.getX());
    // Hardware.telemetry.printToConsole("Left Op Y: " + Hardware.leftOperator.getY());
    // Hardware.telemetry.printToConsole("Left Op Z: " + Hardware.leftOperator.getZ());

    // Right Operator
    // Hardware.telemetry.printToConsole("Right Op X: " + Hardware.rightOperator.getX());
    // Hardware.telemetry.printToConsole("Right Op Y: " + Hardware.rightOperator.getY());
    // Hardware.telemetry.printToConsole("Right Op Z: " + Hardware.rightOperator.getZ());

    
    // ========== OUTPUTS ==========

    // ---------- DIGITAL ----------

    // ---------- ANALOG -----------

    // ----------- CAN -------------

    // Motor Percentages
    // Hardware.telemetry.printToConsole("L.R. Motor: " + Hardware.leftRearMotor.get());
    // Hardware.telemetry.printToConsole("R.R. Motor: " + Hardware.rightRearMotor.get());
    // Hardware.telemetry.printToConsole("L.F. Motor: " + Hardware.leftFrontMotor.get());
    // Hardware.telemetry.printToConsole("R.F. Motor: " + Hardware.rightFrontMotor.get());


    // -------- SUBSYSTEMS ---------

    // ---------- OTHER ------------

}



} // end class
