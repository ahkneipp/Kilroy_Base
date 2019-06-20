package frc.HardwareInterfaces.Transmission;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SpeedController;

/**
 * One of the most complex and rewarding drivetrains to both fabricate and
 * program. Swerve Drive uses motorized caster wheels to control the angle of
 * four traction wheels. Think office roller chair where you can control the
 * direction each wheel faces. This allows the maneuverability of mecanum drive,
 * coupled with the traction, power and speed of a 4 wheel traction system.
 * 
 * There are two main ways to accomplish this: To have one single directional
 * motor connected by a chain to all four wheels, or a directional motor on each
 * wheel independently.
 * 
 * As for software, each directional motor requires a continuous sensor that can
 * accurately measure position, for use in a PID loop.
 * 
 * @author Ryan McGee
 *
 */
public class SwerveTransmission extends TransmissionBase
{

private final BaseMotorController[] multiPID;

private final PIDController monoPID;

private final SpeedController directionalController;

private final Encoder monoSensor;

private final SwerveControlType currentControlScheme;

/**
 * Create the SwerveTransmission Object
 * 
 * If we are using a single motor with a chain wrapped around all four motors to
 * control direction, use this constructor.
 * 
 * @param leftRear
 * @param rightRear
 * @param leftFront
 * @param rightFront
 * @param monoDirectionalController
 * @param directionalSensor
 */
public SwerveTransmission (SpeedController leftRear,
        SpeedController rightRear, SpeedController leftFront,
        SpeedController rightFront,
        SpeedController monoDirectionalController,
        Encoder directionalSensor)
{

    super(leftRear, rightRear, leftFront, rightFront);
    directionalSensor.setPIDSourceType(PIDSourceType.kDisplacement);
    this.monoPID = new PIDController(0, 0, 0, directionalSensor,
            monoDirectionalController);
    this.directionalController = monoDirectionalController;
    this.monoSensor = directionalSensor;

    this.multiPID = null;

    this.currentControlScheme = SwerveControlType.MONO_DIRECTIONAL_CONTROLLER;
    this.type = TransmissionType.TANK;
}

/**
 * Creates the SwerveTransmission Object
 *
 * If we are using separate motors to control direction on each wheel, use this
 * constructor. We are using CAN speed controllers because this would require 16
 * DIO ports and 8 pwm ports for the drivetrain alone otherwise. Controlling the
 * wheel speed:
 * 
 * @param leftRear
 * @param rightRear
 * @param leftFront
 * @param rightFront
 *            Controlling the wheel direction:
 * @param leftRearDirection
 * @param rightRearDirection
 * @param leftFrontDirection
 * @param rightFrontDirection
 */
public SwerveTransmission (SpeedController leftRear,
        SpeedController rightRear, SpeedController leftFront,
        SpeedController rightFront,
        BaseMotorController leftRearDirection,
        BaseMotorController rightRearDirection,
        BaseMotorController leftFrontDirection,
        BaseMotorController rightFrontDirection)
{
    super(leftRear, rightRear, leftFront, rightFront);
    this.multiPID = new BaseMotorController[]
        {leftRearDirection, rightRearDirection, leftFrontDirection,
                rightFrontDirection};

    for (BaseMotorController motor : multiPID)
        {
        motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,
                0, 0);
        motor.setSelectedSensorPosition(0, 0, 0);
        }

    this.monoPID = null;
    this.directionalController = null;
    this.monoSensor = null;

    this.currentControlScheme = SwerveControlType.MULTI_DIRECTIONAL_CONTROLLER;
    super.type = TransmissionType.TANK;
}

/**
 * Sets up PID variables for the directional motors.
 * 
 * @param p
 *            Proportional loop: basic correction based on error.
 * @param i
 *            Integral loop: Correction based on total accumulated error.
 * @param d
 *            Derivative loop: Correction based on slope of distance from
 *            setpoint.
 */
public void setPID (double p, double i, double d)
{
    switch (currentControlScheme)
        {
        case MONO_DIRECTIONAL_CONTROLLER:
            this.monoPID.setPID(p, i, d);
            break;
        case MULTI_DIRECTIONAL_CONTROLLER:
            for (BaseMotorController tempPID : multiPID)
                {
                tempPID.config_kP(0, p, 0);
                tempPID.config_kI(0, i, 0);
                tempPID.config_kD(0, d, 0);
                }
            break;
        default:
            System.out.println(
                    "ERROR: Could't find state in SwerveTransmission.setPID()");
        }
}

/**
 * sets how close the directional motors must be for the drive motors to start
 * running.
 * 
 * @param ticks
 *            Encoder ticks: Directly compared to the error of the PID loop.
 */
public void setOnTargetTolerance (int ticks)
{
    this.onTargetTolerance = ticks;
}

/**
 * Sets the number of ticks per full rotation of the directional motors.
 * 
 * @param ticks
 */
public void setTicksPerRotation (int ticks)
{
    this.ticksPerRotation = ticks;
}

/**
 * Sets how much the robot should rotate based on the joystick input.
 * 
 * @param scalar
 *            Percentage, from 0.0 to 1.0
 */
public void setRotationScalar (double scalar)
{
    this.rotationScalar = Math.abs(scalar);
}

/**
 * 
 * @param wheel
 *            which directional wheel's encoder should be returned
 * @return the encoder position of the sensor attached to the Talon's CAN bus.
 */
public int getRawEncoderPosition (MotorPosition wheel)
{
    switch (wheel)
        {
        case LEFT_REAR:
            // return multiPID[0].getSensorCollection()
            //         .getQuadraturePosition();

        case RIGHT_REAR:
            // return multiPID[1].getSensorCollection()
            //         .getQuadraturePosition();

        case LEFT_FRONT:
            // return multiPID[2].getSensorCollection()
            //         .getQuadraturePosition();

        case RIGHT_FRONT:
            // return multiPID[3].getSensorCollection()
            //         .getQuadraturePosition();

        case ALL:
            return (int) Math.round(monoSensor.pidGet());

        default:
            return 0;
        }
}

/**
 * Gets how far off the current position is from it's setpoint.
 * 
 * @param wheel
 *            Which directional motor to grab the error from
 * @return The error, in ticks
 */
public double getError (MotorPosition wheel)
{
    switch (wheel)
        {
        case LEFT_REAR:
            return multiPID[0].getClosedLoopError(0);
        case RIGHT_REAR:
            return multiPID[1].getClosedLoopError(0);
        case LEFT_FRONT:
            return multiPID[2].getClosedLoopError(0);
        case RIGHT_FRONT:
            return multiPID[3].getClosedLoopError(0);
        case ALL:
            return monoPID.getError();
        default:
            return 0;
        }
}

/**
 * Calibrates the directional motors using a digital sensor to detect whether
 * each wheel is centered.
 * 
 * @param leftRearCondition
 *            When will the left rear be done calibrating?
 * @param rightRearCondition
 *            When will the right rear be done calibrating?
 * @param leftFrontCondition
 *            When will the left front be done calibrating?
 * @param rightFrontCondition
 *            When will the right front be done calibrating?
 * @param override
 *            If this button is pressed, then calibration ceases. This is to
 *            make sure users don't get locked out if calibration fails.
 * @return Whether or not calibration is complete.
 */
public boolean calibrateMotors (boolean leftRearCondition,
        boolean rightRearCondition, boolean leftFrontCondition,
        boolean rightFrontCondition, boolean override)
{
    // Make sure that we don't completely lock out driver control, in case
    // of
    // emergency.
    if (override == true)
        return true;
    // Make sure the transmission can't move while calibrating
    super.stop();
    // Left Rear Motor Calibration
    if (leftRearCondition == true)
        multiPID[0].set(ControlMode.PercentOutput, 0);
    else
        multiPID[0].set(ControlMode.PercentOutput, calibrationSpeed);

    // Right Rear Motor Calibration
    if (rightRearCondition == true)
        multiPID[1].set(ControlMode.PercentOutput, 0);
    else
        multiPID[1].set(ControlMode.PercentOutput, calibrationSpeed);

    // Left Front Motor Calibration
    if (leftFrontCondition == true)
        multiPID[2].set(ControlMode.PercentOutput, 0);
    else
        multiPID[2].set(ControlMode.PercentOutput, calibrationSpeed);

    // Right Front Motor Calibration
    if (rightFrontCondition == true)
        multiPID[3].set(ControlMode.PercentOutput, 0);
    else
        multiPID[3].set(ControlMode.PercentOutput, calibrationSpeed);

    // If all motors are in the right orientation, then stop everything,
    // reset
    // encoders, and continue
    if (leftRearCondition == true && rightRearCondition == true
            && leftFrontCondition == true
            && rightFrontCondition == true)
        {
        // Reset the CAN encoders
        for (BaseMotorController ctrlr : multiPID)
            {
            ctrlr.setSelectedSensorPosition(0, 0, 0);
            ctrlr.set(ControlMode.PercentOutput, 0);
            }

        return true;
        }

    return false;
}

/**
 * Calibrates the single directional motor to make sure what the robot thinks is
 * forwards is actually forwards.
 * 
 * @param centeredCondition
 *            The digital sensor that determines if the calibration is complete
 * @param override
 *            If this is true, then the calibration will stop immediately
 *            without changing the encoder position.
 * @return Whether or not calibration has completed.
 */
public boolean calibrateMotor (boolean centeredCondition,
        boolean override)
{
    if (centeredCondition == true)
        {
        this.monoSensor.reset();
        directionalController.stopMotor();
        return true;
        }

    directionalController.set(calibrationSpeed);

    return false;
}

/**
 * Drives the robot using joystick deadbands and gear ratios. Useful for teleop.
 * 
 * @param magnitude
 *            Overall speed in percentage: 0.0 to 1.0
 * @param direction
 *            Direction in degrees: -180 (backwards) to 0 (forwards) to 180
 *            (backwards again)
 * @param rotation
 *            Rotation, in percent: -1.0 (left) to 1.0 (right)
 */
public void drive (double magnitude, double direction, double rotation)
{
    this.driveRaw(
            scaleJoystickForDeadband(magnitude) * getCurrentGearRatio(),
            direction,
            scaleJoystickForDeadband(rotation) * getCurrentGearRatio()
                    * rotationScalar);
}

/**
 * Drives the robot using raw values
 * 
 * @param magnitude
 *            Overall speed in percentage: 0.0 to 1.0
 * @param direction
 *            Direction in degrees: -180 (backwards) to 0 (forwards) to 180
 *            (backwards again)
 * @param rotation
 *            Rotation, in percent: -1.0 (left) to 1.0 (right)
 */
public void driveRaw (double magnitude, double direction,
        double rotation)
{
    double adjustedMagnitude = magnitude;
    double adjustedDirection = ((direction / 180) + 1) * 180;// Change -180
                                                             // -> 180 to
                                                             // 0 -> 360
    double adjustedRotation = rotation;

    // Are we using one directional motor or four?
    switch (currentControlScheme)
        {
        case MONO_DIRECTIONAL_CONTROLLER:
            // The direction that the single directional motor should face all
            // the wheels toward, based on what's most time efficient.
            double directionalMotorPosition = findDesiredEncoderTicks(
                    findShortestRouteTo((int) adjustedDirection,
                            MotorPosition.ALL),
                    MotorPosition.ALL);

            this.monoPID.setSetpoint(directionalMotorPosition);

            // Decide what speed to set each drive motor, based on robot
            // direction, joystick rotation, and joystick magnitude.
            // This part is almost a carbon copy of the mecanum calculations; it
            // does almost exactly the same thing.
            double leftRearSpeed = -Math
                    .cos(adjustedDirection - (Math.PI / 4.0));
            double rightRearSpeed = -Math
                    .sin(adjustedRotation - (Math.PI / 4.0));
            double leftFrontSpeed = -rightRearSpeed;
            double rightFrontSpeed = -leftRearSpeed;

            // Set all the motors' speeds
            super.getSpeedController(MotorPosition.LEFT_REAR)
                    .set((leftRearSpeed * Math.abs(adjustedRotation))
                            + (adjustedMagnitude * motorSigns[4]));
            super.getSpeedController(MotorPosition.RIGHT_REAR)
                    .set((rightRearSpeed * Math.abs(adjustedRotation))
                            * (adjustedMagnitude * motorSigns[4]));
            super.getSpeedController(MotorPosition.LEFT_FRONT)
                    .set((leftFrontSpeed * Math.abs(adjustedRotation))
                            * (adjustedMagnitude * motorSigns[4]));
            super.getSpeedController(MotorPosition.RIGHT_FRONT)
                    .set((rightFrontSpeed * Math.abs(adjustedRotation))
                            * (adjustedMagnitude * motorSigns[4]));

            break;
        case MULTI_DIRECTIONAL_CONTROLLER:
            // What angle the motors should be going to, based on their current
            // position and what's fastest for them.
            Vector leftFrontVector = calculateWheelVector(
                    adjustedMagnitude, adjustedDirection,
                    adjustedRotation,
                    MotorPosition.LEFT_FRONT);
            Vector leftRearVector = calculateWheelVector(
                    adjustedMagnitude, adjustedDirection,
                    adjustedRotation,
                    MotorPosition.LEFT_REAR);
            Vector rightFrontVector = calculateWheelVector(
                    adjustedMagnitude, adjustedDirection,
                    adjustedRotation,
                    MotorPosition.RIGHT_FRONT);
            Vector rightRearVector = calculateWheelVector(
                    adjustedMagnitude, adjustedDirection,
                    adjustedRotation,
                    MotorPosition.RIGHT_REAR);

            // Make sure no magnitude is going over 1.0, by changing the ratio
            // of
            // all others to make sure it doesn't cap on one vector.
            Vector[] normalized = Vector.normalize(leftRearVector,
                    rightRearVector, leftFrontVector, rightFrontVector);
            leftRearVector = normalized[0];
            rightRearVector = normalized[1];
            leftFrontVector = normalized[2];
            rightFrontVector = normalized[3];

            // Decides whether or not the motors will be allowed to run if the
            // direction is not yet correct.
            boolean canRunMotors = (Math.abs(multiPID[0]
                    .getClosedLoopError(0)) < onTargetTolerance)
                    && (Math.abs(multiPID[1]
                            .getClosedLoopError(0)) < onTargetTolerance)
                    && (Math.abs(multiPID[2]
                            .getClosedLoopError(0)) < onTargetTolerance)
                    && (Math.abs(multiPID[3].getClosedLoopError(
                            0)) < onTargetTolerance);

            // Set the PID loop's set-point to be the angle we found.
            multiPID[0].set(ControlMode.Position,
                    findDesiredEncoderTicks(
                            findShortestRouteTo(
                                    (int) leftRearVector.dirDeg,
                                    MotorPosition.LEFT_REAR),
                            MotorPosition.LEFT_REAR));
            multiPID[1].set(ControlMode.Position,
                    findDesiredEncoderTicks(
                            findShortestRouteTo(
                                    (int) rightRearVector.dirDeg,
                                    MotorPosition.RIGHT_REAR),
                            MotorPosition.RIGHT_REAR));
            multiPID[2].set(ControlMode.Position,
                    findDesiredEncoderTicks(
                            findShortestRouteTo(
                                    (int) leftFrontVector.dirDeg,
                                    MotorPosition.LEFT_FRONT),
                            MotorPosition.LEFT_FRONT));
            multiPID[3].set(ControlMode.Position,
                    findDesiredEncoderTicks(
                            findShortestRouteTo(
                                    (int) rightFrontVector.dirDeg,
                                    MotorPosition.RIGHT_FRONT),
                            MotorPosition.RIGHT_FRONT));

            // If the motors are on target, run them at speed.
            if (canRunMotors == true)
                {
                // If the motors are being reversed to improve efficiency, then
                // motorSigns contains that data.
                super.getSpeedController(MotorPosition.LEFT_REAR)
                        .set(leftRearVector.mag * motorSigns[0]);
                super.getSpeedController(MotorPosition.RIGHT_REAR)
                        .set(rightRearVector.mag * motorSigns[1]);
                super.getSpeedController(MotorPosition.LEFT_FRONT)
                        .set(leftFrontVector.mag * motorSigns[2]);
                super.getSpeedController(MotorPosition.RIGHT_FRONT)
                        .set(rightFrontVector.mag * motorSigns[3]);
                }
            else
                {
                super.stop();
                }
            break;
        default:
            this.stop();
            return;
        }

}

/**
 * Drives the robot using Tank style controls, WHILE turning the wheels to
 * account for rotation correctly.
 */
public void driveRaw (double leftVal, double rightVal)
{
    double direction = 0;
    double magnitude = (leftVal + rightVal) / 2.0;
    double rotation = (leftVal - rightVal) / 2.0;

    if (magnitude < 0)
        direction = 180;
    magnitude = Math.abs(magnitude);

    this.driveRaw(magnitude, direction, rotation);
}

/**
 * Changes whatever the encoder reads to match 0 to 360 degrees, clockwise
 * increasing.
 * 
 * @param original
 *            the raw encoder value
 * @return the adjusted angle the encoder reads, in degrees.
 */
private int adjustEncoderAngle (double original)
{
    // Turn whatever the encoder reads to 0 to 360 degrees in rotation.
    int angle = (int) (((original / ticksPerRotation) * 360) % 360);
    return angle;
}

/**
 * Finds the shortest route to a desired angle for the direction motors, as we
 * can simply reverse the motor direction for quick strafe changes
 * 
 * @param angle
 *            The desired angle
 * @param wheel
 *            Which wheel we're finding the shortest route for
 * @return Either the input angle or 180 degrees from the input angle based on
 *         which is the shortest distance.
 */
private int findShortestRouteTo (int angle, MotorPosition wheel)
{
    // Get the current encoder position
    int encoderPosition = adjustEncoderAngle(
            getRawEncoderPosition(wheel));
    int output = 0;

    // The normal delta between angles, ex. 270 - 260 would be 10 degrees,
    // so 270 is the closest angle.
    if (Math.max(angle, encoderPosition)
            - Math.min(angle, encoderPosition) < 90)
        output = angle;
    // The abnormal delta between angles, ex 10 moved to 350 degrees, so 350
    // would still be the closest.
    else if ((360 - Math.max(angle, encoderPosition)
            + Math.min(angle, encoderPosition)) < 90)
        output = angle;
    // No other closest routes? use the 180 from the input delta and run the
    // motor backwards.
    else
        output = adjustEncoderAngle(angle + 180);

    switch (wheel)
        {
        case LEFT_REAR:
            if (output == angle)
                motorSigns[0] = 1;
            else
                motorSigns[0] = -1;
            break;
        case RIGHT_REAR:
            if (output == angle)
                motorSigns[1] = 1;
            else
                motorSigns[1] = -1;
            break;
        case LEFT_FRONT:
            if (output == angle)
                motorSigns[2] = 1;
            else
                motorSigns[2] = -1;
            break;
        case RIGHT_FRONT:
            if (output == angle)
                motorSigns[3] = 1;
            else
                motorSigns[3] = -1;
            break;
        default:
        case ALL:
            // Single Motor config
            if (output == angle)
                motorSigns[4] = 1;
            else
                motorSigns[4] = -1;
            break;
        }

    return output;
}

/**
 * Calculates a number based on the angle input and the encoder of the wheel
 * input to find the final encoder ticks we want the PID loop to set the
 * directional motors to.
 * 
 * @param angle
 *            What angle the wheel will be angling to
 * @param wheel
 *            Which wheel is being articulated
 * @return an adjusted encoder position based on it's current position. (ex. go
 *         to encoder ticks 370 if the encoder reads 400 and we want 10 degrees)
 */
private int findDesiredEncoderTicks (int angle, MotorPosition wheel)
{
    int encPos = getRawEncoderPosition(wheel);
    int adjustedEncPos = adjustEncoderAngle(encPos);

    // If it is a normal case, then it's just the truncated 360 plus angle
    int possibility1 = (encPos - adjustedEncPos)
            + (angle * (ticksPerRotation / 360));
    // If it is a special case (ex. position is 350, target is 10), then add
    // 360
    int possibility2_CW = possibility1 + ticksPerRotation;
    // Other special case: counter clock wise (ex. from 10 to target 350)
    int possibility2_CCW = possibility1 - ticksPerRotation;

    // If it will cross the 0 degree mark (we figure this out by finding the
    // total delta
    // angle)
    if ((360 - Math.max(adjustedEncPos, angle))
            + Math.min(adjustedEncPos, angle) < 180)
        {
        // The wheel will turn clockwise?
        if (adjustedEncPos > angle)
            return possibility2_CW;
        // The wheel will turn counter-clockwise?
        return possibility2_CCW;
        }

    // If it's a normal case
    return possibility1;
}

/**
 * Calculates the vector of each wheel given the robot's lateral vector, by
 * adding it to a vector defined by the wheel's "turn on spot" position and the
 * amount of rotation from the joystick.
 * 
 * @param magnitude
 *            Magnitude of the Robot
 * @param direction
 *            Direction of the Robot, in degrees
 * @param rotation
 *            Rotation of the Robot, in percentage (-1 to 1)
 * @param wheel
 *            which wheel to choose the rotation position for
 * @return the resultant vector of the given wheel: The magnitude should be fed
 *         into the power of the drive wheel, and the direction fed into the
 *         directional motor's PID loop.
 */
private Vector calculateWheelVector (double magnitude, double direction,
        double rotation, MotorPosition wheel)
{
    Vector robotVector = new Vector(magnitude, direction);
    int specificRotationAngle = 0;
    switch (wheel)
        {
        case LEFT_REAR:
            specificRotationAngle = 135;
            break;
        case RIGHT_REAR:
            specificRotationAngle = 45;
            break;
        case LEFT_FRONT:
            specificRotationAngle = 225;
            break;
        case RIGHT_FRONT:
            specificRotationAngle = 315;
            break;
        default:
            break;
        }
    Vector wheelRotationVector = new Vector(rotation,
            specificRotationAngle);
    return Vector.add(wheelRotationVector, robotVector);
}

/**
 * The kinds of swerve drive available:
 * 
 * mono-directional-control: One motor controls a chain around the whole bot
 * that controls which direction it goes. Simpler and more power efficient.
 * 
 * mulit-directional-control: Each wheel has a different motor that controls
 * it's direction More flexible programming and smoother control.
 * 
 * @author Ryan McGee
 */
private enum SwerveControlType
    {
MONO_DIRECTIONAL_CONTROLLER, MULTI_DIRECTIONAL_CONTROLLER
    }

// ================Variables================

private int[] motorSigns = new int[]
    {1, 1, 1, 1, 1};

private int ticksPerRotation = 360;

private int onTargetTolerance = 3;

private double rotationScalar = 1;

private double calibrationSpeed = .3;

/**
 * Basic vector class designed to do math with said vectors easier
 * 
 * @author Ryan McGee
 */
static class Vector
{
enum VectorIn
    {
Point, Vector
    }

final double mag, dirRad, dirDeg, x, y;

/**
 * Creates the Vector object with a given magnitude and direction
 * 
 * @param mag
 *            Magnitude of the vector: the hypotenuse of the x and y points.
 * @param dir
 *            Direction of the vector: tan-1(x / y) of the points.
 */
public Vector (double mag, double dir)
{
    this.mag = mag;
    this.dirRad = dir;
    this.dirDeg = Math.toDegrees(dirRad);

    this.x = mag * Math.sin(dir);
    this.y = mag * Math.cos(dir);
}

/**
 * Creates a vector based on an x and y coordinate pair.
 * 
 * @param point
 *            the given point, and the vector will be based on it's position
 *            from 0,0.
 */
public Vector (Point point)
{
    this.x = point.x;
    this.y = point.y;

    this.mag = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    this.dirRad = Math.atan2(y, x);
    this.dirDeg = Math.toDegrees(dirRad);
}

/**
 * Adds two vectors together
 * 
 * @param a
 *            First vector to add
 * @param b
 *            Second vector to add to the first
 * @return the resultant vector
 */
public static Vector add (Vector a, Vector b)
{
    return new Vector(new Point(a.x + b.x, a.y + b.y));
}

/**
 * Subtracts vector b from vector a
 * 
 * @param a
 *            The initial vector
 * @param b
 *            What's subtracted from the initial vector
 * @return The resultant vector
 */
public static Vector subtract (Vector a, Vector b)
{
    return new Vector(new Point(a.x - b.x, a.y - b.y));
}

/**
 * Change the magnitudes of all vectors input to be a maximum at one, but keep
 * the ratio between each magnitude in the array constant.
 * 
 * @param vec
 *            The vectors to be normalized
 * @return the normalized vectors
 */
public static Vector[] normalize (Vector... vec)
{
    // Make sure that the max isn't more than one: We don't need it to
    // be. Search for the maximum magnitude.
    double maxMag = 1;
    for (Vector a : vec)
        maxMag = Math.max(Math.abs(a.mag), maxMag);
    // If the biggest magnitude is still under one, don't bother
    // processing further.
    if (maxMag != 1 && maxMag != 0)
        // If it's over one, then normalize all vectors with a constant
        // ratio, the largest magnitude remaining one.
        for (Vector a : vec)
            a = new Vector(a.mag / maxMag, a.dirRad);

    return vec;
}
}

/**
 * Basic class to store a single x/y coordinate pair
 * 
 * @author Ryan McGee
 *
 */
static class Point
{
public double x = 0, y = 0;

/**
 * Create the Point object.
 * 
 * @param x
 * @param y
 */
public Point (double x, double y)
{
    this.x = x;
    this.y = y;
}

}
}
