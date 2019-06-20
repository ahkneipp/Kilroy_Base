package frc.Utils;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.Hardware.Hardware;
import frc.HardwareInterfaces.KilroyEncoder;
import frc.HardwareInterfaces.RobotPotentiometer;
import frc.robot.Teleop;
import frc.HardwareInterfaces.LightSensor;
import frc.HardwareInterfaces.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.HardwareInterfaces.QuickSwitch;

/**
 *
 *
 * @author Cole (mostly) 2019 build season
 *         Other Contributors: Ashley
 */
public class GamePieceManipulator
{

private SpeedController armMotor = null;

private RobotPotentiometer armPot = null;

private KilroyEncoder armEncoder = null;

private RollerIntakeMechanism intake = null;


/**
 * constructor to use in hardware
 */
public GamePieceManipulator (SpeedController armMotor,
        RobotPotentiometer armPot, SpeedController armRollers,
        LightSensor photoSwitch, DoubleSolenoid armSolenoid)
{
    this.armMotor = armMotor;
    this.armPot = armPot;
    this.intake = new RollerIntakeMechanism(armRollers, photoSwitch,
            armSolenoid);
}

/**
 * constructor to use in hardware
 */
public GamePieceManipulator (SpeedController armMotor,
        KilroyEncoder armEncoder, SpeedController armRollers,
        LightSensor photoSwitch, DoubleSolenoid armSolenoid)
{
    this.armMotor = armMotor;
    this.armEncoder = armEncoder;
    this.intake = new RollerIntakeMechanism(armRollers, photoSwitch,
            armSolenoid);
}

public static enum GamePiece
    {
    HATCH_PANEL, CARGO, NONE, BOTH
    }

// placeholder, will need to do something
// used to tell us things also
public static enum DeployState
    {
    DEPLOYED, MIDDLE, RETRACTED
    }

public static enum DeployMovementState
    {
    MOVING_TO_POSITION, MOVING_BY_JOY, STAY_AT_POSITION, STOP, SET_MANUALLY_FOR_CLIMB, MOVING_TO_POSITION_PRECISE
    }

private static enum DeployMovementDirection
    {
    MOVING_UP, MOVING_DOWN, NEUTRAL
    }

// =========================================================================
// General Methods
// =========================================================================

// placeholder function since Forklift will need to understand which piece
// the manipulator has
// public GamePiece hasWhichGamePiece ()
// {
// if (this.intake.hasCargo() /* and does not have a Hatch */)
// {
// return GamePiece.CARGO;
// }

// return GamePiece.NONE;
// }

public void initiliazeConstantsFor2018 ()
{
    // UP_JOYSTICK_SCALER = UP_JOYSTICK_SCALER_2018;
    // DOWN_JOYSTICK_SCALER = DOWN_JOYSTICK_SCALER_2018;
    SET_ANGLE_UPWARD_ARM_MOVEMENT_SCALER = UPWARD_ARM_MOVEMENT_SCALER_2018;
    SET_ANGLE_DOWNWARD_ARM_MOVEMENT_SCALER = DOWNWARD_ARM_MOVEMENT_SCALER_2018;
    MAX_ARM_POSITION_ADJUSTED = MAX_ARM_POSITION_ADJUSTED_2018;
    MIN_ARM_POSITION_ADJUSTED = MIN_ARM_POSITION_ADJUSTED_2018;
    RETRACTED_ARM_POSITION_ADJUSTED = RETRACTED_ARM_POSITION_ADJUSTED_2018;
    PARALLEL_TO_GROUND_ADJUSTED = PARALLEL_TO_GROUND_ADJUSTED_2018;
    ARM_POT_SCALE_TO_DEGREES = ARM_POT_SCALE_TO_DEGREES_2018;
    STAY_UP_WITH_CARGO = STAY_UP_WITH_CARGO_2018;
    STAY_UP_NO_PIECE = STAY_UP_NO_PIECE_2018;
    ARM_POT_RAW_HORIZONTAL_VALUE = ARM_POT_RAW_HORIZONTAL_VALUE_2018;
    DEPLOYED_ARM_POSITION_ADJUSTED = DEPLOYED_ARM_POSITION_ADJUSTED_2018;
    DEFAULT_DEPLOY_SPEED_UNSCALED = DEFAULT_DEPLOY_SPEED_UNSCALED_2018;
    DEFAULT_RETRACT_SPEED_UNSCALED = DEFAULT_RETRACT_SPEED_UNSCALED_2018;
    DEFAULT_MOVE_BY_BUTTON_SPEED_UNSCALED = DEFAULT_MOVE_BY_BUTTON_SPEED_UNSCALED_2018;
}

/**
 * Returns true is the manipulator is holding cargo, based on the photo
 * switch. False otherwise
 */
public boolean hasCargo ()
{
    return this.intake.hasCargo();
}


/**
 * Returns true is the deploy mechanism has been deployed (based on the
 * deploy potentiometer/ encoder), false otherwise
 */
public boolean isDeployed ()
{
    return this.getDeployState() == DeployState.DEPLOYED;
}

/**
 * returns true is the arm is clear of the frame and if we are able to raise the
 * forklift
 *
 */
public boolean isArmClearOfFrame ()
{
    return this.getCurrentArmPosition() < IS_FULLY_CLEAR_OF_FRAME_ANGLE;
}

public boolean isArmPartiallyClearOfFrame ()
{
    return this
            .getCurrentArmPosition() < IS_PARTIALLY_CLEAR_OF_FRAME_ANGLE;
}




/**
 * Gets the current state of the deploy mechanism (DEPLOYED, MIDDLE
 * (not deployed or retracted), and RETRACTED). Not to be confused
 * with the deployMovementState, which is used by the state machine
 * to determine how the deploy should be moving
 *
 * @return the current state of the deploy mechanism
 */
public DeployState getDeployState ()
{
    if (this.getCurrentArmPosition() >= RETRACTED_ARM_POSITION_ADJUSTED
            - ACCEPTABLE_ERROR)
        return DeployState.RETRACTED;
    if (this.getCurrentArmPosition() <= DEPLOYED_ARM_POSITION_ADJUSTED
            + ACCEPTABLE_ERROR)
        return DeployState.DEPLOYED;
    return DeployState.MIDDLE;
}

/** Update all the states machines for this class */
public void masterUpdate ()
{
    this.intake.update();
    this.deployUpdate();
}


// =========================================================================
// armMotor methods
// =========================================================================


/**
 * call during teleop to move the arm up and down based on the joystick controls
 */
public void moveArmByJoystick (Joystick armJoystick,
        boolean overrideButton)
{
    if (Math.abs(armJoystick
            .getY()) > DEPLOY_JOYSTICK_DEADBAND)
        {
        // assuming up is a positive value
        // speed is not scaled based off the DEADBAND_BAND
        // because if it was, we might give it too weak of
        // a value at low joystick values outside the deadband
        // (i.e.: if we are just .01 outside the deadband, and
        // we sent the motor .01, then it might actually move
        // down due to gravity)
        double joystickValue = armJoystick.getY();

        // if override button is pressed, ignore potentiometer/ encoder.

        // If we are trying to move up and past the max angle, or
        // trying to move down and below the min height, tell the
        // arm to stay where it is
        if (overrideButton == false)
            {
            if ((joystickValue > 0
                    && this.getCurrentArmPosition() > currentDeployMaxAngle)
                    || (joystickValue < 0 && this
                            .getCurrentArmPosition() < currentDeployMinAngle))
                {
                this.deployMovementState = DeployMovementState.STAY_AT_POSITION;
                // return so we exit the method and do not accidentally set
                // deployMovementState to MOVING_BY_JOY;
                return;
                }
            }

        // scales the speed based on whether it is going up or down
        if (joystickValue > 0)
            this.deployTargetSpeed = this
                    .calculateDesiredArmMotorVoltage(
                            RequiredArmSpeedState.GO_UP,
                            overrideButton, true);
        else
            this.deployTargetSpeed = this
                    .calculateDesiredArmMotorVoltage(
                            RequiredArmSpeedState.GO_DOWN,
                            overrideButton, true);


        // double minScaler = .5;
        // ((Math.abs(joystickValue) - deadband) * (1 - minScaler)/ (1 -
        // deadband)) + minScaler

        // this.deployTargetSpeed *= ((Math.abs(joystickValue) - .2)
        // * .625) + .5;

        // Scales the target speed based off the joystick
        this.deployTargetSpeed *= ((Math.abs(joystickValue) - .2)
                * .9375) + .25;

        this.deployMovementState = DeployMovementState.MOVING_BY_JOY;

        }

}


/**
 * set the power of the manipulator to prepare to climb to level two
 *
 * @param button
 *                   joystick button to be pressed to run the function
 */
public void poweredDeployDownForClimb (JoystickButton button)
{
    if (button.get() == true)
        this.setArmMotorSpeedManuallyForClimb(
                SUPPORT_DRIVE_FOR_CLIMB_ARM_SPEED);
}


private static final double SUPPORT_DRIVE_FOR_CLIMB_ARM_SPEED = -.8;

/**
 * Returns angle of the arm by scaling the potentiometer value
 * for the deploy.
 *
 * @returns the angle of the arm in degrees, with 0 representing when
 *          the arm is parallel to the ground, and +90 when the arm
 *          is straight up and down
 */
public double getCurrentArmPosition ()
{
    // if (armPot != null)
    // {
    // scales the value from the arm pot so parallel to the ground is
    // zero, and perpendicular to the ground and pointing up is 90

    // if (Hardware.demoMode == false)
        {
        return (this.armPot.get()
                - ARM_POT_RAW_HORIZONTAL_VALUE)
                * ARM_POT_SCALE_TO_DEGREES;
        }


    // } else // if we are not using an armPot, we should be using an encoder
    // {
    // // assumes that the value from the encoder is reset to 0
    // // when the robot is started and negative when the manipulator
    // // is below the starting position
    // double valueFromHorizontal = (armEncoder.get()
    // - ARM_ENCODER_RAW_HORIZONTAL_VALUE)
    // * ARM_ENCODER_SCALE_TO_DEGREES;

    // return valueFromHorizontal;
    // }
}


// public double getCurrentArmPositionOld ()
// {
// return (this.armPot.get()
// - ARM_POT_RAW_HORIZONTAL_VALUE)
// * ARM_POT_SCALE_TO_DEGREES;
// }

// public double getCurrentArmPositionAverage ()
// {
// return (armAngle1Ago + armAngle2Ago + armAngle3Ago) / 3;
// }

// public void initiliazeArmPositonAverage ()
// {
// armAngle1Ago = this.getCurrentArmPositionOld();
// armAngle2Ago = armAngle1Ago;
// armAngle3Ago = armAngle1Ago;
// }

// public void updatePastArmPositions ()
// {
// armAngle3Ago = armAngle2Ago;

// armAngle2Ago = armAngle1Ago;

// armAngle1Ago = this.getCurrentArmPositionOld();
// }

// // ToDO maybe replace this with an array in future
// // was not done since it would require a for loop
// // average values in array

// private double armAngle1Ago = 0.0;

// private double armAngle2Ago = 0.0;

// private double armAngle3Ago = 0.0;


/**
 * Method for setting the deploy arm to a preset angle using a button.
 * For use in teleop. The button just needs to be pressed once (not held)
 * and the dpeloy state machine will start moving to the necessary angle.
 * This can be interruted at any time by moving the joysticks past their
 * deadzones (causing joystick control to take over).
 *
 * This should be called directly as is in teleop and does not need to
 * be surrounded by any if statements
 *
 *
 * @param angle
 *                     the angle the arm will be moved to, in degrees; 0 is
 *                     parallel to the ground and 90 is perpendicualr to the
 *                     ground.
 * @param armSpeed
 *                     the desired speed the arm will be moved at
 * @param button
 *                     the QuickSwitch we are using to say when we want
 *                     to move to the specified angle
 *
 */
public void moveArmByButton (double angle, QuickSwitch button)
{
    // if the button is being held down and was not being held down before
    if (button.getCurrentValue() == true)
        {
        isSetDeployPositionInitReady = true;
        this.moveArmToPosition(angle);
        }
}

/**
 * Function to move the deploy arm to a specified angle. For use
 * in autonomous only (not in teleop). For teleop, please use
 * moveArmByButton.
 *
 * @param angle
 *                  the angle the arm will be moved to, in degrees; 0 is
 *                  parallel to the ground and 90 is perpendicualr to the
 *                  ground.
 *
 * @param speed
 *                  the speed the arm will move at
 *
 *
 * @return true when the arm has finished moving to the proper
 *         position, false otherwise
 */
public boolean moveArmToPosition (double angle)
{
    // Sets the target position and speed, enables "moving-to-position"
    // state.
    if (isSetDeployPositionInitReady == true)
        {
        this.deployTargetAngle = angle;
        deployDirection = DeployMovementDirection.NEUTRAL;
        this.deployMovementState = DeployMovementState.MOVING_TO_POSITION;
        isSetDeployPositionInitReady = false;
        }

    // return true is we are done moving, false is we are still going
    if (this.deployMovementState == DeployMovementState.STAY_AT_POSITION)

        {
        isSetDeployPositionInitReady = true;
        return true;
        }

    if (deployTargetAngle > this.getCurrentArmPosition())
        {
        this.deployTargetSpeed = this
                .calculateDesiredArmMotorVoltage(
                        RequiredArmSpeedState.GO_UP,
                        false, false);
        }
    else // if the manipulator will move down
        {
        this.deployTargetSpeed = this
                .calculateDesiredArmMotorVoltage(
                        RequiredArmSpeedState.GO_DOWN,
                        false, false);
        }

    this.deployTargetSpeed = Math.abs(this.deployTargetSpeed);

    return false;
}

public boolean moveArmToPositionPrecise (double angle)
{
    if (deployTargetAngle != angle)
        {
        moveArmToPositionPreciseInit = true;
        }
    // Sets the target position and speed, enables "moving-to-position"
    // state.
    if (moveArmToPositionPreciseInit == true)
        {
        this.deployTargetAngle = angle;
        deployDirection = DeployMovementDirection.NEUTRAL;
        this.deployMovementState = DeployMovementState.MOVING_TO_POSITION_PRECISE;
        moveArmToPositionPreciseInit = false;
        }

    // return true is we are done moving, false is we are still going
    if (this.deployMovementState == DeployMovementState.STAY_AT_POSITION)

        {
        moveArmToPositionPreciseInit = true;
        return true;
        }

    if (deployTargetAngle > this.getCurrentArmPosition())
        {
        this.deployTargetSpeed = this
                .calculateDesiredArmMotorVoltage(
                        RequiredArmSpeedState.GO_UP,
                        false, false);
        }
    else // if the manipulator will move down
        {
        this.deployTargetSpeed = this
                .calculateDesiredArmMotorVoltage(
                        RequiredArmSpeedState.GO_DOWN,
                        false, false);
        }

    this.deployTargetSpeed = Math.abs(this.deployTargetSpeed);

    return false;
}

private boolean moveArmToPositionPreciseInit = true;


public boolean defaultSetAngle (double angle)
{
    // return this.moveArmToPosition(angle);
    return this.moveArmToPositionPrecise(angle);
}


/**
 * Tells the state machine to deploy the arm, if it is not
 * already retracted
 *
 * Can be called once to tell the deploy to move using the state
 * machine in the background, or called continually for autonomous
 * code that waits until the arm deploys before moving on
 *
 * @returns true if the arm finished deploying, or was already deployed
 */
public boolean deployArm ()
{
    // if (this.getDeployState() != DeployState.DEPLOYED)
    // {
    // isSetDeployPositionInitReady = true;
    return this.moveArmToPosition(DEPLOYED_ARM_POSITION_ADJUSTED);
    // }
    // else
    // {
    // return true; // if we are already deployed
    // }
}

/**
 * Tells the state machine to retract the arm, if it is not
 * already retracted
 *
 * Can be called once to tell the deploy to move using the state
 * machine in the background, or called continually for autonomous
 * code that waits until the arm retracts before moving on
 *
 * @returns true if the arm finished retracting, or was already retracted
 */
public boolean retractArm ()
{
    // if (this.getDeployState() != DeployState.RETRACTED)
    // {
    // isSetDeployPositionInitReady = true;
    return this.moveArmToPosition(RETRACTED_ARM_POSITION_ADJUSTED);
    // }
    // return true; // if we are already deployed
}


/**
 * sets the state of the deploy state machine to whatever state you pass in
 *
 * @param State
 */
public boolean setDeployMovementState (
        DeployMovementState targetDeployMovementState)
{
    deployMovementState = targetDeployMovementState;

    if (deployMovementState == targetDeployMovementState)
        {
        return true;
        }
    else
        {
        return false;
        }
}

public void setMaxArmAngle (double angle)
{
    this.currentDeployMaxAngle = angle;
}

public boolean toggleIgnoreMakeBreak ()
{
    return this.intake.toggleIgnoreMakeBreak();
}


/**
 *
 */
public void setAngleForForkliftNextPostion (double angle)
{
    double tempCurrentAngle = this.getCurrentArmPosition();

    if (Math.abs(tempCurrentAngle
            - angle) > SET_ANGLE_DEADBAND)
        {
        // we are below the target angle, move up
        if (tempCurrentAngle < angle)
            {
            angle -= NEXT_HIGHER_ANGLE_ADJUSTMENT;
            // SmartDashboard.putNumber("Next Higher Angle Adjustment",
            // angle);
            }
        // we are above the target angle, move down
        else
            {
            angle += NEXT_LOWER_ANGLE_ADJUSTMENT;
            // SmartDashboard.putNumber("Next Lower Angle Adjustment",
            // angle);
            }

        this.moveArmToPosition(angle);
        }

}

private final double SET_ANGLE_DEADBAND = 3.0;

private final double NEXT_HIGHER_ANGLE_ADJUSTMENT = 3;

private final double NEXT_LOWER_ANGLE_ADJUSTMENT = 3;


public void setIntakeState (RollerIntakeMechanism.IntakeState state)
{
    this.intake.setIntakeState(state);
}


/**
 * Update method for the deploy state machine. Is what actually tells
 * the armMotor what to do based off the current deployMovementState.
 * This method needs to be called in Teleop or Autonomous periodic
 * in order for the deploy to be used in either function, respectively
 */
public void deployUpdate ()
{

    if (Hardware.demoMode == true)
        {
        if (this.intake.isOpen() == true)
            {
            currentDeployMinAngle = DEMO_MODE_OPEN_MIN_ANGLE;
            }
        else
            {
            currentDeployMinAngle = MIN_ARM_POSITION_ADJUSTED;
            }
        }
    // this.updatePastArmPositions();
    if (deployMovementState != DeployMovementState.STAY_AT_POSITION)
        {
        this.stayAtPosition2018InitIsReady = true;
        this.stayAtPositionInitIsReady = true;
        }
    if (deployMovementState != DeployMovementState.MOVING_TO_POSITION_PRECISE)
        moveArmToPositionPreciseInit = true;

    switch (deployMovementState)
        {
        case MOVING_TO_POSITION:
            if ((this.deployTargetAngle > currentDeployMaxAngle)
                    || (this.deployTargetAngle < currentDeployMinAngle))
                {
                deployMovementState = DeployMovementState.STAY_AT_POSITION;
                break;
                }

            // Begins by stating whether we are increasing or decreasing
            if (deployDirection == DeployMovementDirection.NEUTRAL)
                {
                if (deployTargetAngle < this.getCurrentArmPosition())
                    deployDirection = DeployMovementDirection.MOVING_DOWN;
                else
                    deployDirection = DeployMovementDirection.MOVING_UP;
                }

            // Differentiate moving up from down
            if (deployDirection == DeployMovementDirection.MOVING_UP)
                {
                // If we have passed the value we wanted...
                if (this.getCurrentArmPosition() > deployTargetAngle)
                    {
                    deployMovementState = DeployMovementState.STAY_AT_POSITION;
                    // Reset the direction for next time.
                    deployDirection = DeployMovementDirection.NEUTRAL;
                    break;
                    }
                // we have NOT passed the value , keep going up.
                this.armMotor.set(deployTargetSpeed);
                }
            else
                {
                // If we have passed the value we wanted...
                if (this.getCurrentArmPosition() < deployTargetAngle)
                    {
                    deployMovementState = DeployMovementState.STAY_AT_POSITION;
                    // Reset the direction for next time.
                    deployDirection = DeployMovementDirection.NEUTRAL;
                    break;
                    }
                // we have NOT passed the value , keep going down.
                this.armMotor.set(-deployTargetSpeed);
                }
            break;

        case MOVING_TO_POSITION_PRECISE:
            this.movingToPositionPreciseState();
            break;

        case MOVING_BY_JOY:
            isSetDeployPositionInitReady = true;
            this.armMotor.set(deployTargetSpeed);
            // If we are no longer holding the joystick, then it will
            // automatically stay at position. If we are holding the
            // joysticks, then other functions will set
            // deployMovementState back to MOVINg_BY_JOY before we get
            // back here
            deployMovementState = DeployMovementState.STAY_AT_POSITION;
            break;

        default:
        case STAY_AT_POSITION:
            // TODO the new armMotor might not even need a voltage to
            // stay in place, so we might be able to just give the arm motor
            // 0.0 no matter what game piece we have

            // Depending on what piece the manipulator has, send the appropriate
            // value to the motor so the forklift does not slide down due to
            // gravity
            // If the manipulator has a cargo piece, send the appropriate
            // value to the motor so the forklift does not slide down due to
            // gravity
            if (Hardware.whichRobot == Hardware.RobotYear.KILROY_2018)
                {
                if (this.stayAtPosition2018InitIsReady == true)
                    {
                    if (this.hasCargo() == true)
                        stayAtPositionTempSpeed = Math
                                .abs(STAY_UP_WITH_CARGO_2018
                                        * Math.cos(this
                                                .getCurrentArmPosition()));
                    else
                        stayAtPositionTempSpeed = Math
                                .abs(STAY_UP_NO_PIECE_2018
                                        * Math.cos(this
                                                .getCurrentArmPosition()));
                    this.stayAtPosition2018InitIsReady = false;
                    }
                this.armMotor.set(stayAtPositionTempSpeed);
                }
            else
                {
                if (this.stayAtPositionInitIsReady == true)
                    {

                    stayAtPositionTempSpeed = this
                            .calculateDesiredArmMotorVoltage(
                                    RequiredArmSpeedState.HOLD, false,
                                    false);

                    this.stayAtPositionInitIsReady = false;
                    }

                this.armMotor.set(stayAtPositionTempSpeed);
                // this.armMotor.set(0.0);
                }

            // Reset the direction for next move-to-position.
            deployDirection = DeployMovementDirection.NEUTRAL;
            isSetDeployPositionInitReady = true;
            break;

        // allows for the code to autonomously constantly send a voltage to
        // the manipualtor, since the climb function will need to be able
        // to do so in order to use the manipulator to hoist up the robot.
        case SET_MANUALLY_FOR_CLIMB:
            isSetDeployPositionInitReady = true;
            this.armMotor.set(deployTargetSpeed);
            deployMovementState = DeployMovementState.STAY_AT_POSITION;
            break;

        case STOP:
            this.armMotor.set(0.0);
            break;

        }

}


private void movingToPositionPreciseState ()
{
    double currentAngle = this.getCurrentArmPosition();
    double adjustedSpeed = this.deployTargetSpeed;
    // positive if we are above target
    double distanceFromHeight = Math
            .abs(currentAngle - this.deployTargetAngle);



    if ((currentAngle > currentDeployMaxAngle)
            || (currentAngle < currentDeployMinAngle)
            || (this.deployTargetAngle > currentDeployMaxAngle)
            || (this.deployTargetAngle < currentDeployMinAngle))
        {
        deployMovementState = DeployMovementState.STAY_AT_POSITION;
        return;
        }

    // Begins by stating whether we are increasing or decreasing
    if (deployDirection == DeployMovementDirection.NEUTRAL)
        {
        if (deployTargetAngle < currentAngle)
            {
            deployDirection = DeployMovementDirection.MOVING_DOWN;
            }
        else
            deployDirection = DeployMovementDirection.MOVING_UP;
        }

    // Differentiate moving up from down
    if (deployDirection == DeployMovementDirection.MOVING_UP)
        {
        // If we have passed the value we want to stop at, adjusted
        // so the manipulator does not overshoot
        if (distanceFromHeight < UPWARD_EARLIER_STOP_ADJUSTMENT
                || deployTargetAngle < currentAngle)
            {
            deployMovementState = DeployMovementState.STAY_AT_POSITION;
            return;
            }
        // When we are close enough to the target height, set the adjusted
        // speed to a preset speed that will slow down the manipulator so
        // it will not overshoot/ will only overshoot by a predictable
        // amount
        if (distanceFromHeight < UPWARD_DECELLERATION_START_ADJUSTMENT)
            adjustedSpeed = this.calculateDesiredArmMotorVoltage(
                    RequiredArmSpeedState.GO_UP_PRECISE,
                    false, false);
        else
            adjustedSpeed = this.calculateDesiredArmMotorVoltage(
                    RequiredArmSpeedState.GO_UP,
                    false, false);

        // we have NOT passed the value , keep going up.
        this.armMotor.set(adjustedSpeed);
        }
    else
        {
        // If we have passed the value we want to stop at, adjusted
        // so the manipulator does not overshoot
        if (distanceFromHeight < DOWNWARD_EARLIER_STOP_ADJUSTMENT
                || deployTargetAngle > currentAngle)
            {
            deployMovementState = DeployMovementState.STAY_AT_POSITION;
            return;
            }
        // When we are close enough to the target height, set the adjusted
        // speed to a preset speed that will slow down the manipulator so
        // it will not overshoot/ will only overshoot by a predictable
        // amount
        if (distanceFromHeight < DOWNWARD_DECELLERATION_START_ADJUSTMENT)
            adjustedSpeed = this.calculateDesiredArmMotorVoltage(
                    RequiredArmSpeedState.GO_DOWN_PRECISE,
                    false, false);
        else
            adjustedSpeed = this.calculateDesiredArmMotorVoltage(
                    RequiredArmSpeedState.GO_DOWN,
                    false, false);

        // we have NOT passed the value , keep going down.
        this.armMotor.set(adjustedSpeed);
        }


}

private double UPWARD_EARLIER_STOP_ADJUSTMENT = 6.0;

// private double UPWARD_SLOWED_PRECISE_SCALER = 0.5;

// probably does not need to be this high; can be scaled down to 30 or 20ish
// which would make it faster
private double UPWARD_DECELLERATION_START_ADJUSTMENT = 20.0;

private double DOWNWARD_EARLIER_STOP_ADJUSTMENT = 4.0;

// private double DOWNWARD_SLOWED_PRECISE_SCALER = .2;

private double DOWNWARD_DECELLERATION_START_ADJUSTMENT = 20.0;

// private final double UPWARD_SLOWED_SPEED_2018 = 0.7;



/**
 *
 *
 * @param state
 *                            the state that is callingthis function
 * @param isOverriding
 *                            if we are overridingwhile calling this function
 * @param isUsingJoystick
 *                            if we are using the joystick while calling this
 *                            function
 * @return
 */
private double calculateDesiredArmMotorVoltage (
        RequiredArmSpeedState state,
        boolean isOverriding, boolean isUsingJoystick)
{
    double speed = 0.0;
    double currentArmAngle = this.getCurrentArmPosition();

    switch (state)
        {
        case GO_UP:
            if (isOverriding == true)
                {
                speed = GO_UP_GRAVITY_OUT_OF_FRAME_LOW_SPEED;
                }
            else
                {
                // if not overridingis false
                if (currentArmAngle > ARM_NO_GRAVITY_ANGLE)
                    speed = GO_UP_HOLD_ARM_NO_GRAVITY_SPEED;
                // else
                // if (currentArmAngle >
                // ARM_GRAVITY_OUT_OF_FRAME_HIGH_ANGLE)
                // speed = GO_UP_GRAVITY_OUT_OF_FRAME_HIGH_SPEED;
                else
                    speed = GO_UP_GRAVITY_OUT_OF_FRAME_LOW_SPEED;
                }

            if (isUsingJoystick == false)
                speed *= SET_POSITION_SPEED_SCALE_FACTOR;
            break;
        case GO_DOWN:
            if (isOverriding == true)
                {
                speed = GO_DOWN_HOLD_ARM_NO_GRAVITY_SPEED;
                }
            else
                {
                if (currentArmAngle > ARM_NO_GRAVITY_ANGLE)
                    speed = GO_DOWN_HOLD_ARM_NO_GRAVITY_SPEED;
                else
                    // if (currentArmAngle >
                    // ARM_GRAVITY_OUT_OF_FRAME_HIGH_ANGLE)
                    // speed = GO_DOWN_GRAVITY_OUT_OF_FRAME_HIGH_SPEED;
                    // else
                    speed = GO_DOWN_GRAVITY_OUT_OF_FRAME_LOW_SPEED;
                }
            if (isUsingJoystick == false)
                speed *= SET_POSITION_SPEED_SCALE_FACTOR;
            break;
        case GO_UP_PRECISE:
            if (currentArmAngle > ARM_NO_GRAVITY_ANGLE)
                speed = GO_UP_HOLD_ARM_NO_GRAVITY_SPEED_PRECISE;
            // else
            // if (currentArmAngle >
            // ARM_GRAVITY_OUT_OF_FRAME_HIGH_ANGLE)
            // speed = GO_UP_GRAVITY_OUT_OF_FRAME_HIGH_SPEED;
            else
                speed = GO_UP_GRAVITY_OUT_OF_FRAME_LOW_SPEED_PRECISE;
            break;
        case GO_DOWN_PRECISE:
            if (currentArmAngle > ARM_NO_GRAVITY_ANGLE)
                speed = GO_DOWN_HOLD_ARM_NO_GRAVITY_SPEED_PRECISE;
            else
                // if (currentArmAngle >
                // ARM_GRAVITY_OUT_OF_FRAME_HIGH_ANGLE)
                // speed = GO_DOWN_GRAVITY_OUT_OF_FRAME_HIGH_SPEED;
                // else
                speed = GO_DOWN_GRAVITY_OUT_OF_FRAME_LOW_SPEED_PRECISE;
            break;
        case FORCE_DOWN:
            break;
        case HOLD:
            // if (this.hasCargo() == true)
            // {

            // }
            // else
            // {
            if (currentArmAngle > ARM_GRAVITY_IN_FRAME_ANGLE)
                speed = HOLD_ARM_GRAVITY_IN_FRAME_SPEED;
            else
                if (currentArmAngle > ARM_NO_GRAVITY_ANGLE)
                    speed = HOLD_ARM_NO_GRAVITY_SPEED;
                else
                    if (currentArmAngle > ARM_GRAVITY_OUT_OF_FRAME_HIGH_ANGLE)
                        speed = HOLD_ARM_GRAVITY_OUT_OF_FRAME_HIGH_SPEED;
                    else
                        speed = HOLD_ARM_GRAVITY_OUT_OF_FRAME_LOW_SPEED;
            // }
            break;
        }
    // SmartDashboard.putNumber("getArmMotorVoltage", speed);
    return speed;

}

// measured in degrees
private double ARM_GRAVITY_IN_FRAME_ANGLE = 105;

private double HOLD_ARM_GRAVITY_IN_FRAME_SPEED = -0.04;

private double ARM_NO_GRAVITY_ANGLE = 80;

private double HOLD_ARM_NO_GRAVITY_SPEED = 0.02;

private double ARM_GRAVITY_OUT_OF_FRAME_HIGH_ANGLE = 45;

private double HOLD_ARM_GRAVITY_OUT_OF_FRAME_HIGH_SPEED = 0.09;

private double HOLD_ARM_GRAVITY_OUT_OF_FRAME_LOW_SPEED = 0.13;

private double GO_UP_HOLD_ARM_NO_GRAVITY_SPEED = .5
        * MAX_DEPLOY_SPEED_2019;

private double GO_UP_HOLD_ARM_NO_GRAVITY_SPEED_PRECISE = .10; // .3

private double GO_UP_GRAVITY_OUT_OF_FRAME_HIGH_SPEED = .75
        * MAX_DEPLOY_SPEED_2019;

private double GO_UP_GRAVITY_OUT_OF_FRAME_LOW_SPEED = 1.0
        * MAX_DEPLOY_SPEED_2019;

private double GO_UP_GRAVITY_OUT_OF_FRAME_LOW_SPEED_PRECISE = .2; // .6

private double GO_DOWN_HOLD_ARM_NO_GRAVITY_SPEED = -.7
        * MAX_DEPLOY_SPEED_2019;

private double GO_DOWN_HOLD_ARM_NO_GRAVITY_SPEED_PRECISE = -.15; // -.42

private double GO_DOWN_GRAVITY_OUT_OF_FRAME_HIGH_SPEED = -.6
        * MAX_DEPLOY_SPEED_2019;

private double GO_DOWN_GRAVITY_OUT_OF_FRAME_LOW_SPEED = -.5
        * MAX_DEPLOY_SPEED_2019;

private double GO_DOWN_GRAVITY_OUT_OF_FRAME_LOW_SPEED_PRECISE = -.10; // -.3

private double SET_POSITION_SPEED_SCALE_FACTOR = 1.0;

public enum RequiredArmSpeedState
    {
    GO_UP, GO_DOWN, FORCE_DOWN, HOLD, GO_UP_PRECISE, GO_DOWN_PRECISE
    }

/**
 * Allows the user to manually set the arm motor to a speed using
 * the SET_MANUALLY_FOR_CLIMB state. This should not be used outside
 * of autonomous climb functions, where we need to give the
 * manipulator a higher speed than usual.
 *
 * Should be called continously. If this funtion is not called even
 * for one loop through periodic, the state machine will reset the
 * manipulator state to STAY_AT_POSITION
 *
 * TODO add a similar function Forklift
 *
 * @param speed
 *                  the speed the motor will be set to (unscaled)
 *                  This function will scale down the passed in value,
 *                  but until more testing on 2019's robot,
 *                  DO NOT pass in any values greater than 1.0;
 *                  positive values move the arm up, negative
 *                  values move the arm down
 */
public void setArmMotorSpeedManuallyForClimb (double speed)
{
    // If we are trying to move up and past the max angle, or
    // trying to move down and below the min height, tell the
    // arm to stay where it is
    // if ((speed > 0
    // && this.getCurrentArmPosition() > currentDeployMaxAngle)
    // || (speed < 0 && this
    // .getCurrentArmPosition() < currentDeployMinAngle))
    // {
    // this.deployMovementState = DeployMovementState.STAY_AT_POSITION;
    // // return so we exit the method and do not accidentally set
    // // deployMovementState to SET_MANUALLY_FOR_CLIMB;
    // return;
    // }

    deployTargetSpeed = speed;

    this.deployMovementState = DeployMovementState.SET_MANUALLY_FOR_CLIMB;
}

/** Puts various debug into smart dashboard */
public void printDeployDebugInfo ()
{
    SmartDashboard.putString("Arm Potentiometer Raw",
            "" + armPot.get());
    SmartDashboard.putString("Arm Angle Adjusted",
            "" + this.getCurrentArmPosition());
    SmartDashboard.putString("Deploy Movement State",
            "" + this.deployMovementState);
    SmartDashboard.putString("Arm Motor Value",
            "" + this.armMotor.get());
    SmartDashboard.putString("Deploy State",
            "" + this.getDeployState());
    // SmartDashboard.putString("Is Deployed", "" + this.isDeployed());
    // SmartDashboard.putNumber("Left Operator",
    // Hardware.leftOperator.getY());
    // SmartDashboard.putNumber("deployTargetSpeed",
    // deployTargetSpeed);
    // SmartDashboard.putString("stayAtPositionInitIsReady",
    // "" + this.stayAtPositionInitIsReady);
    SmartDashboard.putString("Has Cargo:", "" + this.hasCargo());
    // SmartDashboard.putNumber("currentDeployMaxAngle",
    // currentDeployMaxAngle);
    // SmartDashboard.putNumber("currentDeployMinAngle",
    // currentDeployMinAngle);
    // SmartDashboard.putString("isSetDeployPositionInitReady",
    // "" + isSetDeployPositionInitReady);
    // SmartDashboard.putString("armSolenoid setForward",
    // "" + this.intake.armSolenoid.getForward());
    SmartDashboard.putString("intake isOpen:",
            "" + this.intake.isOpen());
}

// =========================================================================
// Hatch Panel Methods
// =========================================================================





// =========================================================================
// Roller methods
// =========================================================================

// TODO do we just want it so if you hit the override, even without pulling
// trigger, it intakes?
/**
 * Method for calling intake and outtake when one button is for moving the
 * rollers, and the other determines which direction they are being moved in
 *
 * This is private because this control scheme is not the one that the
 * operators want. intakeOuttakeByButtonsSeperated should be used instead.
 *
 * @param intakeButtonValue
 *                                      value of the button used for intake/
 *                                      outtake
 * @param reverseIntakeButtonValue
 *                                      value of the button that, if held, will
 *                                      reverse the direction on the intake
 *                                      motors when the intakeButton is held
 *                                      (causing the manipulator to outake
 *                                      instead of intale when both buttons are
 *                                      held)
 * @param intakeOverrideButtonValue
 *                                      value of the override button for intake,
 *                                      used if the photoSwitch is failing
 */
private void intakeOuttakeByButtons (boolean intakeButtonValue,
        boolean reverseIntakeButtonValue,
        boolean intakeOverrideButtonValue)
{
    this.intake.intakeOuttakeByButtons(intakeButtonValue,
            reverseIntakeButtonValue, intakeOverrideButtonValue);
}


/**
 * Method for calling intake and outtake when they are both mapped to two
 * different buttons. This is in contrast to the intakeOuttakeByButtons
 * method, which has one button for intake, and another that
 * reverses the intake
 *
 * @param intakeButtonValue
 *                                      value of the button used for intake
 * @param reverseIntakeButtonValue
 *                                      value of the button used for outtake
 * @param intakeOverrideButtonValue
 *                                      value of the override button for intake,
 *                                      used if the photoSwitch is failing
 */
public void intakeOuttakeByButtonsSeperated (boolean intakeButtonValue,
        boolean outtakeButtonValue, boolean intakeOverrideButtonValue)
{
    this.intake.intakeOuttakeByButtonsSeperated(intakeButtonValue,
            outtakeButtonValue, intakeOverrideButtonValue);
}


/**
 * Autonomously spins out the cargo based on a timer.
 * For use in autonomous.
 *
 * @return true if the function has finished, false
 *         if it is still going
 */
public boolean spinOutCargoByTimer ()
{
    return this.intake.spinOutCargoByTimer();
}

/**
 * Allows the operator to press a button to manually
 * toggle the manipulator solenoid
 *
 * @param button
 *                   - QuickSwitch that toggles the solenoid
 *                   when pressed
 */
public void toggleSolenoid (QuickSwitch button)
{
    this.intake.toggleSolenoid(button);
}

/**
 * Resets the state machine so the manipulator does not keep trying to run
 * code from a previous enable after a disable. Should be called in teleop
 * init ONLY.
 */
public void resetStateMachine ()
{
    // *resets*
    this.deployMovementState = DeployMovementState.STAY_AT_POSITION;
    this.intake.resetStateMachine();
}

/**
 * @return returns the current deploy min angle
 */
public double getCurrentDeployMinAngle ()
{
    return /* MIN_ARM_POSITION_ADJUSTED; */ currentDeployMinAngle;
}

// =========================================================================
// Constants
// =========================================================================

// used to scale all relevant values for 2019 since the max speed
// the 2019 deploy arm can go is .2
private static double MAX_DEPLOY_SPEED_2019 = .6;

// ----- Joystick Constants 2019 -----
private static final double DEPLOY_JOYSTICK_DEADBAND = 0.2;

// private static double UP_JOYSTICK_SCALER = 1.0 * MAX_DEPLOY_SPEED_2019;

// private static double DOWN_JOYSTICK_SCALER = 0.5
// * MAX_DEPLOY_SPEED_2019;

// ----- Joystick Constants 2018 -----


// ----- Deploy Position Constants 2019 -----

public int MAX_ARM_POSITION_ADJUSTED = 120;

private static int MIN_ARM_POSITION_ADJUSTED = 0;

private static int DEPLOYED_ARM_POSITION_ADJUSTED = 15;

private static int RETRACTED_ARM_POSITION_ADJUSTED = 59;

private double ARM_LEANING_BACK_ANGLE = 90;

private double IS_FULLY_CLEAR_OF_FRAME_ANGLE = 60;

// the maximum angle for the deploy so
public double MAX_FORKLIFT_UP_ANGLE = 70;

private double IS_PARTIALLY_CLEAR_OF_FRAME_ANGLE = 70;

public double HIGHER_MAX_FORKLIFT_UP_ANGLE = 50;

public double FORKLIFT_PARTIALLY_UP_MAX_ANGLE = 86;

private static int PARALLEL_TO_GROUND_ADJUSTED = 0;

// value that the arm pot returns when the manipulator is
// parallel to the floor
private static double ARM_POT_RAW_HORIZONTAL_VALUE = 56;

// vertical angle: 106

private static final double ACCEPTABLE_ERROR = 0.0;

// Temporary values; should be unnecessay on the 2019 robot

// value that is multipled to the value from the arm pot to convert
// it to degrees

// calculate by via the formula: 90/ (Raw Pot Value when arm is vertical -
// Raw Pot Horizontal Value)
private static double ARM_POT_SCALE_TO_DEGREES = 1.11111111; // 90/81

// // value that is multiplied by the number of ticks to convert it to degrees
// private static final double ARM_ENCODER_SCALE_TO_DEGREES = 0.0; //
// placeholder


// ----- Deploy Position Constants 2018 -----


private static final int MAX_ARM_POSITION_ADJUSTED_2018 = 85;

public static final int MIN_ARM_POSITION_ADJUSTED_2018 = 5;// software stop

private static final int RETRACTED_ARM_POSITION_ADJUSTED_2018 = 80;

private static final int DEPLOYED_ARM_POSITION_ADJUSTED_2018 = 10;

private static final int PARALLEL_TO_GROUND_ADJUSTED_2018 = 10;

// private static final int ACCEPTABLE_ERROR = 6;

// Temporary values; should be unnecessay on the 2019 robot

private static final double ARM_POT_RAW_RETRACTED_VALUE_2018 = 45;
// no higher than 70

// value that the arm pot returns when the manipulator is
// parallel to the floor
private static final double ARM_POT_RAW_HORIZONTAL_VALUE_2018 = 225; // placeholder

// value that the arm encoder returns when the manipulator is
// parallel to the floor
private static final double ARM_ENCODER_RAW_HORIZONTAL_VALUE_2018 = 0.0; // placeholder

// value that is multipled to the value from the arm pot to convert
// it to degrees
private static final double ARM_POT_SCALE_TO_DEGREES_2018 = -0.486486; // placeholder

// value that is multiplied by the number of ticks to convert it to degrees
private static final double ARM_ENCODER_SCALE_TO_DEGREES_2018 = 0.0; // placeholder

// ----- Deploy Speed Constants 2019 -----

private static double DEFAULT_KEEP_ARM_UP_SPEED = 0.2
        * MAX_DEPLOY_SPEED_2019;

private static double STAY_UP_WITH_CARGO = 0.5 * MAX_DEPLOY_SPEED_2019;

private static double STAY_UP_NO_PIECE = 0.3 * MAX_DEPLOY_SPEED_2019;

private static double SET_ANGLE_UPWARD_ARM_MOVEMENT_SCALER = 1.0
        * MAX_DEPLOY_SPEED_2019;

private static double SET_ANGLE_DOWNWARD_ARM_MOVEMENT_SCALER = 0.5
        * MAX_DEPLOY_SPEED_2019;

private static double DEFAULT_DEPLOY_SPEED_UNSCALED = 1.0;

private static double DEFAULT_RETRACT_SPEED_UNSCALED = 1.0;

public static double DEFAULT_MOVE_BY_BUTTON_SPEED_UNSCALED = 1.0;

// ----- Deploy Speed Constants 2018 -----

// should be used in 2018 only
private static final double CLOSE_ENOUGH_TO_PARALLEL_DEADBAND = 3;

// should be used in 2018 only
private static final double STAY_AT_PARALLEL_2018 = .1;

private static final double STAY_UP_WITH_CARGO_2018 = 0.3;

private static final double STAY_UP_NO_PIECE_2018 = 0.2;

private static final double UPWARD_ARM_MOVEMENT_SCALER_2018 = .65;

private static final double DOWNWARD_ARM_MOVEMENT_SCALER_2018 = 0.05;

private static final double DEFAULT_DEPLOY_SPEED_UNSCALED_2018 = 1.0;

private static final double DEFAULT_RETRACT_SPEED_UNSCALED_2018 = 1.0;

private static final double DEFAULT_MOVE_BY_BUTTON_SPEED_UNSCALED_2018 = 1.0;

// =========================================================================
// Variables
// =========================================================================

private DeployMovementState deployMovementState = DeployMovementState.STAY_AT_POSITION;

private DeployMovementDirection deployDirection = DeployMovementDirection.NEUTRAL;

// The angle the manipulator is trying to move to; 0 is the start angle,
// positive angles are above the start, negative angles are below the starts
private double deployTargetAngle = 0.0;

private double deployTargetSpeed = 0.0;

private double currentDeployMaxAngle = MAX_ARM_POSITION_ADJUSTED;

private double currentDeployMinAngle = MIN_ARM_POSITION_ADJUSTED;

private double DEMO_MODE_OPEN_MIN_ANGLE = 10;

private boolean isSetDeployPositionInitReady = true;

// 2018 requires different speeds depending on the position of the arm
// this is used to determine whether or not we need to calculate a new
// speed
private boolean stayAtPosition2018InitIsReady = true;

private boolean stayAtPositionInitIsReady = true;

private double stayAtPositionTempSpeed = 0.0;

// =========================================================================
// Tuneables
// =========================================================================

/**
 * Deploy goals:
 *
 * have a set max position and variable min positions (the
 * deploy will be able to go low enough to lift the bottom of
 * the robot up, and we probably don't want to do this
 * until we climb)
 *
 *
 *
 */


}
