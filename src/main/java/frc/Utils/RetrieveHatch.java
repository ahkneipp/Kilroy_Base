package frc.Utils;

import edu.wpi.first.wpilibj.Timer;
import frc.Hardware.Hardware;
import frc.HardwareInterfaces.DriveWithCamera.DriveWithCameraState;
import frc.Utils.RollerIntakeMechanism.IntakeState;

public class RetrieveHatch
{
public static enum RetrievalState
    {
    STANDBY, START_RETRIEVAL, ALIGN_VISION, DOUBLE_CHECK_POSITIONS, DELAY_ONE, DRIVE_FORWARD, DELAY_TWO, LIFT_FORKLIFT_TO_GET_HATCH, DELAY_THREE, BACKUP_SLOWLY, STOP, FINISH
    }

public static RetrievalState retrievalState = RetrievalState.STANDBY;

public static Timer delayTimer = new Timer();


public static void retrieveHatch ()
{
    retrievalState = RetrievalState.START_RETRIEVAL;

}


public void retrievalUpdate ()
{

    // System.out.println(
    // "retrieval state " + Hardware.retriever.retrievalState);
    switch (retrievalState)
        {
        case STANDBY:

            break;

        case START_RETRIEVAL:
            // initiates the retrieval sequence
            retrievalState = RetrievalState.DOUBLE_CHECK_POSITIONS;
            break;
        case ALIGN_VISION:

            if (!armIsReady)
                {
                prepareArmPositions();
                }
            // prepareArmPositions();
            if (alignWithVision(.15))
                {
                retrievalState = RetrievalState.DELAY_ONE;
                }
            break;

        case DOUBLE_CHECK_POSITIONS:
            // double checks that the forklift and the manipulator are in the
            // right positions and moves them to the right positions if theyre
            // not in the right positions
            if (forkliftHeightReached == false && Hardware.lift
                    .setLiftPositionPrecise(
                            LIFT_HEIGHT_TO_RETRIEVE_HATCH,
                            Forklift.DEFAULT_TELEOP_BUTTON_SPEED_UNSCALED) == true)
                {
                forkliftHeightReached = true;
                }

            // only moves the manipulator if it hasn't already reached it's
            // target position
            if (manipulatorAngleReached == false && Hardware.manipulator
                    .moveArmToPositionPrecise(
                            Forklift.PLAYER_STATION_ANGLE) == true)
                {
                manipulatorAngleReached = true;
                }

            if (forkliftHeightReached == true
                    && manipulatorAngleReached == true)
                {
                delayInit();
                forkliftHeightReached = false;
                manipulatorAngleReached = false;
                retrievalState = RetrievalState.DELAY_ONE;
                break;
                }

            // if (forkliftHeightReached == true && Hardware.manipulator
            // .moveArmToPositionPrecise(
            // Forklift.PLAYER_STATION_ANGLE) == true)
            // {
            // delayInit();
            // forkliftHeightReached = false;
            // retrievalState = RetrievalState.DELAY_ONE;
            // break;
            // }
            break;

        case DELAY_ONE:
            // if (!armIsReady)
            // {
            // prepareArmPositions();
            // }
            if (delayTimer.get() >= DELAY_ONE_TIME)
                {
                delayTimer.stop();
                Hardware.manipulator.setIntakeState(IntakeState.INTAKE);
                retrievalState = RetrievalState.DRIVE_FORWARD;
                }
            break;

        case DRIVE_FORWARD:
            // if (!armIsReady)
            // {
            // prepareArmPositions();
            // }
            Hardware.manipulator.setIntakeState(IntakeState.INTAKE);
            if ((Hardware.drive.driveStraightInches(
                    DISTANCE_TO_DRIVE_FORWARD, SPEED_TO_DRIVE_FORWARD,
                    0.0, true)) /* && armIsReady */)
                {
                delayInit();
                retrievalState = RetrievalState.DELAY_THREE;
                }
            break;

        case DELAY_TWO:
            Hardware.manipulator.setIntakeState(IntakeState.INTAKE);
            if (delayTimer.get() >= DELAY_TWO_TIME)
                {
                delayTimer.stop();
                // Hardware.manipulator
                // .setIntakeState(IntakeState.OUTTAKE);
                retrievalState = RetrievalState.LIFT_FORKLIFT_TO_GET_HATCH;
                }
            break;

        case LIFT_FORKLIFT_TO_GET_HATCH:
            // if (Hardware.lift.setLiftPosition(
            // LIFT_HEIGHT_TO_PULL_BACK_HATCH) == true)
            // {
            delayInit();
            retrievalState = RetrievalState.DELAY_THREE;
            // }
            break;

        case DELAY_THREE:
            if (delayTimer.get() >= DELAY_THREE_TIME)
                {
                delayTimer.stop();

                if (usingPartialRetrieve == true)
                    retrievalState = RetrievalState.STOP;
                else
                    retrievalState = RetrievalState.BACKUP_SLOWLY;
                }
            break;

        case BACKUP_SLOWLY:
            // Hardware.manipulator.setIntakeState(IntakeState.OUTTAKE);
            if (Hardware.drive.driveStraightInches(
                    DISTANCE_TO_BACKUP_SLOWLY, SPEED_TO_BACKUP_SLOWLY,
                    0, true))
                {
                retrievalState = RetrievalState.STOP;
                }
            break;

        case STOP:
            stop();
            retrievalState = RetrievalState.FINISH;
            break;

        case FINISH:
            retrievalState = RetrievalState.STANDBY;
            break;


        }

}


public void stopRetrieveHatch ()
{
    retrievalState = RetrievalState.STOP;
}

private boolean usingPartialRetrieve = true;


public void prepareArmPositions ()
{
    // double checks that the forklift and the manipulator are in the
    // right positions and moves them to the right positions if theyre
    // not in the right positions
    if (forkliftHeightReached == false && Hardware.lift
            .setLiftPositionPrecise(
                    LIFT_HEIGHT_TO_RETRIEVE_HATCH,
                    Forklift.DEFAULT_TELEOP_BUTTON_SPEED_UNSCALED) == true)
        {
        forkliftHeightReached = true;
        }

    // only moves the manipulator if it hasn't already reached it's
    // target position
    if (manipulatorAngleReached == false && Hardware.manipulator
            .moveArmToPositionPrecise(
                    Forklift.PLAYER_STATION_ANGLE) == true)
        {
        manipulatorAngleReached = true;
        }

    if (forkliftHeightReached == true
            && manipulatorAngleReached == true)
        {
        delayInit();
        forkliftHeightReached = false;
        manipulatorAngleReached = false;
        armIsReady = true;
        }

}

/**
 * resets and starts delay timer to use in delay states
 */
public void delayInit ()
{
    // resets and starts delayTimer
    delayTimer.reset();
    delayTimer.start();
}

public void stop ()
{

    armIsReady = false;
    retrievalState = RetrievalState.FINISH;
    Hardware.drive.stop();
    Hardware.lift.resetStateMachine();
    Hardware.manipulator.resetStateMachine();

}

public boolean alignWithVision (double speed)
{

    if (Hardware.driveWithCamera.driveToTargetClose(.15, false))
        {

        Hardware.alignAndStopButton.setValue(false);
        Hardware.driveWithCamera.state = DriveWithCameraState.INIT;
        return true;
        }

    return false;
}


// CONSTANTS AND MISC
public boolean armIsReady = false;

public static double DELAY_ONE_TIME = 0.0;

public static double DELAY_TWO_TIME = 0.0;

public static double DELAY_THREE_TIME = 1.0;

// DISTANCES
public static double DISTANCE_TO_DRIVE_FORWARD = 11.0;

public static double LENGTH_OF_NESSIE_HEAD = 25.0;

public static double DISTANCE_TO_STOP_ALIGN = DISTANCE_TO_DRIVE_FORWARD
        + LENGTH_OF_NESSIE_HEAD;

public static double DISTANCE_TO_RETRIEVE = DISTANCE_TO_DRIVE_FORWARD
        + LENGTH_OF_NESSIE_HEAD;

public static double DISTANCE_TO_BACKUP_SLOWLY = 4.0;

// POSITIONS
public static double LIFT_HEIGHT_TO_RETRIEVE_HATCH = Forklift.PLAYER_STATION_HEIGHT;// lift
// height to go
// into the hatch

public static double LIFT_HEIGHT_TO_PULL_BACK_HATCH = LIFT_HEIGHT_TO_RETRIEVE_HATCH
        + 3.0;// height to get in the center of the hatch plus the height to
              // move up the lift to actually have the hatch resting on the hook

// SPEEDS
public static double SPEED_TO_BACKUP_SLOWLY = -0.25;

public static double SPEED_TO_DRIVE_FORWARD = 0.15;

// BOOLEANS

public static boolean forkliftHeightReached = false;

public static boolean manipulatorAngleReached = false;

}
