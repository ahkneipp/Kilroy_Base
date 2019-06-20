package frc.Utils;

import frc.Hardware.Hardware;
import frc.HardwareInterfaces.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.HardwareInterfaces.KilroyEncoder;
import frc.HardwareInterfaces.RobotPotentiometer;
import frc.HardwareInterfaces.UltraSonic;
import frc.Utils.drive.Drive;
import frc.robot.Teleop;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;

public class ClimbToLevelTwo
{
// swing arm double solenoid to lift back frame/drop back wheels
private DoubleSolenoid driveSolenoid = null;

// motor that rotates the arm DO NOT USE
private SpeedController armMotor = null;

// encoder in the event that we change from a pot to an encoder DO NOT USE
private KilroyEncoder armEncoder = null;

// potentiometer to sense the ticks of the arm, can be replaced with an encoder
// DO NOT USE
private RobotPotentiometer armSensor = null;

// private DoubleSolenoid testSolenoid = null;

// drive system to...wait for it...drive
private Drive drive = null;

// lift thats an object of the Forklift class
private Forklift lift = null;

// ultrasonic that senses the distance to the front til the nearest object
private UltraSonic ultraSonic = null;

// timer used to time the various functions of climb
private Timer climbTimer = new Timer();

// not actually used but TODO change over newClimb() to this timer
private Timer newClimbTimer = new Timer();

// timer to deal with how long we need to drive
private Timer driveTimer = new Timer();

// states of the climb state machine
public static enum ClimberState
    {
    STANDBY, START_CLIMB, LOWER_FORKLIFT_TO_POSITION, DELAY_ONE, LOWER_ARM, DELAY_TWO, DEPLOY_BACK_WHEELS, DELAY_THREE, LOWER_FORKLIFT_COMPLETELY, DELAY_FOUR, DRIVE_FORWARD, DELAY_FIVE, RAISE_ARM, DELAY_SIX, RETRACT_WHEELS, DELAY_SEVEN, FINISH_DRIVING, DELAY_INIT, STOP
    }


// initializes climb state and prev climb state to standby
public static ClimberState climbState = ClimberState.STANDBY;

private static ClimberState prevState = ClimberState.STANDBY;


/**
 * default construstor sets everything to null so we know to fix ourselves and
 * pass in all of our objects
 */
public ClimbToLevelTwo ()
{
    this.driveSolenoid = null;
    this.armMotor = null;
    this.armSensor = null;
    this.armEncoder = null;
    this.drive = null;
    this.lift = null;
    this.ultraSonic = null;
}

/**
 * Constructor for testing on Frogger, swaps out the single solenoid for a test
 * double solenoid
 *
 * @author Ashley Espeland
 * @param-testSolenoid
 *                     solenoid to simulate the drive solenoids to lower the
 *                     back wheels
 *
 * @param-armMotor
 *                 motor controller that controls the nessie head's movements
 *                 about a point
 *
 * @param-armEncoder
 *                   encoder that stands in for the armPot that will be on the
 *                   new robot
 * @param-drive
 *              drive object that obviously drves and deals with all that
 * @param-lift
 *             Forklift object that controlls the forklift's up and down motions
 * @param- ultraSonic
 *         // * Ultrasonic that is mounted on the front of the robot and used to
 *         tell
 *         // * us when to stop based on how far away from the wall we are
 *         // *
 *         //
 */
// public ClimbToLevelTwo (DoubleSolenoid testSolenoid,
// SpeedController armMotor, RobotPotentiometer armSensor,
// Drive drive,
// Forklift lift, UltraSonic ultraSonic)
// {

// this.testSolenoid = testSolenoid;
// this.armMotor = armMotor;
// this.armSensor = armSensor;
// this.drive = drive;
// this.lift = lift;
// this.ultraSonic = ultraSonic;
// }


/**
 * Constructor for the new robot- has armPot and single solenoid
 *
 * @author Ashley Espeland
 * @param -driveSolenoid
 *            single solenoid to lower and raise the back wheels
 *
 * @param -armMotor
 *            motor controller that controls the nessie head's movements
 *            about a point
 *
 * @param -armPot
 *            potentiometer that reads the nessie head's position
 *
 * @param -drive
 *            drive object that obviously drves and deals with all that
 *
 * @param -lift
 *            Forklift object that controlls the forklift's up and down motions
 *
 * @param -ultraSonic
 *            Ultrasonic that is mounted on the front of the robot and used to
 *            tell
 *            us when to stop based on how far away from the wall we are
 *
 */
public ClimbToLevelTwo (DoubleSolenoid driveSolenoid,
        SpeedController armMotor, RobotPotentiometer armSensor,
        Drive drive, Forklift lift, UltraSonic ultraSonic)
{
    // assigns the parameters to the appropriate variables
    this.driveSolenoid = driveSolenoid;
    this.armMotor = armMotor;
    this.armSensor = armSensor;
    this.drive = drive;
    this.lift = lift;
    this.ultraSonic = ultraSonic;
}



// state of the new climb state machine
public static enum NewClimberState
    {
    STANDBY, START_CLIMB, PREP_ARM, BACK_UP_TIL_BUMPERS_HIT, DELAY_ONE, DEPLOY_BACK_WHEELS, DELAY_TWO, BACK_UP_TIL_REAR_WHEELS_HIT, DELAY_THREE, RETRACT_WHEELS, DELAY_FOUR, BACK_UP_TIL_MID_WHEELS_ON_PLATFORM, DELAY_FIVE, POWERED_ARM_DOWN, DELAY_SIX, DRIVE_BACKWARDS_ONTO_PLATFORM, DELAY_SEVEN, RAISE_ARM, STOP, FINISH
    }


// initializes new climb state to standby
public static NewClimberState newClimbState = NewClimberState.STANDBY;

public static NewClimberState prevNewClimbState = NewClimberState.STANDBY;

/**
 *
 * @author Ashley Espeland
 *
 *         function that sets the newClimbState to startClimb and essentially is
 *         the catalyst for climbing
 */
public void newClimb ()
{
    newClimbState = NewClimberState.START_CLIMB;
    driveTimerInit();
}

public void skipStepClimb ()
{
    newClimbState = NewClimberState.BACK_UP_TIL_BUMPERS_HIT;
    driveTimerInit();
}

int counter = 0;

/**
 * @author Ashley Espeland
 *
 *         must be called periodically in order for newClimb() to work, runs the
 *         state machine and contains all that you really need to have in terms
 *         of the procession of events needed to climb
 *
 *         stays in standby until newClimb() is called which starts the chain of
 *         events
 *
 *         when the process is finished it will shut down every component and
 *         then loop back into stand by
 *
 *         finishEarly() has been modified to deal with this new version of
 *         climb
 *         and will send it to stop and then finish and then loop to standby in
 *         order to possibly rerun the climb code
 *
 */
public void newClimbUpdate ()
{
    // good for debugging
    // System.out.println(newClimbState);
    if (Hardware.climber.newClimbState != Hardware.climber.prevNewClimbState)
        {
        System.out.println(
                "newClimbState " + Hardware.climber.newClimbState);
        prevNewClimbState = Hardware.climber.newClimbState;
        }

    switch (newClimbState)
        {
        case STANDBY:
            // state to wait in during the match until climb() or reverseClimb()
            // is called
            // testSolenoid.set(Value.kForward);

            break;

        case START_CLIMB:
            // state the climb() function sets the state to and is basically an
            if (counter == 0)
                {
                Hardware.lift.resetStateMachine();
                counter++;
                }
            // init state
            Hardware.drive.setGearPercentage(Teleop.FIRST_GEAR_NUMBER,
                    1.0);
            Hardware.drive.setGearPercentage(Teleop.SECOND_GEAR_NUMBER,
                    1.0);

            // TODO @CR @ANE change to 0?
            if (this.lift.setLiftPositionPrecise(0.5, 1.0) == true)
                {
                newClimbState = NewClimberState.PREP_ARM;
                driveTimerInit();
                counter = 0;
                break;
                }
            break;
        case PREP_ARM:
            if (this.lowerArm() == true)
                {
                newClimbState = NewClimberState.BACK_UP_TIL_BUMPERS_HIT;
                }
            break;

        case BACK_UP_TIL_BUMPERS_HIT:
            // lowers forklift to the first position needed to climb, not all
            // the way down but mostly dwn
            if (this.backUpTilBumpersHit() == true)
                {
                // moves on to delay init and then DELAY_ONE
                this.delayInit();
                newClimbState = NewClimberState.DELAY_ONE;
                }
            break;

        case DELAY_ONE:
            // first delay state after LOWER_FORKLIFT_TO_POSITION and before
            // LOWER_ARM
            if (climbTimer.get() >= NEW_DELAY_ONE_TIME)
                {
                climbTimer.stop();
                newClimbState = NewClimberState.DEPLOY_BACK_WHEELS;
                }
            break;

        case DEPLOY_BACK_WHEELS:
            // lowers nessie head all the way
            if (this.deployBackWheels() == true)
                {
                // goes to DELAY_TWO after DELAY_INIT
                this.delayInit();
                newClimbState = NewClimberState.DELAY_TWO;
                }
            break;

        case DELAY_TWO:
            // delay state after LOWER_ARM and before DEPLOY_BACK_WHEELS
            if (climbTimer.get() >= NEW_DELAY_TWO_TIME)
                {
                // sets state to
                climbTimer.stop();
                driveTimerInit();
                newClimbState = NewClimberState.BACK_UP_TIL_REAR_WHEELS_HIT;
                }
            break;

        case BACK_UP_TIL_REAR_WHEELS_HIT:
            if (this.backUpTilRearWheelsHit() == true)
                {
                // Goes to DELAY_THREE
                this.delayInit();
                newClimbState = NewClimberState.DELAY_THREE;
                }
            break;

        case DELAY_THREE:
            if (climbTimer.get() >= NEW_DELAY_THREE_TIME)
                {
                climbTimer.stop();
                driveTimerInit();
                newClimbState = NewClimberState.RETRACT_WHEELS;
                }
            break;

        case RETRACT_WHEELS:
            if (this.retractWheels() == true)
                {
                // goes to delay four
                this.delayInit();
                newClimbState = NewClimberState.DELAY_FOUR;
                }
            break;

        case DELAY_FOUR:
            if (climbTimer.get() >= NEW_DELAY_FOUR_TIME)
                {
                climbTimer.stop();
                newClimbState = NewClimberState.BACK_UP_TIL_MID_WHEELS_ON_PLATFORM;
                }
            break;

        case BACK_UP_TIL_MID_WHEELS_ON_PLATFORM:

            if (this.backUpTilMidWheelsOnPlatform() == true)
                {
                // goes to DELAY_FIVE
                this.delayInit();
                newClimbState = NewClimberState.DELAY_FIVE;
                }
            break;

        case DELAY_FIVE:
            if (climbTimer.get() >= NEW_DELAY_FIVE_TIME)
                {
                climbTimer.stop();
                newClimbState = NewClimberState.DRIVE_BACKWARDS_ONTO_PLATFORM;// .POWERED_ARM_DOWN;
                }
            break;

        case POWERED_ARM_DOWN:
            // raises nessie head all the way
            if (this.poweredArmDown() == true)
                {
                // goes to DELAY_SIX
                this.delayInit();
                newClimbState = NewClimberState.DELAY_SIX;
                }
            break;

        case DELAY_SIX:
            // delay state after
            if (climbTimer.get() >= NEW_DELAY_SIX_TIME)
                {
                climbTimer.stop();
                driveTimerInit();
                newClimbState = NewClimberState.FINISH;// .DRIVE_BACKWARDS_ONTO_PLATFORM;
                }
            break;

        case DRIVE_BACKWARDS_ONTO_PLATFORM:

            if (this.driveBackwardsOntoPlatform() == true)
                {
                // goes to DELAY_SEVEN
                this.delayInit();
                newClimbState = NewClimberState.DELAY_SEVEN;
                }
            break;

        case DELAY_SEVEN:
            // delay after RETRACT_WHEELS and before FINISH_DRIVING
            if (climbTimer.get() >= NEW_DELAY_SEVEN_TIME)
                {
                climbTimer.stop();
                climbTimer.reset();
                climbTimer.start();
                newClimbState = NewClimberState.RAISE_ARM;
                }
            break;

        case RAISE_ARM:
            //
            if (this.raiseArm() == true || climbTimer.get() >= 3.0)
                {
                // goes to stop
                this.delayInit();
                newClimbState = NewClimberState.STOP;
                }
            break;

        case STOP:
            // stops stuff
            Hardware.drive.setGearPercentage(Teleop.FIRST_GEAR_NUMBER,
                    Teleop.FIRST_GEAR_RATIO_KILROY_XX);
            Hardware.drive.setGearPercentage(Teleop.SECOND_GEAR_NUMBER,
                    Teleop.SECOND_GEAR_RATIO_KILROY_XX);
            this.stop();
            newClimbState = NewClimberState.FINISH;
            break;
        case FINISH:
            System.out.println("WE'VE CLIMBED MOUNT EVEREST!!!");
            newClimbState = NewClimberState.STANDBY;
            break;
        // welp heres a default just in case
        default:
            System.out.println("DEFAULT CLIMB CASE REACHED");
            break;
        }
}



// method that deals with the states of climbing
// should be called wherever climb() or reverseClimb() is used otherwise they
// will not work
public void climbUpdate ()
{
    // System.out.println(climbState);


    switch (climbState)
        {
        case STANDBY:
            // state to wait in during the match until climb() or reverseClimb()
            // is called
            // testSolenoid.set(Value.kForward);
            break;

        case START_CLIMB:
            // state the climb() function sets the state to and is basically an
            // init state
            Hardware.drive.setGearPercentage(Teleop.FIRST_GEAR_NUMBER,
                    1.0);
            Hardware.drive.setGearPercentage(Teleop.SECOND_GEAR_NUMBER,
                    1.0);
            climbState = ClimberState.LOWER_FORKLIFT_TO_POSITION;
            break;

        case LOWER_FORKLIFT_TO_POSITION:
            // lowers forklift to the first position needed to climb, not all
            // the way down but mostly dwn
            if (this.lowerForkliftToPosition() == true)
                {
                // moves on to delay init and then DELAY_ONE
                prevState = ClimberState.LOWER_FORKLIFT_TO_POSITION;
                climbState = ClimberState.DELAY_INIT;
                }
            break;

        case DELAY_ONE:
            // first delay state after LOWER_FORKLIFT_TO_POSITION and before
            // LOWER_ARM
            if (climbTimer.get() >= DELAY_ONE_TIME)
                {
                // sets state to LOWER_ARM
                climbTimer.stop();
                prevState = ClimberState.DELAY_ONE;
                // Hardware.intakeDeploySensor.reset();
                climbState = ClimberState.LOWER_ARM;
                }
            break;

        case LOWER_ARM:
            // lowers nessie head all the way
            if (this.lowerArm() == true)
                {
                // goes to DELAY_TWO after DELAY_INIT
                prevState = ClimberState.LOWER_ARM;
                climbState = ClimberState.DELAY_INIT;
                }
            break;

        case DELAY_TWO:
            // delay state after LOWER_ARM and before DEPLOY_BAK_WHEELS
            if (climbTimer.get() >= DELAY_TWO_TIME)
                {
                // sets state to DEPLOY_BACK_WHEELS
                climbTimer.stop();
                prevState = ClimberState.DELAY_TWO;
                climbState = ClimberState.DEPLOY_BACK_WHEELS;
                }
            break;

        case DEPLOY_BACK_WHEELS:
            this.holdArmToSupportDrive();
            // deploys back wheels
            if (this.deployBackWheels() == true)
                {
                // Goes to DELAY_THREE
                prevState = ClimberState.DEPLOY_BACK_WHEELS;
                climbState = ClimberState.DELAY_INIT;
                }
            break;

        case DELAY_THREE:
            this.holdArmToSupportDrive();
            // delay state after DEPLOY_BACK_WHEELS and before
            // LOWER_FORKLFT_COMPLETELY
            if (climbTimer.get() >= DELAY_THREE_TIME)
                {
                // goes to LOWER_FORKLIFT_COMPLETELY
                climbTimer.stop();
                prevState = ClimberState.DELAY_THREE;
                climbState = ClimberState.LOWER_FORKLIFT_COMPLETELY;
                }
            break;

        case LOWER_FORKLIFT_COMPLETELY:
            this.holdArmToSupportDrive();
            // lowers forklift all the way and lifts the front end of the robot
            // subsequentially
            // this.holdArmToSupportDrive();
            if (this.lowerForkliftCompletely() == true)
                {
                // goes to delay four
                prevState = ClimberState.LOWER_FORKLIFT_COMPLETELY;
                climbState = ClimberState.DELAY_INIT;
                }
            break;

        case DELAY_FOUR:
            this.holdArmToSupportDrive();
            // delay after LOWER_FORKLIFT_COMPLETELY and before DRIVE_FORWARD
            // this.holdArmToSupportDrive();
            if (climbTimer.get() >= DELAY_FOUR_TIME)
                {
                // goes to DRIVE_FORWARD
                climbTimer.stop();
                prevState = ClimberState.DELAY_FOUR;
                climbState = ClimberState.DRIVE_FORWARD;
                }
            break;

        case DRIVE_FORWARD:
            this.holdArmToSupportDrive();
            // drives forward to the point where the the rear wheels can be
            // retracted
            // this.holdArmToSupportDrive();
            if (this.driveForward() == true)
                {
                // goes to DELAY_FIVE
                prevState = ClimberState.DRIVE_FORWARD;
                climbState = ClimberState.DELAY_INIT;
                }
            break;

        case DELAY_FIVE:
            this.holdArmToSupportDrive();
            // dely state after DRIVE_FORWARD and before RAISE_ARM
            if (climbTimer.get() >= DELAY_FIVE_TIME)
                {
                // goes to RAISE_ARM
                climbTimer.stop();
                prevState = ClimberState.DELAY_FIVE;
                // Hardware.intakeDeploySensor.reset();
                climbState = ClimberState.RAISE_ARM;
                }
            break;

        case RAISE_ARM:
            // raises nessie head all the way
            if (this.raiseArm() == true)
                {
                // goes to DELAY_SIX
                prevState = ClimberState.RAISE_ARM;
                climbState = ClimberState.DELAY_INIT;
                }
            break;

        case DELAY_SIX:
            // delay state after RAISE_ARM ad before RETRACT_WHEELS
            if (climbTimer.get() >= DELAY_SIX_TIME)
                {
                // goes to RETRACT_WHEELS
                climbTimer.stop();
                prevState = ClimberState.DELAY_SIX;
                climbState = ClimberState.RETRACT_WHEELS;
                }
            break;

        case RETRACT_WHEELS:
            // retracts wheels
            if (this.retractWheels() == true)
                {
                // goes to DELAY_SEVEN
                prevState = ClimberState.RETRACT_WHEELS;
                climbState = ClimberState.DELAY_INIT;
                }
            break;

        case DELAY_SEVEN:
            // delay after RETRACT_WHEELS and before FINISH_DRIVING
            if (climbTimer.get() >= DELAY_SEVEN_TIME)
                {
                // goes to FINISH_DRIVING
                climbTimer.stop();
                prevState = ClimberState.DELAY_SEVEN;
                climbState = ClimberState.FINISH_DRIVING;
                }
            break;

        case FINISH_DRIVING:
            // drives until we get close enough to the wall
            if (this.finishDriving() == true)
                {
                // goes to stop
                prevState = ClimberState.FINISH_DRIVING;
                climbState = ClimberState.STOP;
                }
            break;

        case STOP:
            // stops stuff
            Hardware.drive.setGearPercentage(Teleop.FIRST_GEAR_NUMBER,
                    Teleop.FIRST_GEAR_RATIO_KILROY_XX);
            Hardware.drive.setGearPercentage(Teleop.SECOND_GEAR_NUMBER,
                    Teleop.SECOND_GEAR_RATIO_KILROY_XX);
            this.stop();
            break;

        case DELAY_INIT:
            // state to initialize all the delay states
            climbTimer.reset();
            climbTimer.start();

            // determines the next state to go to based in the previous state
            if (prevState == ClimberState.LOWER_FORKLIFT_TO_POSITION)
                {
                climbState = ClimberState.DELAY_ONE;
                }
            else
                if (prevState == ClimberState.LOWER_ARM)
                    {
                    climbState = ClimberState.DELAY_TWO;
                    }
                else
                    if (prevState == ClimberState.DEPLOY_BACK_WHEELS)
                        {
                        climbState = ClimberState.DELAY_THREE;
                        }
                    else
                        if (prevState == ClimberState.LOWER_FORKLIFT_COMPLETELY)
                            {
                            climbState = ClimberState.DELAY_FOUR;
                            }
                        else
                            if (prevState == ClimberState.DRIVE_FORWARD)
                                {
                                climbState = ClimberState.DELAY_FIVE;
                                }
                            else
                                if (prevState == ClimberState.RAISE_ARM)
                                    {
                                    climbState = ClimberState.DELAY_SIX;
                                    }
                                else
                                    if (prevState == ClimberState.RETRACT_WHEELS)
                                        {
                                        climbState = ClimberState.DELAY_SEVEN;
                                        }
            break;

        // welp heres a default just in case
        default:
            System.out.println("DEFAULT CLIMB CASE REACHED");
            break;
        }

}

/**
 * method to call in teleop to climb onto the platform when you are already
 * lined up with the platform
 *
 */
public void climb ()
{
    // if (climbState == climberState.STANDBY)
    // {
    climbState = ClimberState.START_CLIMB;
    // }
}


// state of the climb state machine
private static enum ReverseClimberState
    {
    STANDBY, START_REVERSE_CLIMB, DRIVE_BACKWARDS, DELAY_ONE, DEPLOY_BACK_WHEELS, DELAY_TWO, LOWER_FORKLIFT_COMPLETELY, DELAY_THREE, LOWER_ARM, DELAY_FOUR, DRIVE_OFF_COMPLETELY, DELAY_FIVE, RAISE_FORKLIFT_TO_POSITION, DELAY_SIX, RETRACT_WHEELS, DELAY_SEVEN, RAISE_ARM, DELAY_INIT, STOP, FINISH;
    }


// initializes climb state and prev climb state to standby
private static ReverseClimberState reverseClimbState = ReverseClimberState.STANDBY;

private static ReverseClimberState prevReverseState = ReverseClimberState.STANDBY;


/**
 * method that encompasses the reverseClimb state machine, must be called in
 * order for reverseClimb() to work
 */

// must reorganize @ANE
public void reverseClimbUpdate ()
{
    // System.out.println(reverseClimbState);

    switch (reverseClimbState)
        {
        case STANDBY:
            // state to wait in during the match until climb() or reverseClimb()
            // is called
            driveSolenoid.set(Value.kForward);
            break;

        case START_REVERSE_CLIMB:
            // state the reverseClimb() function sets the state to and is
            // basically an
            // init state
            reverseClimbState = ReverseClimberState.DRIVE_BACKWARDS;
            break;


        case DRIVE_BACKWARDS:
            // drives until we get far enough from the wall
            if (this.driveBackwards() == true)
                {
                // goes to DELAY_ONE
                prevReverseState = ReverseClimberState.DRIVE_BACKWARDS;
                reverseClimbState = ReverseClimberState.DELAY_INIT;
                }
            break;

        case DELAY_ONE:
            // first delay state after DRIVE_BACKWARDS and before
            // DEPLOY_BACK_WHEELS
            if (climbTimer.get() >= DELAY_ONE_TIME)
                {
                // sets state to DEPLOY_BACK_WHEELS
                climbTimer.stop();
                prevReverseState = ReverseClimberState.DELAY_ONE;
                // Hardware.intakeDeployEncoder.reset();
                reverseClimbState = ReverseClimberState.DEPLOY_BACK_WHEELS;
                }
            break;

        case DEPLOY_BACK_WHEELS:
            // deploys back wheels
            if (this.deployBackWheels() == true)
                {
                // Goes to DELAY_TWO
                prevReverseState = ReverseClimberState.DEPLOY_BACK_WHEELS;
                reverseClimbState = ReverseClimberState.DELAY_INIT;
                }
            break;

        case DELAY_TWO:
            // delay state after DEPLOY_BACK_WHEELS and before
            // LIFT_FORKLIFT_COMPLETELY
            if (climbTimer.get() >= DELAY_TWO_TIME)
                {
                // sets state to LOWER_FORKLIFT_COMPLETELY
                climbTimer.stop();
                prevReverseState = ReverseClimberState.DELAY_TWO;
                reverseClimbState = ReverseClimberState.LOWER_FORKLIFT_COMPLETELY;
                }
            break;

        case LOWER_FORKLIFT_COMPLETELY:
            // lowers forklift all the way and lifts the front end of the robot
            // subsequentially
            if (this.lowerForkliftCompletely() == true)
                {
                // goes to DELAY_THREE
                prevReverseState = ReverseClimberState.LOWER_FORKLIFT_COMPLETELY;
                reverseClimbState = ReverseClimberState.DELAY_INIT;
                }
            break;

        case DELAY_THREE:
            // delay state after LOWER_FORKLFT_COMPLETELY and before LOWER_ARM
            if (climbTimer.get() >= DELAY_THREE_TIME)
                {
                // goes to LOWER_ARM
                climbTimer.stop();
                prevReverseState = ReverseClimberState.DELAY_THREE;
                reverseClimbState = ReverseClimberState.LOWER_ARM;
                }
            break;


        case LOWER_ARM:
            // lowers nessie head all the way
            if (Hardware.manipulator.deployArm() == true)
                {
                // goes to DELAY_FOUR
                prevReverseState = ReverseClimberState.LOWER_ARM;
                reverseClimbState = ReverseClimberState.DELAY_INIT;
                }
            break;


        case DELAY_FOUR:
            this.holdArmToSupportDrive();
            // delay after LOWER_ARM and before DRIVE_OFF_COMPLETELY
            if (climbTimer.get() >= DELAY_FOUR_TIME)
                {
                // goes to DRIVE_OFF_COMPLETELY
                climbTimer.stop();
                prevReverseState = ReverseClimberState.DELAY_FOUR;
                reverseClimbState = ReverseClimberState.DRIVE_OFF_COMPLETELY;
                }
            break;

        case DRIVE_OFF_COMPLETELY:
            this.holdArmToSupportDrive();
            // drives backwards to the point where the the rear wheels can be
            // retracted
            if (this.reverseDriveOffCompletely() == true)
                {
                // goes to DELAY_FIVE
                prevReverseState = ReverseClimberState.DRIVE_OFF_COMPLETELY;
                reverseClimbState = ReverseClimberState.DELAY_INIT;
                }
            break;

        case DELAY_FIVE:
            this.holdArmToSupportDrive();
            // delay state after DRIVE_FORWARD and before RETRACT_WHEELS
            if (climbTimer.get() >= DELAY_FIVE_TIME)
                {
                // goes to RAISE_ARM
                climbTimer.stop();
                prevReverseState = ReverseClimberState.DELAY_FIVE;
                // Hardware.intakeDeployEncoder.reset();
                reverseClimbState = ReverseClimberState.RETRACT_WHEELS;
                }
            break;

        case RETRACT_WHEELS:
            this.holdArmToSupportDrive();
            // retracts wheels
            if (this.retractWheels() == true)
                {
                // goes to DELAY_SIX
                prevReverseState = ReverseClimberState.RETRACT_WHEELS;
                reverseClimbState = ReverseClimberState.DELAY_INIT;
                }
            break;

        case DELAY_SIX:
            this.holdArmToSupportDrive();
            // delay state after RETRACT_WHEELS and before RAISE_ARM
            if (climbTimer.get() >= DELAY_SIX_TIME)
                {
                // goes to RAISE_ARM
                climbTimer.stop();
                prevReverseState = ReverseClimberState.DELAY_SIX;
                reverseClimbState = ReverseClimberState.RAISE_FORKLIFT_TO_POSITION;
                }
            break;


        case RAISE_FORKLIFT_TO_POSITION:
            this.holdArmToSupportDrive();
            // raises forklift to the position needed to lower robot, not all
            // the way up

            if (this.lowerForkliftToPosition() == true)
                {
                // moves on to STOP
                prevReverseState = ReverseClimberState.RAISE_FORKLIFT_TO_POSITION;
                reverseClimbState = ReverseClimberState.DELAY_INIT;
                }
            break;

        case DELAY_SEVEN:
            this.holdArmToSupportDrive();
            // delay after RAISE_FORKLIFT_TO_POSITION and before RAISE_ARM
            if (climbTimer.get() >= DELAY_SEVEN_TIME)
                {
                // goes to RAISE_FORKLIFT_TO_POSITION
                climbTimer.stop();
                prevReverseState = ReverseClimberState.DELAY_SEVEN;
                reverseClimbState = ReverseClimberState.RAISE_ARM;
                }
            break;

        case RAISE_ARM:
            // raises nessie head all the way
            if (Hardware.manipulator.retractArm() == true)
                {
                // goes to DELAY_SEVEN
                prevReverseState = ReverseClimberState.RAISE_ARM;
                reverseClimbState = ReverseClimberState.STOP;
                }

        case STOP:
            // stops stuff
            this.stop();
            if (finishedEarly == true)
                {
                climbState = ClimberState.STANDBY;
                }
            break;

        case DELAY_INIT:
            // state to initialize all the delay states
            climbTimer.reset();
            climbTimer.start();

            // determines the next state to go to based in the previous state
            if (prevReverseState == ReverseClimberState.DRIVE_BACKWARDS)
                {
                reverseClimbState = ReverseClimberState.DELAY_ONE;
                }
            else
                if (prevReverseState == ReverseClimberState.DEPLOY_BACK_WHEELS)
                    {
                    reverseClimbState = ReverseClimberState.DELAY_TWO;
                    }
                else
                    if (prevReverseState == ReverseClimberState.LOWER_FORKLIFT_COMPLETELY)
                        {
                        reverseClimbState = ReverseClimberState.DELAY_THREE;
                        }
                    else
                        if (prevReverseState == ReverseClimberState.LOWER_ARM)
                            {
                            reverseClimbState = ReverseClimberState.DELAY_FOUR;
                            }
                        else
                            if (prevReverseState == ReverseClimberState.DRIVE_OFF_COMPLETELY)
                                {
                                reverseClimbState = ReverseClimberState.DELAY_FIVE;
                                }
                            else
                                if (prevReverseState == ReverseClimberState.RETRACT_WHEELS)
                                    {
                                    reverseClimbState = ReverseClimberState.DELAY_SIX;
                                    }
                                else
                                    if (prevReverseState == ReverseClimberState.RAISE_FORKLIFT_TO_POSITION)
                                        {
                                        reverseClimbState = ReverseClimberState.DELAY_SEVEN;
                                        }
            break;

        case FINISH:
            this.stop();
            // System.out.println(
            // "WE HAVE FINISHED WHAT WE WANTED TO OF REVERSE CLIMB");
            break;


        // welp heres a default just in case
        default:
            System.out.println("DEFAULT CLIMB CASE REACHED");
            break;
        }

}

/**
 * method to get down from the platform by running the states of climb in
 * reverse
 */
public void reverseClimb ()
{
    reverseClimbState = ReverseClimberState.START_REVERSE_CLIMB;
}


/**
 * method to lower forklift to the height where wecn start the sequence of
 * climbing on to the second level
 */
private boolean lowerForkliftToPosition ()
{
    // System.out.println(liftEncoder.get());
    // if (liftEncoder.get() >= LIFT_HEIGHT_TO_START_CLIMB) {
    // liftMotor.set(LOWER_LIFT_SPEED);
    // } else {
    // liftMotor.set(0.0);
    // System.out.println("Trying to lower forklift to position ");
    if (this.lift.setLiftPosition(LIFT_HEIGHT_TO_START_CLIMB) == true)
        {
        return true;
        }
    return false;

}

/**
 * method to lower arm to the position needed to climb- all the way down
 */
private boolean lowerArm ()
{
    // checks the position of the arm and if it has not gone far enough, sets
    // the motor to the sped to lower the arm
    // System.out.println("Trying to lower arm");
    // if (this.armSensor.get() <= LOWERED_ARM_POSITION)
    // {
    // armMotor.set(LOWER_ARM_SPEED);
    // }
    // else
    // {
    // armMotor.set(0.0);
    // return true;
    // }
    // TODO @ANE @CR moveArmToPsotion with only one parameter passes in an old,
    // slow value for the downward movements
    if (Hardware.manipulator.moveArmToPosition(
            GamePieceManipulator.MIN_ARM_POSITION_ADJUSTED_2018) == true)
        {
        return true;
        }
    return false;
}

private void holdArmToSupportDrive ()
{
    Hardware.manipulator
            .setArmMotorSpeedManuallyForClimb(SUPPORT_DRIVE_ARM_SPEED);
}

/**
 * method that calls the forklift method and sets it to the the min position
 *
 * @return whether it has completed the movement
 */
private boolean lowerForkliftCompletely ()
{
    // System.out.println("Trying to lower forklift completely");
    // if (liftEncoder.get() >= MIN_LIFT_HEIGHT_TO_CLIMB) {
    // liftMotor.set(LOWER_LIFT_SPEED);
    // } else {
    // liftMotor.set(0.0);
    // }
    // armMotor.set(ARM_HOLD_SPEED);
    if (lift.setLiftPosition(MIN_LIFT_HEIGHT_TO_CLIMB,
            LOWER_LIFT_SPEED))
        {
        return true;
        }
    return false;
}

/**
 * method to lower the back wheels by pneumatics in order to climb
 *
 * @return whether the maction has been completed
 */
private boolean deployBackWheels ()
{
    // System.out.println("Trying to deploy back wheels");
    // driveSolenoid.setForward(LOWER_WHEELS_POSITION);
    Hardware.driveSolenoid.setForward(false);
    // sets test solnoid to a position
    // testSolenoid.set(Value.kReverse);
    return true;

}

/**
 * drives to the point that we can raise our wheels and continue climbing
 *
 * @return- whether its been completed
 */
private boolean driveForward ()
{
    // drive forward a set distance
    // System.out.println("Trying to drive forward");
    if (drive.driveStraightInches(DISTANCE_TO_DRIVE_B4_RETRACTION_NORM,
            SPEED_TO_DRIVE_UP, ACCELERATION_TIME, false) == true)
        {
        return true;
        }
    return false;
}

/**
 * method to raise arm all the way back
 *
 * @return- whether it has been completed
 */
private boolean raiseArm ()
{
    // checks to see if the arm has gone far enough, if not it sets th ar to the
    // raise arm speed
    // System.out.println("Trying to raise arm");
    // if (armSensor.get() >= RAISED_ARM_POSITION)
    // {
    // armMotor.set(RAISE_ARM_SPEED);
    // }
    // else
    // {
    // armMotor.set(0.0);
    // return true;
    // }
    if (Hardware.manipulator.retractArm() == true)
        {
        return true;
        }

    return false;
}

/**
 * retracts the wheels
 *
 * @return - whether its completed
 */
private boolean retractWheels ()
{
    // sets the solenoid to the correct position
    // System.out.println("Trying to retract wheels");
    // driveSolenoid.setForward(RETRACT_WHEELS_POSITION);
    Hardware.driveSolenoid.setForward(true);
    // testSolenoid.set(Value.kForward);
    return true;
}


/**
 * drives until the ultraSonic reads the right value
 *
 * @return - whether it has been completed
 */
private boolean finishDriving ()
{
    // checks th ultrasonic and if it does not red the right value it drives
    // forward
    // System.out.println("Trying to finish driving");
    if (this.ultraSonic
            .getDistanceFromNearestBumper() >= DISTANCE_B4_STOPPING)
        {
        this.drive.driveStraight(SPEED_TO_FINISH_DRIVING, .6, false);
        return false;
        }
    else
        {
        stop();
        return true;
        }

}


// reverse specific methods

public boolean driveBackwards ()
{
    if (ultraSonic
            .getDistanceFromNearestBumper() >= DISTANCE_TO_DRIVE_B4_DEPLOYMENT)
        {
        drive.stop();
        return true;
        }
    else
        {
        drive.drive(SPEED_TO_FINISH_REVERSE_DRIVING,
                SPEED_TO_FINISH_REVERSE_DRIVING);
        return false;
        }

}


public boolean reverseDriveOffCompletely ()
{
    if (drive.driveStraightInches(DISTANCE_BEFORE_END_REVERSE_CLIMB,
            SPEED_TO_FINISH_REVERSE_CLIMB, ACCELERATION_TIME,
            true) == true)
        {
        drive.stop();
        return true;
        }
    return false;
}

// -------------------------------------------------------------------------
// NEW CLIMB FUNCTIONS
// -------------------------------------------------------------------------

/**
 * function to back up and straighten out on the wall in order to be lind up to
 * climb using the new method
 */
public boolean backUpTilBumpersHit ()
{
    if (driveTimer.get() >= TIME_TO_BACK_UP_TIL_BUMPERS_HIT)
        {
        drive.stop();
        return true;
        }
    else
        {
        drive.driveStraight(SPEED_BACK_UP_TIL_BUMPERS_HIT, 0.0, true);
        return false;
        }
}


/**
 * function to back up while the wheels are retracted til the point where the
 * rear wheels hit the wall to climb using the new method
 */
public boolean backUpTilRearWheelsHit ()
{
    if (driveTimer.get() >= TIME_TO_DRIVE_BACKWARDS_ONTO_PLATFORM)
        {
        drive.stop();
        return true;
        }
    else
        {
        drive.driveStraight(SPEED_DRIVE_BACKWARDS_ONTO_PLATFORM, 0.0,
                true);
        return false;
        }
}



/**
 * drive backwards until the second set of wheels are on the platform and
 * actually have traction
 */
public boolean backUpTilMidWheelsOnPlatform ()
{
    if (driveTimer.get() >= TIME_TO_BACK_UP_TIL_MID_WHEELS_ON_PLATFORM)
        {
        drive.stop();
        return true;
        }
    else
        {
        drive.driveStraight(SPEED_BACK_UP_TIL_MID_WHEELS_ON_PLATFORM,
                0.0, true);
        return false;
        }
}


/**
 * gives increased power in order to lower the arm and rock the robot onto the
 * platform while the solenoids are retracted and we have two wheels on the
 *
 * runs for a set amount of time
 */
public boolean poweredArmDown ()
{

    if ((Hardware.leftBackIR.isOn() == true)
            || (Hardware.rightBackIR.isOn() == true)
            || (driveTimer.get() >= TIME_TO_POWER_ARM_DOWN))
        {
        Hardware.manipulator.setArmMotorSpeedManuallyForClimb(0.0);

        return true;
        }
    else
        {
        this.holdArmToSupportDrive();
        return false;
        }
    // if (Hardware.manipulator
    // .getCurrentArmPosition() >= Hardware.manipulator
    // .getCurrentDeployMinAngle())
    // {
    // this.holdArmToSupportDrive();
    // return false;
    // }
    // else
    // {
    // return true;
    // }

}

/**
 * drive backwards until we rock back on to the platform and slam against the
 * wall
 */
public boolean driveBackwardsOntoPlatform ()
{


    if (driveTimer.get() >= delayBeforeDriveFromPower
            && driveTimer.get() <= TIME_TO_POWER_ARM_DOWN)
        {
        drive.driveStraight(SPEED_DRIVE_BACKWARDS_ONTO_PLATFORM, 0.0,
                true);
        }
    else
        {
        drive.stop();
        }

    if (driveTimer.get() <= TIME_TO_POWER_ARM_DOWN)
        {
        this.holdArmToSupportDrive();
        return false;
        }
    else
        {
        return true;
        }

    // if (driveTimer.get() >= TIME_TO_DRIVE_BACKWARDS_ONTO_PLATFORM)
    // {
    // drive.stop();
    // return true;
    // }
    // else
    // {
    // drive.driveStraight(SPEED_DRIVE_BACKWARDS_ONTO_PLATFORM, 0.0,
    // true);
    // return false;
    // }
}
// -------------------------------------------------------------------------

/**
 * stops all the various mechanisms so we just sit there on the platform
 */
private void stop ()
{
    // System.out.println("Trying to stop");
    drive.stop();
    Hardware.manipulator.setDeployMovementState(
            GamePieceManipulator.DeployMovementState.STAY_AT_POSITION);
    lift.liftState = Forklift.ForkliftState.STAY_AT_POSITION;
    driveSolenoid.setForward(RETRACT_WHEELS_POSITION);
}

/**
 * resets and starts delay timer to use in delay states
 */
public void delayInit ()
{
    // resets and starts delayTimer
    this.climbTimer.reset();
    this.climbTimer.start();
}

/**
 * resets and starts the driveTimer
 */
public void driveTimerInit ()
{
    // resets and starts timer
    driveTimer.reset();
    driveTimer.start();
}



/**
 * method to end climbing early
 */
public void finishEarly ()
{
    // sets state to STOP
    finishedEarly = true;
    climbState = ClimberState.STOP;
    newClimbState = NewClimberState.STOP;
}


// =======================================================================
//

// ---------------------------------------------
// Constants
// ---------------------------------------------

// Positions
private static final double LOWERED_ARM_POSITION = 260;

private static final double RAISED_ARM_POSITION = 225;

private static final double LIFT_HEIGHT_TO_START_CLIMB = 10.0;

private static final double MIN_LIFT_HEIGHT_TO_CLIMB = 8.0;

private static final Boolean LOWER_WHEELS_POSITION = false;

private static final Boolean RETRACT_WHEELS_POSITION = true;


// SPEEDS

// private static final double RAISE_ARM_SPEED = .7;

// private static final double LOWER_ARM_SPEED = -.4;

private static final double ARM_HOLD_SPEED = 1.0;

// temporary placeholder value
private static final double SUPPORT_DRIVE_ARM_SPEED = -.5;

private static final double SPEED_TO_DRIVE_UP = .4;

private static final double SPEED_TO_REVERSE_DRIVE_OFF = -.4;

private static final double SPEED_TO_FINISH_DRIVING = .4;

private static final double SPEED_TO_FINISH_REVERSE_DRIVING = -.4;

private static final double LOWER_LIFT_SPEED = .3;

private static final double SPEED_TO_FINISH_REVERSE_CLIMB = -.4;

// DISTANCES

private static final int DISTANCE_B4_STOPPING = 10;

private static final int DISTANCE_TO_DRIVE_B4_RETRACTION_NORM = 20;

private static final int DISTANCE_TO_DRIVE_B4_DEPLOYMENT = 20;

private static final int DISTANCE_BEFORE_END_REVERSE_CLIMB = 10;


// TIMES

private static final double ACCELERATION_TIME = .2;

private static final double DELAY_ONE_TIME = 0.0;

private static final double DELAY_TWO_TIME = 0.0;

private static final double DELAY_THREE_TIME = 5.0;

private static final double DELAY_FOUR_TIME = 0.0;

private static final double DELAY_FIVE_TIME = 0.0;

private static final double DELAY_SIX_TIME = 0.0;

private static final double DELAY_SEVEN_TIME = 1.0;


// Extra tuneable stuff

public static boolean finishedEarly = false;

public static boolean reverseClimb = false;


// new climb stuff
public static double SPEED_BACK_UP_TIL_BUMPERS_HIT = -.25;

public static double TIME_TO_BACK_UP_TIL_BUMPERS_HIT = .25;
// --------------------------------

public static double SPEED_BACK_UP_TIL_REAR_WHEELS_HIT = -.25;// .3; changed at
                                                              // 8:20 2/18

public static double TIME_TO_BACK_UP_TIL_REAR_WHEELS_HIT = .5;
// --------------------------------

public static double SPEED_BACK_UP_TIL_MID_WHEELS_ON_PLATFORM = -.4;// .5;
                                                                    // changed
                                                                    // at 8:20
                                                                    // 2/18

public static double TIME_TO_BACK_UP_TIL_MID_WHEELS_ON_PLATFORM = .8;
// --------------------------------

public static double SPEED_DRIVE_BACKWARDS_ONTO_PLATFORM = -.55;// @ANE 3/2/19
                                                                // 10:05

public static double TIME_TO_DRIVE_BACKWARDS_ONTO_PLATFORM = 1.0;

private static final double TIME_TO_POWER_ARM_DOWN = 2.5;
// --------------------------------

private static final double NEW_DELAY_ONE_TIME = 0.0;

private static final double NEW_DELAY_TWO_TIME = 1.25;// after solenoids

private static final double NEW_DELAY_THREE_TIME = 0.5;

private static final double NEW_DELAY_FOUR_TIME = 0.75;

private static final double NEW_DELAY_FIVE_TIME = 0.2;

private static final double NEW_DELAY_SIX_TIME = 0.0;

private static final double NEW_DELAY_SEVEN_TIME = 0.4;

private static final double delayBeforeDriveFromPower = .5;

}
