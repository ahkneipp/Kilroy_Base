package frc.HardwareInterfaces;

import frc.Hardware.Hardware;
import frc.HardwareInterfaces.Transmission.TransmissionBase;
import frc.Utils.drive.Drive;
import frc.vision.VisionProcessor;
import frc.vision.VisionProcessor.ImageType;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.Relay.Value;

/**
 * Contains all game specific vision code, including code to drive to the switch
 * using vision
 *
 * @author: Becky Button and Conner McKevitt
 */
public class DriveWithCamera extends Drive
{

// private TankTransmission tankTransmission = null;

// private MecanumTransmission mecanumTransmission = null;

// private KilroyEncoder leftFrontEncoder = null;
// private KilroyEncoder rightFrontEncoder = null;
// private KilroyEncoder leftRearEncoder = null;
// private KilroyEncoder rightRearEncoder = null;

private UltraSonic frontUltrasonic = null;

// private UltraSonic rearUltrasonic = null;

// private GyroBase gyro = null;

private VisionProcessor visionProcessor = null;

// private final TransmissionType transmissionType;

/**
 * Creates the drive with camera object. If a sensor listed is not used (except
 * for encoders), set it to null.
 *
 *
 * @param transmission
 *                              The robot's transmission object
 * @param leftFrontEncoder
 *                              The left-front corner encoder
 * @param rightFrontEncoder
 *                              The right-front corner encoder
 * @param leftRearEncoder
 *                              The left-rear corner encoder
 * @param rightRearEncoder
 *                              The right-rear corner encoder
 * @param ultrasonic
 *                              The sensor that finds distance using sound
 * @param gyro
 *                              A sensor that uses a spinning disk to measure
 *                              rotation.
 * @param visionProcessor
 *                              The camera's vision processing code, as a
 *                              sensor.
 */
public DriveWithCamera (TransmissionBase transmission,
        KilroyEncoder leftFrontEncoder,
        KilroyEncoder rightFrontEncoder, KilroyEncoder leftRearEncoder,
        KilroyEncoder rightRearEncoder,
        GyroBase gyro, VisionProcessor visionProcessor)
{
    super(transmission, leftFrontEncoder, rightFrontEncoder,
            leftRearEncoder, rightRearEncoder, gyro);

    this.visionProcessor = visionProcessor;
    // this.transmissionType = transmission.getType();
    // this.leftFrontEncoder = leftFrontEncoder;
    // this.rightFrontEncoder = rightFrontEncoder;
    // this.leftRearEncoder = leftRearEncoder;
    // this.rightRearEncoder = rightRearEncoder;
    // this.gyro = gyro;

}

/**
 * Creates drive with camera object
 *
 * @param transmission
 *                            The robot's transmission object
 * @param leftEncoder
 *                            The left encoder
 * @param rightEncoder
 *                            The right encoder
 * @param frontUltrasonic
 *                            The robot's front ultrasonic
 * @param rearUltrasonic
 *                            The robots's read ultrasonic
 * @param gyro
 *                            A sensor that uses a spinning disk to measure
 *                            rotation.
 * @param visionProcessor
 *                            The camera's vision processing code, as a sensor.
 *
 */
public DriveWithCamera (TransmissionBase transmission,
        KilroyEncoder leftEncoder, KilroyEncoder rightEncoder,
        UltraSonic frontUltrasonic, UltraSonic rearUltrasonic,
        GyroBase gyro, VisionProcessor visionProcessor)
{
    super(transmission, leftEncoder, rightEncoder, gyro);



    this.frontUltrasonic = frontUltrasonic;
    // this.rearUltrasonic = rearUltrasonic;
    this.visionProcessor = visionProcessor;
    // this.transmissionType = transmission.getType();
    // this.leftRearEncoder = leftEncoder;
    // this.rightRearEncoder = rightEncoder;
    // this.gyro = gyro;
}

/**
 * Creates drive with camera object
 *
 * @param transmission
 *                            The robot's transmission object
 * @param leftEncoder
 *                            The left encoder
 * @param rightEncoder
 *                            The right encoder
 * @param frontUltrasonic
 *                            The robot's front ultrasonic
 * @param rearUltrasonic
 *                            The robots's read ultrasonic
 * @param gyro
 *                            A sensor that uses a spinning disk to measure
 *                            rotation.
 * @param visionProcessor
 *                            The camera's vision processing code, as a sensor.
 * @param ringlightRelay
 *                            The janky fix for relay not working
 *
 */
public DriveWithCamera (TransmissionBase transmission,
        KilroyEncoder leftEncoder, KilroyEncoder rightEncoder,
        UltraSonic frontUltrasonic, UltraSonic rearUltrasonic,
        GyroBase gyro, VisionProcessor visionProcessor,
        DigitalOutput ringlightRelay)
{
    super(transmission, leftEncoder, rightEncoder, gyro);

    this.frontUltrasonic = frontUltrasonic;
    // this.rearUltrasonic = rearUltrasonic;
    this.visionProcessor = visionProcessor;
    // this.transmissionType = transmission.getType();
    // this.leftRearEncoder = leftEncoder;
    // this.rightRearEncoder = rightEncoder;
    // this.gyro = gyro;
}

/**
 * Drives using the camera until it hits CAMERA_NO_LONGER_WORKS inches, where it
 * then drives using the ultrasonic until the stopping distance
 *
 * Multiply the compensationFactor by speed to determine what values we are
 * sending to the motor controller
 *
 * Used a compensation factor to slow down as we get closer to the target
 *
 *
 * @param speed
 *                  have the speed greater than 0 and less than 1
 * @return true if the robot has driven all the way to the front of the target,
 *         and false if it hasn't
 */
public boolean driveToTarget (double speed, boolean cargoAuto)
{
    // System.out.println("vision state: " + state);
    switch (state)
        {
        case INIT:
            Hardware.axisCamera.processImage();
            // Hardware.axisCamera.setRelayValue(Value.kOn);
            Hardware.drive.resetEncoders();
            visionProcessor.saveImage(ImageType.RAW);
            visionProcessor.saveImage(ImageType.PROCESSED);

            double correctionValue = DRIVE_CORRECTION;
            double motorspeed = speed;
            double slowAmount;
            double slowestSpeed;

            state = DriveWithCameraState.DRIVE_WITH_CAMERA;

            break;
        case DRIVE_WITH_CAMERA:
            // Hardware.axisCamera.processImage();// TODO check and see if this
            // cause lag or watchdog errors
            visionProcessor.saveImage(ImageType.RAW);
            visionProcessor.saveImage(ImageType.PROCESSED);
            correctionValue = DRIVE_CORRECTION;

            // visionProcessor.saveImage(ImageType.RAW);
            // visionProcessor.saveImage(ImageType.PROCESSED);
            // adjust speed based on distance
            // System.out.println("ultrasonic distance: "
            // + this.frontUltrasonic
            // .getDistanceFromNearestBumper());

            // if we lose camera before aligned
            /*
             * if (this.frontUltrasonic
             * .getDistanceFromNearestBumper() <= CAMERA_NO_LONGER_WORKS
             * && (Hardware.rightFrontDriveEncoder
             * .getDistance() >= MIN_INCHES
             * || Hardware.leftFrontDriveEncoder
             * .getDistance() >= MIN_INCHES))
             * {
             *
             * state = DriveWithCameraState.DRIVE_WITH_US;
             * }
             */

            // if we get close enought to the target and have to stop
            if (!cargoAuto)
                {
                if (this.frontUltrasonic
                        .getDistanceFromNearestBumper() <= DISTANCE_FROM_WALL_TO_STOP
                        && Hardware.rightFrontDriveEncoder
                                .getDistance() >= MIN_INCHES)
                    {
                    state = DriveWithCameraState.STOP;
                    }
                }
            else
                {
                if (this.frontUltrasonic
                        .getDistanceFromNearestBumper() <= DISTANCE_FROM_WALL_TO_STOP_CARGO)
                    {
                    state = DriveWithCameraState.STOP;
                    }
                }

            if (this.frontUltrasonic
                    .getDistanceFromNearestBumper() < DISTANCE_FROM_WALL_TO_SLOW1
                    && this.frontUltrasonic
                            .getDistanceFromNearestBumper() >= DISTANCE_FROM_WALL_TO_SLOW2)
                {
                slowAmount = SLOW_MODIFIER;
                correctionValue = DRIVE_CORRECTION * SLOW_MODIFIER;
                }
            else
                if (this.frontUltrasonic
                        .getDistanceFromNearestBumper() < DISTANCE_FROM_WALL_TO_SLOW2)
                    {
                    slowAmount = SLOW_MODIFIER * SLOW_MODIFIER;
                    correctionValue = DRIVE_CORRECTION * SLOW_MODIFIER;
                    }
                else
                    {
                    slowAmount = 1;
                    }
            // System.out.println("slow amount: " + slowAmount);

            motorspeed = speed * slowAmount;

            // adjust speed so that motors never reverse
            if (motorspeed - DRIVE_CORRECTION <= 0)
                {
                slowestSpeed = 0.05;
                }
            else
                {
                slowestSpeed = motorspeed - DRIVE_CORRECTION;
                }

            // System.out.println("right speed: "
            // + Hardware.rightFrontCANMotor.get());
            // System.out.println("left speed: "
            // + Hardware.leftFrontCANMotor.get());




            // gets the position of the center
            double centerX = this.getCameraCenterValue();
            // turns on the ring light




            // if the switch center is to the right of our center set by the
            // SWITCH_CAMERA_CENTER, correct by driving faster on the left
            if (centerX >= SWITCH_CAMERA_CENTER - CAMERA_DEADBAND)
                {
                // the switch's center is too far right, drive faster on the
                // left

                // System.out.println("WE ARE TOO LEFT");
                this.getTransmission().driveRaw(
                        motorspeed + correctionValue,
                        slowestSpeed);

                }
            // if the switch center is to the left of our center set by the
            // SWITCH_CAMERA_CENTER, correct by driving faster on the right

            else
                if (centerX <= SWITCH_CAMERA_CENTER + CAMERA_DEADBAND)
                    {
                    // the switch's center is too far left, drive faster on the
                    // right
                    // System.out.println("WE ARE TOO RIGHT");
                    this.getTransmission().driveRaw(slowestSpeed,
                            motorspeed + correctionValue);

                    }
                else
                    {
                    // System.out.println(
                    // "Driving straight center of blobs");
                    driveStraight(motorspeed, 2, true);
                    }



            break;
        case DRIVE_WITH_US:


            driveStraight(speed, 2, true);


            // take a picture when we start to drive with ultrasonic

            if (this.frontUltrasonic
                    .getDistanceFromNearestBumper() <= DISTANCE_FROM_WALL_TO_STOP
            /*
             * (Hardware.leftFrontDriveEncoder
             * .getDistance() >= MIN_INCHES
             * || Hardware.rightFrontDriveEncoder
             * .getDistance() >= MIN_INCHES)
             */)
                {

                visionProcessor.saveImage(ImageType.RAW);
                visionProcessor.saveImage(ImageType.PROCESSED);
                state = DriveWithCameraState.STOP;
                }

            break;
        default:
        case STOP:
            // Hardware.autoTimer.stop();

            // Hardware.axisCamera.setRelayValue(Value.kOff);
            // if we are too close to the wall, brake, then set all motors to
            // zero, else drive by ultrasonic
            // System.out.println("We are stopping");
            this.getTransmission().driveRaw(0, 0);
            state = DriveWithCameraState.INIT;
            return true;
        }
    return false;
}

/**
 * drives to target with speed and compensation lower to allow for aligning from
 * close distances.
 *
 *
 * @param speed
 * @param stopClose
 *                      true if you want to stop close to the target, flase if
 *                      not
 * @return true when completed
 */
public boolean driveToTargetClose (double speed, boolean stopClose)
{
    // Deprecated comment until we can tesxt blob area
    // make this not use the ultrasonic. I am thinking that we lower the
    // adjustment value each time we change directions. This ways we can retrun
    // true when we align and not rely on the ultrasonic. Also need to make it
    // override by a button.
    // System.out.println("vision state: " + state);
    switch (state)
        {
        case INIT:
            Hardware.axisCamera.processImage();
            Hardware.drive.resetEncoders();

            double correctionValue = (DRIVE_CORRECTION_CLOSE / .1)
                    * speed;// porpotion based off of tested code at .1 speed

            double motorspeed = speed;
            state = DriveWithCameraState.DRIVE_WITH_CAMERA;
            break;
        case DRIVE_WITH_CAMERA:
            correctionValue = DRIVE_CORRECTION_CLOSE;
            // Hardware.axisCamera.processImage();// TODO
            visionProcessor.saveImage(ImageType.RAW);
            visionProcessor.saveImage(ImageType.PROCESSED);
            if (Hardware.axisCamera.hasBlobs() == true)
                {
                // System.out.println("area of blob" + Hardware.axisCamera
                // .getNthSizeBlob(0).area);
                if (stopClose)
                    {
                    if ((Hardware.frontUltraSonic
                            .getDistanceFromNearestBumper() > Hardware.retriever.LENGTH_OF_NESSIE_HEAD
                                    + 3
                            || Hardware.axisCamera
                                    .getNthSizeBlob(
                                            0).area > MIN_BLOB_AREA)
                            && (Hardware.leftFrontDriveEncoder
                                    .getDistance() > MIN_INCHES_CLOSE
                                    || Hardware.rightFrontDriveEncoder
                                            .getDistance() > MIN_INCHES_CLOSE))
                        {
                        System.out.println("stopped by blobs");
                        state = DriveWithCameraState.STOP;
                        }
                    }
                else
                    {
                    if ((Hardware.axisCamera
                            .getNthSizeBlob(
                                    0).area > MIN_BLOB_AREA_CLOSE
                            || Hardware.frontUltraSonic
                                    .getDistanceFromNearestBumper() > Hardware.retriever.LENGTH_OF_NESSIE_HEAD
                                            + 3)
                            && (Hardware.leftFrontDriveEncoder
                                    .getDistance() > MIN_INCHES_CLOSE
                                    || Hardware.rightFrontDriveEncoder
                                            .getDistance() > MIN_INCHES_CLOSE))
                        {
                        System.out.println("stopped by blobs");
                        state = DriveWithCameraState.STOP;
                        }
                    }
                }
            if (Hardware.whichRobot == Hardware.RobotYear.KILROY_2019)
                {
                correctionValue = (DRIVE_CORRECTION_CLOSE / .1)
                        * speed;// porpotion based off of tested code at .1
                                // speed
                }
            else
                if (Hardware.whichRobot == Hardware.RobotYear.KILROY_2018)
                    {
                    correctionValue = (DRIVE_CORRECTION_CLOSE_2018
                            / .1)
                            * speed;// porpotion based off of tested code at .1
                                    // speed
                    }

            motorspeed = speed;

            // adjust speed based on distance

            // if we get close enought to the target and have to stop
            // gets the position of the center
            double centerX = this.getCameraCenterValue();
            // turns on the ring light

            // if the switch center is to the right of our center set by the
            // SWITCH_CAMERA_CENTER, correct by driving faster on the left
            if (centerX >= SWITCH_CAMERA_CENTER - CAMERA_DEADBAND)
                {
                // the switch's center is too far right, drive faster on the
                // left
                // System.out.println("too right");
                this.getTransmission().driveRaw(
                        motorspeed + correctionValue,// TODO
                        motorspeed/* - correctionValue */);
                }
            // if the switch center is to the left of our center set by the
            // SWITCH_CAMERA_CENTER, correct by driving faster on the right
            else
                if (centerX <= SWITCH_CAMERA_CENTER + CAMERA_DEADBAND)
                    {
                    // the switch's center is too far left, drive faster on the
                    // right
                    // System.out.println("too left");
                    this.getTransmission().driveRaw(
                            motorspeed/* - correctionValue */,
                            motorspeed + correctionValue);// TODO
                    }
                else
                    {
                    this.getTransmission().driveRaw(.3, .3);
                    }
            break;
        default:
        case STOP:
            // Hardware.axisCamera.setRelayValue(Value.kOff);
            // if we are too close to the wall, brake, then set all motors to
            // zero, else drive by ultrasonic

            state = DriveWithCameraState.INIT;
            this.getTransmission().driveRaw(0, 0);

            return true;
        }
    return false;
}

/**
 * Code for 2019 camera to align to the rocket. returns Side that the camera see
 * vision targets on
 *
 * @author Conner McKevitt
 * @return Side
 */
public Side getTargetSide ()
{
    Hardware.axisCamera.processImage();
    if (this.getCameraCenterValue() < SWITCH_CAMERA_CENTER
            - CAMERA_DEADBAND)
        {
        side = Side.RIGHT;
        return side;
        }
    else
        if (this.getCameraCenterValue() > SWITCH_CAMERA_CENTER
                + CAMERA_DEADBAND)
            {
            side = Side.LEFT;
            return side;
            }
        else
            if (this.getCameraCenterValue() > SWITCH_CAMERA_CENTER
                    - CAMERA_DEADBAND
                    && this.getCameraCenterValue() < SWITCH_CAMERA_CENTER
                            + CAMERA_DEADBAND)
                {
                side = Side.CENTER;
                return side;
                }
    side = Side.NULL;
    return side;
}

public static enum Side
    {
    RIGHT, LEFT, NULL, CENTER
    }

private Side side = Side.NULL;

public DriveWithCameraState state = DriveWithCameraState.INIT;

// private boolean takePicture = false;

public enum DriveWithCameraState
    {
    INIT, DRIVE_WITH_CAMERA, DRIVE_WITH_US, STOP
    }

/**
 * Drives using the camera until it hits CAMERA_NO_LONGER_WORKS inches, where it
 * then drives using the ultrasonic, uses the janky relay fix
 *
 * Multiply the compensationFactor by speed to determine what values we are
 * sending to the motor controller
 *
 * @param compensationFactor
 *                               have the compensation factor greater than 1 and
 *                               less than 1.8
 * @param speed
 *                               have the speed greater than 0 and less than 1
 * @return true if the robot has driven all the way to the front of the scale,
 *         and false if it hasn't
 */




/**
 * Method to test a new way to align with vision
 *
 * @param speed
 * @param compensationFactor
 */




// private int currentPictureIteration = 0;

/**
 * Gets the center x value of of the vision targets (average of the x values of
 * both visions targets)
 *
 * @return the current center x value
 */
public double getCameraCenterValue ()
{
    Hardware.axisCamera.setRelayValue(Value.kOn);
    double center = 0;

    visionProcessor.processImage();
    // if we have at least two blobs, the center is equal to the average
    // center
    // x position of the 1st and second largest blobs
    if (visionProcessor.getParticleReports().length >= 2)
        {
        center = (visionProcessor.getNthSizeBlob(0).center.x
                + visionProcessor.getNthSizeBlob(1).center.x) / 2;


        // System.out.println("TWO BLOBS");
        // System.out.println("blob center: " + center);
        }
    // if we only can detect one blob, the center is equal to the center x
    // position of the blob
    else
        if (visionProcessor.getParticleReports().length == 1)
            {
            center = visionProcessor.getNthSizeBlob(0).center.x;
            // System.out.println("ONE BLOBS");
            // System.out.println("blob center: " + center);
            }
        // if we don't have any blobs, set the center equal to the constanct
        // center,
        // we can use this to just drive straight
        else
            {
            center = SWITCH_CAMERA_CENTER;
            // System.out.println("NO BLOBS");
            }
    return center;
}



// ================VISION CONSTANTS================


// the distance in inches in which we drive the robot straight using the
// ultrasonic
private final double CAMERA_NO_LONGER_WORKS = 0;

// The minimum encoder distance we must drive before we enable the ultrasonic
// private final double ENCODER_MIN_DISTANCE = 50; // inches
// 38 + 50;

// the number in pixels that the center we are looking for can be off
private final double CAMERA_DEADBAND = 15;

// the distance from the wall (in inches) where we start stopping the robot
private final double DISTANCE_FROM_WALL_TO_STOP = 15;

// length of nessie head + drive forward distance
private final double DISTANCE_FROM_WALL_TO_STOP_CARGO = 29;

private final double DISTANCE_FROM_WALL_TO_SLOW1 = 100;

private final double DISTANCE_FROM_WALL_TO_SLOW2 = 60;

// private final double DISTANCE_FROM_WALL_TO_SLOW_CLOSE = 30;

private final double SLOW_MODIFIER = .75;// lower for slower


private final double SWITCH_CAMERA_CENTER = 160;// Center of a 320x240 image
// 160 originally

private final double DRIVE_CORRECTION = .15;

private final double DRIVE_CORRECTION_CLOSE = .1;

private final double DRIVE_CORRECTION_CLOSE_2018 = .5;


private static double MIN_BLOB_AREA_CLOSE = 600;// TODO

private static double MIN_BLOB_AREA = 900;// TODO

private final double MIN_INCHES = 50;

private final double MIN_INCHES_CLOSE = 15;

}
