package frc.Utils;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.HardwareInterfaces.KilroyEncoder;

/**
 * PathFollower.java
 * 
 * PathFollower is a wrapper class around WPILib's own 2d motion profiling
 * software. This implementation was designed to be as easy to use as possible.
 * 
 * The idea is to be able to set up "waypoints" on the field, and have the robot
 * autnomoustly follow said waypoints. Using splines, the robot does not have to
 * follow "Drive straight, stop, turn, stop, drive straight" method of
 * autonomous, but rather follow a curved spline, resulting in faster and more
 * precise autos.
 * 
 * This class was based on an example from WPILib:
 * https://docs.wpilib.org/en/latest/docs/software/examples-tutorials/trajectory-tutorial/creating-following-trajectory.html
 * 
 * For information on how to automatically tune the PD and kS, kV, kA loop, see
 * this link:
 * https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/robot-characterization/introduction.html
 * 
 * 
 * @written 3/2/2020
 * @author Ryan McGee
 */
class PathFollower extends SubsystemBase
{
  // Generic encoder (canTalon, canSparkMax, DIO)
  private KilroyEncoder leftEnc, rightEnc;
  
  private Gyro gyro;
  
  // WPIlib drive classes
  private DifferentialDriveKinematics tank_kinematics;
  private DifferentialDriveOdometry position_system;
  private DifferentialDrive drive_system;
  
  // Configurations
  private MotionProfile motionConfig;
  private TrajectoryConfig traj_config;
  
  // List of all paths generated during runtime
  private ArrayList<Path> pathList = new ArrayList<Path>(0);
  
  // Since Kilroy uses inches for units, and WPILIB uses meters, do the conversion
  private final double METERS_PER_INCH = 0.0254;
  
  /**
   * Creates a "Path Follower" object. Each PathFollower object created represents
   * a drivetrain and the motion profile connected to it.
   * 
   * @param motionConfig Contains all the motion profile constants, that describe
   *                     how the robot will respond to movement
   * @param leftEnc      The left encoder on a differential drivetrain
   * @param rightEnc     The right encoder on a differential drivetrain
   * @param leftMotors   The group of motors on the left side of a differential
   *                     drivetrain
   * @param rightMotors  The group of motors on the right side of a differential
   *                     drivetrain
   */
  public PathFollower(MotionProfile motionConfig, Gyro gyro_sensor, KilroyEncoder leftEnc, KilroyEncoder rightEnc,
      SpeedControllerGroup leftMotors, SpeedControllerGroup rightMotors)
  {
    this.motionConfig = motionConfig;
    this.gyro = gyro_sensor;
    this.leftEnc = leftEnc;
    this.rightEnc = rightEnc;
    
    // Create the 'kinematics' that can parse data from the encoders into a usable
    // format
    // Create the drive system that will be driving the robot automatically
    this.tank_kinematics = new DifferentialDriveKinematics(motionConfig.trackWidth * METERS_PER_INCH);
    this.drive_system = new DifferentialDrive(leftMotors, rightMotors);
    
    // Constrain the voltage that will be sent to the motors based on the numbers
    // found when
    // profiling the drivetrain
    TrajectoryConstraint voltage_constraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(motionConfig.kS, motionConfig.kV, motionConfig.kA), tank_kinematics, 10);
    
    // Create the trajectory configuration based on the motion profile and above
    // constraints.
    // This will be used when generating a path.
    this.traj_config = new TrajectoryConfig(motionConfig.maxVel, motionConfig.maxAccel);
    this.traj_config.setKinematics(tank_kinematics);
    this.traj_config.addConstraint(voltage_constraint);
    
    // Creates the 'odometry' system: this gives the robot's X/Y position at any
    // given time, based on gyro and encoders
    this.position_system = new DifferentialDriveOdometry(new Rotation2d());
  }
  
  @Override
  public void periodic()
  {
    position_system.update(Rotation2d.fromDegrees(Math.IEEEremainder(gyro.getAngle(), 360.0)),
        leftEnc.getDistance() * METERS_PER_INCH, rightEnc.getDistance() * METERS_PER_INCH);
  }
  
  /**
   * Resets the position of the robot to 0x, 0y, and 0 degrees rotation.
   * 
   * Useful if doing autonomous driving relative to the current position of the
   * robot, rather than relative to the field.
   */
  public void resetPosition()
  {
    this.leftEnc.reset();
    this.rightEnc.reset();
    
    this.position_system.resetPosition(new Pose2d(), new Rotation2d(Math.toRadians(gyro.getAngle())));
  }
  
  /**
   * Generate the robot path. This calculates the left and right trajectories for
   * each side of the robot, and interpolates a number of "segments" that are in
   * between the points specified when creating "Path"
   * 
   * This follows any configurations set up in the "motion profie" while creating
   * the paths, which is why it must be calculated after the robot program has
   * started.
   * 
   * This is based on the example from wpilib found here:
   * https://docs.wpilib.org/en/latest/docs/software/examples-tutorials/trajectory-tutorial/creating-following-trajectory.html
   * 
   * @param robotPath The "Path" to be followed; a list of waypoints and an
   *                  attatched name
   */
  void generatePath(Path robotPath)
  {
    Trajectory tmpTraj = TrajectoryGenerator.generateTrajectory(robotPath.points, traj_config);
    
    // Create the "ramsete" controller, which controls all aspects of the robot
    // moving based on the path. Each "path" has a different ramsete controller.
    robotPath.controller = new RamseteCommand(tmpTraj, () -> position_system.getPoseMeters(), new RamseteController(),
        new SimpleMotorFeedforward(motionConfig.kS, motionConfig.kV, motionConfig.kA), tank_kinematics,
        () -> new DifferentialDriveWheelSpeeds(leftEnc.getRate() * METERS_PER_INCH,
            rightEnc.getRate() * METERS_PER_INCH),
        new PIDController(motionConfig.kP, 0, 0), new PIDController(motionConfig.kP, 0, motionConfig.kD),
        (left, right) -> drive_system.tankDrive(left, right));
    
    pathList.add(robotPath);
  }
  
  /**
   * Searches the list of previously generated paths for one with the name input,
   * and executes that path
   * 
   * Do NOT run this in a loop, it was designed to be run once.
   * 
   * @param pathName The name used when creating the Path object
   */
  void runPath(String pathName)
  {
    for (Path item : pathList)
      if (item.name.equals(pathName) && item.controller != null)
      {
        // Execute only the first path with that name, Conflicting names would be bad!
        item.controller.execute();
        return;
      }
  }
  
  /**
   * Returns whether or not a path with the corresponding name has finished
   * executing. The controller does NOT automatically set the motors to 0 once
   * finished, so remember to do so!
   * 
   * @param pathName Name of the path to check whether or not has finished
   * @return Whether or not the path has finished executing
   */
  boolean isFinished(String pathName)
  {
    for (Path item : pathList)
      if (item.name.equals(pathName) && item.controller != null)
        return item.controller.isFinished();
      
    return false;
  }
  
  /**
   * Stops everything in their path (heh)
   * 
   * This will cancel any running path that has been generated!
   */
  void stopAll()
  {
    for (Path item : pathList)
      if (item.controller != null)
      {
        item.controller.cancel();
        drive_system.stopMotor();
      }
  }
  
  /**
   * A "path", or list of points relative to the field. The first point should be
   * the starting position, and following points are the "ending" points that
   * dictate a single movement.
   * 
   * The second point becomes the end of the first movement, and the start point
   * of the second movement, and so on.
   * 
   * Points are created as such: new Pose2d(x_position, y_position, rotation)
   */
  class Path
  {
    String name;
    List<Pose2d> points;
    
    RamseteCommand controller = null;
    
    /**
     * Create the path.
     * 
     * @param name   Name attatched to the path, which will be used when running the
     *               path later on.
     * @param points A list of points the robot will follow. First point is the
     *               starting position, and the next points will be followed
     *               sequentially.
     */
    public Path(String name, Pose2d... points)
    {
      this.name = name;
      for (Pose2d tmp : points)
        this.points.add(tmp);
    }
  }
  
  /**
   * Motion Profile
   * 
   * Contains variables that relate to the motion of the robot. This information
   * is obtained by using the tool called "FRC-Characterization". This tool will
   * create a robot project that will automatically run the robot forward and
   * backward and gain data from the encoder, and then analyze it from the tool.
   * 
   */
  class MotionProfile
  {
    // Feed-Forward variables
    // kS = positional feedforward
    // kV = velocity feedforward
    // kA = Acceleration feedforward
    public double kS, kV, kA;
    
    // Feed-Back variables
    // kP = Proportional
    // kI = Integral
    // kD = Derivative
    public double kP, kD;
    
    // Maximum velocity and acceleration the
    // robot is allowed to achieve
    // in inches / second, inches / second^2
    public double maxVel, maxAccel;
    
    // Width between the left and right wheels, in inches
    public double trackWidth;
  }
  
}