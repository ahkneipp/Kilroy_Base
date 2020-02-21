package frc.Utils;

import java.util.List;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import frc.HardwareInterfaces.KilroyEncoder;


class PathFollower
{
  private SpeedControllerGroup leftMotors, rightMotors;
  private KilroyEncoder leftEnc, rightEnc;
  private Gyro gyro;
  
  private DifferentialDriveKinematics tank_kinematics;
  private DifferentialDriveOdometry position_system;
  private DifferentialDrive drive_system;
  private RamseteController auto_drive_system;

  private MotionProfile motionConfig;
  private TrajectoryConfig traj_config;

  private boolean move_init = true;
  
  // Since Kilroy uses inches for units, and WPILIB uses meters, do the conversion
  private final double METERS_PER_INCH = 1.0 / 39.3701;
  
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
    this.leftMotors = leftMotors;
    this.rightMotors = rightMotors;
    
    // Create the 'kinematics' that can parse data from the encoders into a usable format
    // Create the drive system that will be driving the robot automatically
    this.tank_kinematics = new DifferentialDriveKinematics(motionConfig.trackWidth * METERS_PER_INCH);
    this.drive_system = new DifferentialDrive(leftMotors, rightMotors);

    // Constrain the voltage that will be sent to the motors based on the numbers found when
    // profiling the drivetrain
    TrajectoryConstraint voltage_constraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(motionConfig.kS, motionConfig.kV, motionConfig.kA), tank_kinematics , 10);
    
    // Create the trajectory configuration based on the motion profile and above constraints.
    // This will be used when generating a path.
    this.traj_config = new TrajectoryConfig(motionConfig.maxVel, motionConfig.maxAccel);
    this.traj_config.setKinematics(tank_kinematics);
    this.traj_config.addConstraint(voltage_constraint);

    // Creates the 'odometry' system: this gives the robot's X/Y position at any given time, based on gyro and encoders
    this.position_system = new DifferentialDriveOdometry(new Rotation2d());

    // Create the 'ramsete controller', which controls calculations on where the robot is vs where it is going next.
    this.auto_drive_system = new RamseteController();

    
  }

  void reset()
  {
    move_init = true;
  }
  
  /**
   * Drives the robot, following the path input.
   * 
   * @param pointMap The "Path" to be followed; a list of waypoints
   * @return Whether or not the robot has finished following the path
   */
  boolean runPath(Path pointMap)
  {
    return false;
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
    List<Pose2d> points;
    
    public Path(Pose2d... points)
    {
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
    public double kP, kI, kD;
    
    // Maximum velocity and acceleration the
    // robot is allowed to achieve
    // in inches / second, inches / second^2
    public double maxVel, maxAccel;
    
    // Width between the left and right wheels, in inches
    public double trackWidth;
  }
  
}