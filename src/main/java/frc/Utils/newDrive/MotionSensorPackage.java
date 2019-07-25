package frc.Utils.newDrive;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.AnalogAccelerometer;

import java.util.ArrayList;

/**
 * Class which conatins several motion sensors (Gyros, Encoders, and Accelerometers) in order to give general motion values
 * encompassing all of their outputs.
 */
class MotionSensorPackage
{
    
    /**
     * The part of the robot a sensor is in.
     * Useful for encoders and acclererometers to figure out what their outputs mean.
     */
    public enum RobotLocation
    {
        FRONT_STARBOARD,
        MID_STARBOARD,
        BACK_STARBOARD,
        FRONT_PORT,
        MID_PORT,
        BACK_PORT,
        MIDDLE;
    }

    /**
     * Class which encompasses and Encoder, its location, and the radius of the wheel it is attached to.
     * This class is immutable, as it fills the role of a Value Object
     */
    private class EncoderInfo
    {
        private Encoder e;
        private RobotLocation l;
        private double wheelRadius;
        /**
         * Create a new EncoderInfo object
         * @param e
         *  The encoder object to store
         * @param l
         *  The location on the robot the wheel e is connected.
         * @param wheelRadius_mm
         *  The radius of the connected wheel in millimeters
         */
        EncoderInfo(Encoder e, RobotLocation l, double wheelRadius_mm)
        {
            this.e = e;
            this.l = l;
            this.wheelRadius = wheelRadius_mm;
        }
        /**
         * @return
         *  The encoder passed to the constructor
         */
        public Encoder getEncoder()
        {
            return this.e;
        }
        /**
         * @return
         *  The RobotLocation passed to the constructor
         */
        public RobotLocation getLocation()
        {
            return this.l;
        }
        /**
         * @return
         *  The radius of the wheel as passed to the constructor
         */
        public double getWheelRadius()
        {
            return this.wheelRadius;
        }
    }

    private ArrayList<Gyro> gyros = null;
    private ArrayList<EncoderInfo> encoders = null;
    private ArrayList<AnalogAccelerometer> accelerometers = null;
    private final double DEFAULT_WHEEL_RADIUS;
    private final double ROBOT_WIDTH;
    private final double ROBOT_LENGTH;

    /**
     * Create a new motion sensor package, assuming the radius of our wheels is 0mm, and that the robot is a point mass.
     */
    public MotionSensorPackage()
    {
        this(0.0, 0.0);
    }

    /**
     * Create a new motion sensor package, with an assumed wheel radius for all of the added encoders
     * @param wheelRadius_mm
     *  The radius of the wheels on the robot, in mm.
     * @param robotRadius_mm
     *  The average distance from the center of the robot to each of the wheels
     */
    public MotionSensorPackage(double wheelRadius_mm, double robotRadius_mm)
    {
        this.DEFAULT_WHEEL_RADIUS = wheelRadius_mm;
        this.ROBOT_WIDTH = robotRadius_mm;
    }

    /**
     * Register a gyroscope sensor with this MotionSensorPackage
     * @param toAdd
     *  The gyroscope object to register
     */
    public void addGyro(Gyro toAdd)
    {
        if(this.gyros == null)
        {
            this.gyros = new ArrayList<>(1);
        }
        this.gyros.add(toAdd);
    }

    /**
     * Register a accelerometer sensor with this MotionSensorPackage
     * @param toAdd
     *  The accelerometer object to register
     */
    public void addAccelerometer(AnalogAccelerometer toAdd)
    {
        if(this.accelerometers == null)
        {
            this.accelerometers = new ArrayList<>(1);
        }   
        this.accelerometers.add(toAdd);
    }

    /**
     * Add an Encoder to sensor tracking, with all the information.
     * @param enc The encoder object
     * @param loc The location of the encoder on the robot
     * @param wheelRadius_mm
     *  The radius of the wheel for this encoder
     */
    public void addEncoder(Encoder enc, RobotLocation loc, double wheelRadius_mm)
    {
        if(this.encoders == null)
        {
            this.encoders = new ArrayList<>(1);
        }
        this.encoders.add(new EncoderInfo(enc, loc, wheelRadius_mm));
    }

    /**
     * Add an Encoder to sensor tracking, and assume the radius of the wheel was passed in the constructor
     * @param enc
     *  The encoder object
     * @param loc
     *  The location on the robot of the wheel
     */
    public void addEncoder(Encoder enc, RobotLocation loc)
    {
        addEncoder(enc, loc, this.DEFAULT_WHEEL_RADIUS);
    }

    public void zeroSensors()
    {
        zeroSensorClass(SensorClass.ENCODERS);
        zeroSensorClass(SensorClass.GYROS);
        zeroSensorClass(SensorClass.ACCELEROMETERS);
    }

    public enum SensorClass
    {
        ENCODERS,
        GYROS,
        ACCELEROMETERS;
    }

    public void zeroSensorClass(SensorClass toZero)
    {
        switch(toZero)
        {
            default:
            case ENCODERS:
                for(EncoderInfo e : this.encoders)
                {
                    e.getEncoder().reset();
                }
            break;
            case GYROS:
                for(Gyro g: this.gyros)
                {
                    g.reset();
                }
            break;
            case ACCELEROMETERS:
                //Do nothing
            break;
        }
    }


    public double getLinearDistance()
    {
        double avg = 0;
        for(EncoderInfo e: this.encoders)
        {
            avg += e.getEncoder().getDistance();
        }
        avg /= this.encoders.size();
        return avg;
    }

    public double getRotationGyro()
    {
        double avg = 0;
        for(Gyro g: this.gyros)
        {
            avg += g.getAngle();
        }
        avg /= g.size();
        return avg;
    }

    private double getPortEncoderAverage()
    {
        double avg = 0;
        int count = 0;
        for(EncoderInfo e: this.encoders)
        {
            if(e.getLocation() == RobotLocation.BACK_PORT || 
                e.getLocation() == RobotLocation.MID_PORT || 
                e.getLocation() == RobotLocation.FRONT_PORT)
            {
                avg += e.getEncoder().getDistance();
                count++;
            }
        }
        return avg/count;
    }

    private double getStarboardEncoderAverage()
    {
        double avg = 0;
        int count = 0;
        for(EncoderInfo e: this.encoders)
        {
            if(e.getLocation() == RobotLocation.BACK_STARBOARD || 
                e.getLocation() == RobotLocation.MID_STARBOARD || 
                e.getLocation() == RobotLocation.FRONT_STARBOARD)
            {
                avg += e.getEncoder().getDistance();
                count++;
            }
        }
        return avg/count;
    }

    public double getRotationEncoders()
    {
        double leftDistance = getPortEncoderAverage();
        double rightDistance = getStarboardEncoderAverage();
        double thetaRadians = (-leftDistance - rightDistance)/(-this.ROBOT_WIDTH);
        return Math.toDegrees(thetaRadians);
    }
}
