package org.firstinspires.ftc.teamcode.sbfHardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.sbfUtil.PVector;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.hardware.Servo;

@Config
/**
 * Defines the robot. Has hardware objects for the chassis system, all the mechanisms on the robot,
 * and all the functionality methods for the chassis as well as any methods that use multiple mechanisms.
 *
 * @author Andrew, 8668 Should Be Fine!
 */
public class Robot
{

    // Robot - Generic Items
    /** A telemetry object passed down from the opmode */
    private Telemetry telemetry;
    /** A hardware map object passed down from the opmode. */
    private HardwareMap hardwareMap;
    /** The robot heading offset -- makes what angle the robot starts at not zero.*/
    private double offset;
    /** A string that describes which camera should be used by an autonomous opmode -- is overridden
     * in the child autonomous classes. */
    public String whichCamera = "leftCam";
    /** Sets the starting location vector for the robot -- overridden in the child autonomous classes. */
    public PVector location = new PVector(39,9);
    /** The starting robot velocity, */
    public PVector velocity = new PVector(0,0);
    /** The robot's current rate of heading change. */
    public double currentAngularVelocity;

    // Lift Class
    /** The lift object -- imports the all the hardware and methods for the lift mechanism. */
    public Lift lift = new Lift();
    /** The Blinkin object -- imports all the hardware and methods for the Blinkin LED driver. */
    public Blinkin lights = new Blinkin();
    /** The intake object -- imports all the hardware and methods for the intake mechanim. */
    public Intake intake = new Intake();

    // Vision System Items
    /** The camera object -- imports all the hardware and methods for the webcams and image pipeline. */
    private CameraVision eyeOfSauron = new CameraVision();
    /** Marks whether or not to run the camera in the opmode. */
    boolean useCamera;

    /**
     * Robot - REV Hub Items
     */
    /** A bulk data object for the primary rev hub. */
    private RevBulkData bulkData;
    /** A bulk data object for the secondary rev hub. */
    private RevBulkData bulkDataAux;
    /** Declares the primary rev hub as an extended expansion hub. */
    private ExpansionHubEx expansionHub;
    /** Declares the secondary rev hub as an extended expansion hub. */
    private ExpansionHubEx expansionHubAux;

    /**
     * Chassis Items
      */
    /** Declares the right front motor as an expanded rev hub motor.*/
    private ExpansionHubMotor RF = null;
    /** Declares the right rear motor as an expanded rev hub motor.*/
    private ExpansionHubMotor RR = null;
    /** Declares the left front motor as an expanded rev hub motor.*/
    private ExpansionHubMotor LF = null;
    /** Declares the left rear motor as an expanded rev hub motor.*/
    private ExpansionHubMotor LR = null;
    /** Declares the internal rev imu. */
    private BNO055IMU gyro;
    /** The dc motor whose encoder is being used for distance measurements. */
    ExpansionHubMotor encoderMotor;

    /**
     * Foundation Fingers Items
     */
    /** The left fonudation finger servo. */
    private Servo leftFoundation = null;
    /** The right foundation finger servo. */
    private Servo rightFoundation = null;

    /**
     * Robot - Odometry Items
     */
    /** Declares the xEncoder as an expanded rev hub motor. */
    private ExpansionHubMotor xEncoder = null;
    /** Declares the yEncoder as an expanded rev hub motor. */
    private ExpansionHubMotor yEncoder = null;

    /**
     * Robot - Encoder information used in odometry
     */
    /** An x-axis velocity measurement. */
    private double xEncInPerSec;
    /** A y-axis velocity measurement. */
    private double yEncInPerSec;
    /** The radius of the encoder wheels. */
    private final double encWheelRadius = 1.96/2.0; //in inches ... encoder is a 50mm diameter wheel.
//    private final double encTickPerRotation = 2400;
    /** The number of encoder ticks per rotation of the encoder wheel. */
    private final double encTickPerRotation = 3200;
//    public static double encDistanceConstant = 195.5/192; //calibrated over 16' & 12' on foam tiles -- 9/13/19
    /** A constant used in calculating robot position change. */
    public static double encDistanceConstant = 1;
    /** The number of inches moved per rotation of the encoder wheel. */
    private final double encInchesPerRotation = 2.0 * encWheelRadius * Math.PI * encDistanceConstant; // this is the encoder wheel distancd
    /** The gearing on the drive train. */
    private final double gearRatio = 1.733333333;
    /** The number of encoder ticks per inch moved. */
    private final double encTicksPerInch = encTickPerRotation / (encInchesPerRotation);

    /** Used to calculate the change in x position. */
    private int prevXEncoder = 0;
    /** Used to calculate the change in y position. */
    private int prevYEncoder = 0;
    /** Stores the change in x position. */
    private int xEncoderChange = 0;
    /** Stores the change in y position. */
    private int yEncoderChange = 0;

    /** The current raw heading of the robot (no offset). */
    private double currentRawHeading = 0;
    /** Marks whether or not the servos are done. */
    private boolean servoDone = false;

    /** Directional variables used to simulate joystick commands in autonomous.
     * Simulates a forward drive command.*/
    static final double FORWARD = 0.0;
    /** Directional variables used to simulate joystick commands in autonomous.
     * Simulates a backward drive command.*/
    static final double BACKWARD = 180.0;
    /** Directional variables used to simulate joystick commands in autonomous.
     * Simulates a right strafe command.*/
    static final double RIGHT = 270.0;
    /** Directional variables used to simulate joystick commands in autonomous.
     * Simulates a left strafe command.*/
    static final double LEFT = 90.0;
    /** Directional variables used to simulate joystick commands in autonomous.
     * Simulates a forward-right diagonal strafe command.*/
    static final double FORWARD_RIGHT_DIAGONAL = -45.0;
    /** Directional variables used to simulate joystick commands in autonomous.
     * Simulates a forward-left diagonal strafe command.*/
    static final double FORWARD_LEFT_DIAGONAL = 45.0;
    /** Directional variables used to simulate joystick commands in autonomous.
     * Simulates a backward-right diagonal strafe command.*/
    static final double REVERSE_RIGHT_DIAGONAL = -135.0;
    /** Directional variables used to simulate joystick commands in autonomous.
     * Simulates a backward-left diagonal strafe command.*/
    static final double REVERSE_LEFT_DIAGONAL = 135.0;

    /** The counts per motor revolution for a REV HD 40:1 Hex Motor.
     * Used in converting inches to encoder ticks. Allows the programmer to code in inches while
     * the motor measures in encoder ticks.*/
    final double COUNTS_PER_MOTOR_REV = 1120;
    /** The wheel diameter of the mecanum wheel currently on the robot.
     * Used in converting inches to encoder ticks. Allows the programmer to code in inches while
     * the motor measures in encoder ticks.*/
    final double WHEEL_DIAMETER_INCHES = 100.0/25.4;
    /** The inches traveled per wheel rotation for the 4" diameter mecanum wheels currently on the robot.
     * Used in converting inches to encoder ticks. Allows the programmer to code in inches while
     * the motor measures in encoder ticks.*/
    final double INCH_PER_REV = WHEEL_DIAMETER_INCHES * Math.PI;
    /** The encoder ticks per inch ( (ticks per mtr rev*10)/(13*4*3.14159) ).
     * Used in converting inches to encoder ticks. Allows the programmer to code in inches while
     * the motor measures in encoder ticks.*/
    final double CHASSIS_COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV)/(gearRatio*INCH_PER_REV);

    /** An int variable used in drive, tankDrive, and pointTurn to capture the encoder position before each move. */
    double initialPosition;
    /** A double variable used in drive and tankDrive to capture the initial heading before each move. */
    double initialHeading;
    /** A double variable used in pointTurn to either turn the robot left or right. */
    double directionalPower;
    /** A double variable used in pointTurn to represent the value by which the robot needs to correct.*/
    double error;
    /** A double that stores the starting heading of the robot for use in reseting the robot's
     * heading to its start heading. */
    private double resetHeading;
    /** Marks whether or not the robot is moving. */
    private boolean moving;
    /** Sets the start time to zero (in nanoseconds). */
    private long startTime = 0; // in nanoseconds
    /** A double that is the number of nanoseconds per second. */
    double NANOSECONDS_PER_SECOND = TimeUnit.SECONDS.toNanos(1);

    /**
     * Robot Public Interface
     */

    /**
     *  Robot - Triggers the initialization of the selected classes.  Intended to be used  when the INIT
     *  button is pressed in an OpMode.  Loads the hardware items from the HardwareMap and gets
     *  things ready to start.
     *
     * @param telem  An instance of Telemetry which allows the use of Telemtry in this class.
     * @param hwmap  An instance of the FIRST-provided HardwareMap which is passed onto more
     *               specific classes for initializing hardware.
     * @param useVision  A boolean flag which tells the class whether or not the camera should be used.
     * @param theOffset  The heading offset which is set by the child autonomous classes.
     * @param isTeleop  Marks whether or not the opmode is teleop.
     * */
    public void init(Telemetry telem, HardwareMap hwmap, boolean useVision, double theOffset, boolean isTeleop )
    {
        telemetry = telem;
        hardwareMap = hwmap;
        useCamera = useVision;
        startTime = 0;
        offset = theOffset;

        if ( useCamera)
        {
            eyeOfSauron.init(hwmap, whichCamera, telemetry);
        }
        lights.init(telemetry, hardwareMap, isTeleop);
        lift.init(telemetry, hardwareMap);

        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        expansionHubAux = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 3");
        /*
         * Setting ExpansionHub I2C bus speed
         */
        expansionHub.setAllI2cBusSpeeds(ExpansionHubEx.I2cBusSpeed.FAST_400K);
        expansionHubAux.setAllI2cBusSpeeds(ExpansionHubEx.I2cBusSpeed.FAST_400K);
        telemetry.addLine("Setting speed of all I2C buses");

        try
        {
            RF = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, "rf");
            RF.setDirection(DcMotorEx.Direction.FORWARD);
            RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        catch(Exception p_exception)
        {
            telemetry.addData("RF not found in config file", "");
            RF = null;
        }
        try
        {
            RR = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, "rr");
            RR.setDirection(DcMotorEx.Direction.FORWARD);
            RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        catch (Exception p_exception)
        {
            telemetry.addData("RR not found in config file", "");
            RR = null;
        }
        try
        {
            LF = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, "lf");
            LF.setDirection(DcMotorEx.Direction.REVERSE);
            LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        catch (Exception p_exception)
        {
            telemetry.addData("LF not found in config file", "");
            LF = null;
        }
        try
        {
            LR = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, "lr");
            LR.setDirection(DcMotorEx.Direction.REVERSE);
            LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        catch (Exception p_exception)
        {
            telemetry.addData("LR not found in config file","");
            LR = null;
        }

        try
        {
            xEncoder = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, "xEncoder");
            xEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            xEncoder.setDirection((DcMotorEx.Direction.REVERSE));
        }
        catch(Exception p_exception)
        {
            telemetry.addData("x encoder not found in config file", "");
            xEncoder = null;
        }
        try
        {
            yEncoder = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, "yEncoder");
            yEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            yEncoder.setDirection((DcMotorEx.Direction.FORWARD ));
        }
        catch(Exception p_exception)
        {
            telemetry.addData("y encoder not found in config file", "");
            yEncoder = null;
        }

        //MUST INITIALIZE INTAKE AFTER X AND Y ENCODERS
        intake.init(telemetry, hardwareMap);


        try
        {
            leftFoundation = hardwareMap.get(Servo.class, "leftFoundation");
            leftFoundation.setDirection(Servo.Direction.FORWARD);
        }
        catch (Exception p_exeception)
        {
            telemetry.addData("leftFoundation not found in config file", 0);
            leftFoundation = null;
        }
        try
        {
            rightFoundation = hardwareMap.get(Servo.class, "rightFoundation");
            rightFoundation.setDirection(Servo.Direction.REVERSE);
        }
        catch (Exception p_exeception)
        {
            telemetry.addData("rightFoundation not found in config file", 0);
            rightFoundation = null;
        }



        try
        {
            // Set up the parameters with which we will use our IMU. Note that integration
            // algorithm here just reports accelerations to the logcat log; it doesn't actually
            // provide positional information.
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            // Retrieve and initialize the IMU.
            gyro = hardwareMap.get(BNO055IMU.class, "imu");
            gyro.initialize(parameters);
        }
        catch (Exception p_exeception)
        {
            telemetry.addData("imu not found in config file", 0);
            gyro = null;
        }

        if(leftFoundation!=null && rightFoundation!=null)
        {
            releaseFoundation();
        }

        encoderMotor = LF;
        moving = false;


    }

    /**
     * Robot - Method that is intended to be called repeatedly after the INIT button is pressed and handled
     * in an OpMode.  If there is something which should happen repeatedly after INIT but before
     * START, call this method from the init_loop() method in the OpMode.
     */
    public void init_loop()
    {
//        telemetry.addData("gyro status ", gyro.getSystemStatus());
    }

    /**
     * Robot - Method that is intended to be called when the START button is pressed and handled
     * in an OpMode.  If there is something which should happen when start() is called in the
     * OpMode, put it in this method.
     *
     * Currently starts the timers inside of the Blinkin LED Driver class.
     */
    public void start()
    {
//        lights.start();
    }

    /**
     * Robot - Queries the REV Hub using bulk data transfer of the OpenFTC REVExtnesions2 tools.  Updates
     * the heading and position values for the chassis and lift systems.
     */
    public void update()
    {
        /*
         * Update the sensor data using bulk transferes from the Rev Hubs
         */
        currentRawHeading = updateHeadingRaw();
        telemetry.addData("raw heading", currentRawHeading);
        telemetry.addData("pursuitHeading: ", getHeadingPursuit());
        bulkData = expansionHub.getBulkInputData();
        bulkDataAux = expansionHubAux.getBulkInputData();

        /*
         * Update the velocity as read by the encoders
         */
        double xEncTicksPerSecond = bulkDataAux.getMotorVelocity(xEncoder) / gearRatio;
        double yEncTicksPerSecond = bulkDataAux.getMotorVelocity(yEncoder) / gearRatio;

        xEncInPerSec = xEncTicksPerSecond / encTicksPerInch;
        yEncInPerSec = yEncTicksPerSecond / encTicksPerInch;

        /*
         * Update the position change since the last time as read by the encoders
         */
        xEncoderChange = bulkDataAux.getMotorCurrentPosition(xEncoder) - prevXEncoder;
        yEncoderChange = bulkDataAux.getMotorCurrentPosition(yEncoder) - prevYEncoder;

//        telemetry.addData("xEncoder", bulkDataAux.getMotorCurrentPosition(xEncoder));
//        telemetry.addData("yEncoder", bulkDataAux.getMotorCurrentPosition(yEncoder));

//        telemetry.addData("left position: ", bulkDataAux.getMotorCurrentPosition(lift.leftVertical));
//        telemetry.addData("right position: ", bulkDataAux.getMotorCurrentPosition(lift.rightVertical));
        lift.encoder = bulkDataAux.getMotorCurrentPosition(lift.leftVertical);

        /* store the current value to use as the previous value the next time around */
        prevXEncoder = bulkDataAux.getMotorCurrentPosition(xEncoder);
        prevYEncoder = bulkDataAux.getMotorCurrentPosition(yEncoder);

        telemetry.addData("x encoder: ", xEncoder.getCurrentPosition());
        telemetry.addData("y encoder: ", yEncoder.getCurrentPosition()) ;

        updateVelocity(this.getVelocity());
        updatePosition(this.getLocationChange());
       // updateHeading(getHeadingPursuit());
        updateAngularVelocity(this.getAngularVelocity());
    //    updateHeading(currentRawHeading);
    }


    /**
     * Robot - applies an offset that is created in the gyro when initializing the robot in the
     * Pursuit enabled autonomous
     * @return double indicating the current heading as read from the gyro with the offset added.
     */
    public double getHeadingPursuit()
    {
        return (currentRawHeading + offset);
    }

    /**
     * Robot - calculate the amount of distance the robot has travelled since the last time this was called
     * @return PVector indicating the number of inches traveled in x,y
     */
    public PVector getLocationChange()
    {
        return new PVector(getXChange(), getYChange());
    }

    /**
     * Robot - command the Chassis to move according to the desired linear velocity and spin calculated
     * by the pursuit code.
     * @param neededVelocity PVector that indicates the linear velocity needed by the robot to continue pursuit
     * @param spin double indicating the angular velocity needed by the robot to continue pursuit
     */
    public void updateMotors(PVector neededVelocity, double spin)
    {
        neededVelocity.rotate((float)Math.toRadians( getHeadingPursuit() ));
        double x = neededVelocity.x / 40.0; //bot.maxSpeed; //max speed is 31.4 in/sec
        double y = neededVelocity.y / 40.0; // bot.maxSpeed;

        telemetry.addData("x encoder: ", xEncoder.getCurrentPosition());
        telemetry.addData("y encoder: ", yEncoder.getCurrentPosition());
        telemetry.addData("SbfJoystick x, y: ", "%.3f, %.3f", x, y );

        double turn = spin / 343;

        joystickDrive(x, y, turn, 0, 1);
    }

    /** Fetches the telemetry relating to the encdoers. */
    public void getEncoderTelem()
    {
        getXChange();
        getYChange();
        getXLinearVelocity();
        getYLinearVelocity();
    }

    /**
     * Robot - Accesses the current angular velocity of the robot as read from the IMU.
     * @return double that represents the angular velocity in deg/sec
     */
    public double getAngularVelocity()
    {
        AngularVelocity gyroReading;
        gyroReading = gyro.getAngularVelocity();
        telemetry.addData("mp.rot rate: ", -gyroReading.zRotationRate);
        return -gyroReading.zRotationRate;
    }

    /**
     * Robot - Get the robot's current linear velocity
     * @return PVector which captures the current x,y velocity of the robot.
     */
    public PVector getVelocity()
    {
        return new PVector(getXLinearVelocity(), getYLinearVelocity());
    }

    /**
     * Robot - Kills the Chassis, Lift, and Intake systems.
     */
    public void stopEverything()
    {
        if(motorsValid())
        {
            RF.setPower(0.0);
            RR.setPower(0.0);
            LF.setPower(0.0);
            LR.setPower(0.0);
        }
        {
            telemetry.addData("A drive motor is not configured properly", "");
        }
        lift.stopLift();
        intake.intakeStop();
    }

    /** Stops the drive train. */
    public void stopChassis()
    {
        if(motorsValid())
        {
            RF.setPower(0.0);
            RR.setPower(0.0);
            LF.setPower(0.0);
            LR.setPower(0.0);
        }
    }
    /** Stops the lift mechanism. */
    public void stopLift()
    {
        lift.stopLift();

    }
    /** Stops the intake mechanism. */
    public void stopIntake()
    {
        intake.intakeStop();

    }

    /**
     * Drives the lift to a specific position at a specified power.
     * @param power  The power that the lift drives at.
     * @param positionInches  The position the lift needs to move to.
     * @param time  The maximum time the move can take.
     * @return Whether or not the method has finished.
     */
    public boolean liftDrive(double power, double positionInches, double time)
    {
        update();
        return lift.vLiftDrive(power, positionInches, time);
    }

    /**
     * Robot Private Methods
     */

    /**
     * Robot - Used to query the IMU and get the robot's heading.  Internal method only.
     * @return  the robot's heading as an double
     */
    private double updateHeadingRaw()
    {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return -AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
    }

    /**
     * Robot - calculate the amount of X distance the robot has travelled since the last time this was called
     * @return float indicating the number of inches traveled
     */
    private float getXChange()
    {
        double inchesX = (((xEncoderChange) / encTickPerRotation) * encInchesPerRotation) * Math.cos(Math.toRadians(getHeadingPursuit())) +
                         (((yEncoderChange) / encTickPerRotation) * encInchesPerRotation) * Math.sin(Math.toRadians(getHeadingPursuit()));
        return (float)inchesX;
    }

    /**
     * Robot - calculate the amount of Y distance the robot has travelled since the last time this was called
     * @return float indicating the number of inches traveled
     */
    private float getYChange()
    {
        double inchesY = ((-(xEncoderChange) / encTickPerRotation) * encInchesPerRotation) * Math.sin(Math.toRadians( getHeadingPursuit() )) +
                         (((yEncoderChange) / encTickPerRotation) * encInchesPerRotation) * Math.cos(Math.toRadians( getHeadingPursuit() ));
        return (float)inchesY;
    }

    /**
     * Robot - Access the current linear velocity in the X direction.
     * @return float representing the X linear velocity in in/sec
     */
    private float getXLinearVelocity()
    {
        double linearX = xEncInPerSec * Math.cos(Math.toRadians( getHeadingPursuit() )) +
                yEncInPerSec * Math.sin(Math.toRadians( getHeadingPursuit() ));
        return (float)linearX;
    }

    /**
     * Robot - Access the current linear velocity in the Y direction.
     * @return float representing the Y linear velocity in in/sec
     */
    private float getYLinearVelocity()
    {
        double linearY = -xEncInPerSec * Math.sin(Math.toRadians( getHeadingPursuit() )) +
                (yEncInPerSec) * Math.cos(Math.toRadians( getHeadingPursuit() ));
        return (float)linearY;
    }

    /**
     * Robot - Get the number of seconds this op mode has been running
     * <p>
     * This method has sub millisecond accuracy.
     * @return number of seconds this op mode has been running
     */
    private double getRuntime()
    {
        return (System.nanoTime() - startTime) / NANOSECONDS_PER_SECOND;
    }

    /**
     * Robot - Reset the internal timer to zero.
     */
    private void resetStartTime()
    {
        startTime = System.nanoTime();
    }

    /**
     * Chassis Public Interface
     */

    /**
     * Chassis - Uses joystick-type inputs to drive the robot. Allows for omnidirectional movement and has a
     * selectable max power.
     *
     * @param leftStickX  The x-axis of the left joystick on the primary gamepad. Controls side to
     *                    side movement.
     * @param leftStickY  The y-axis of the left joystick on the primary gamepad. Controls forward
     *                    and backward movement.
     * @param rightStickX  The x-axis of the right joystick on the primary gamepad. Controls rotation.
     * @param rightStickY  The y-axis of the right joystick on the primary gamepad. N/A
     * @param powerLimit  The maximum power value.
     * */
    public void joystickDrive(double leftStickX, double leftStickY, double rightStickX, double rightStickY, double powerLimit)
    {
        /*
            These are the calculations needed to make a simple mecaccnum drive.
              - The left joystick controls moving straight forward/backward and straight sideways.
              - The right joystick control turning.
        */

        telemetry.addData("x encoder: ", xEncoder.getCurrentPosition());
        telemetry.addData("y encoder: ", yEncoder.getCurrentPosition());
        double forward = leftStickY;
        double right = -leftStickX;
        double clockwise = rightStickX;

        double leftFront = (forward + clockwise + right);
        double rightFront = (forward - clockwise - right);
        double leftRear = (forward + clockwise - right);
        double rightRear = (forward - clockwise + right);


        //Find the largest command value given and assign it to max.
        double max = 0.0;
        if (Math.abs(leftFront) > max)  { max = Math.abs(leftFront); }
        if (Math.abs(rightFront) > max) { max = Math.abs(rightFront); }
        if (Math.abs(leftRear) > max)   { max = Math.abs(leftRear); }
        if (Math.abs(rightRear) > max)  { max = Math.abs(rightRear); }

        //Set the minimum and maximum power allowed for drive moves and compare it to the parameter powerLimit.
        powerLimit = Range.clip(powerLimit, .05, 1);

        //If max still equals zero after checking all four motors, then set the max to 1
        if (max == 0.0)
        {
            max = 1;
        }

        // If max is greater than the power limit, divide all command values by max to ensure that all command
        // values stay below the magnitude of the power limit.
        if (max > powerLimit)
        {
            leftFront  = leftFront  / max * powerLimit;
            rightFront = rightFront / max * powerLimit;
            leftRear   = leftRear   / max * powerLimit;
            rightRear  = rightRear  / max * powerLimit;
        }

//        RF.setVelocity(rightFront * 15.7, AngleUnit.RADIANS);
//        RR.setVelocity(rightRear * 15.7, AngleUnit.RADIANS);
//        LF.setVelocity(leftFront * 15.7, AngleUnit.RADIANS);
//        LR.setVelocity(leftRear * 15.7, AngleUnit.RADIANS);

        if(motorsValid())
        {
            RF.setPower(rightFront);
            RR.setPower(rightRear);
            LF.setPower(leftFront);
            LR.setPower(leftRear);
        }
        else
        {
                telemetry.addData("A drive motor is not configured properly", "");
        }


    }

    /**
     * Chassis - Drives the four drive motors on the robot and will move the robot forward,
     * backward, left, right, or at a 45 degree angle in any direction.
     *
     * @param power  How fast the robot will drive.
     * @param gain  The rate at which the robot will correct an error
     * @param direction  In which direction the robot will drive (forward, backward, left, right,
     *                   or 45 degrees in any direction).
     * @param distance  How far the robot will drive.
     * @param time  The max time this move can take. A time-out feature: if the move stalls for some
     *              reason, the timer will catch it.
     * @return  A boolean that tells us whether or not the robot is moving.
     */
    public boolean drive(double power, double direction, double gain, double distance, double time, double intakePower)
    {
        double driveDistance = CHASSIS_COUNTS_PER_INCH * distance;
        double correction;

        update();

        double actual = currentRawHeading;

        intake.intakeDrive(intakePower, true, true);
//        telemetry.addData( "Is RR-Diagonal?: ", direction ==  REVERSE_RIGHT_DIAGONAL);
//        telemetry.addData("Direction: ", direction);
//        telemetry.addData("RR-Diag: ", REVERSE_RIGHT_DIAGONAL);

        if (!moving)
        {
            setZeroBehavior("BRAKE");

            initialHeading = currentRawHeading;
            if (Math.abs(initialHeading) > 130  &&  initialHeading < 0.0)
            {
                initialHeading += 360.0;
            }
            if( (direction == FORWARD_LEFT_DIAGONAL) || (direction  == REVERSE_RIGHT_DIAGONAL) )
            {
                initialPosition = bulkData.getMotorCurrentPosition(RF); //.getCurrentPosition();
                encoderMotor = RF;
            }
            else
            {
                initialPosition = bulkData.getMotorCurrentPosition(LF); //.getCurrentPosition();
                encoderMotor = LF;
            }
            resetStartTime();
            moving = true;
        }
        telemetry.addData("initial position: ", initialPosition);

        if (Math.abs(initialHeading) > 130  &&  actual < 0.0)
        {
            actual += 360;
        }

        correction = ((initialHeading - actual) * gain);

        double lStickX = power * Math.sin(Math.toRadians(direction));
        double lStickY = -(power * Math.cos(Math.toRadians(direction)));

//        telemetry.addData("Right Motor DD: ", Math.abs(rFrontMotor.getCurrentPosition() - initialPosition));
//        telemetry.addData("Left Motor DD:", Math.abs(lFrontMotor.getCurrentPosition() - initialPosition));

        telemetry.addData("Drive Distance: ", driveDistance);
        double tmpDistance = Math.abs(encoderMotor.getCurrentPosition() - initialPosition);
        telemetry.addData("Distance Driven:", tmpDistance);
        telemetry.addData("getRuntime() = ", getRuntime());
//        telemetry.addData("time = ", time);

        joystickDrive(lStickX, -lStickY, correction, 0.0, power);

//        if (((Math.abs(bulkData.getMotorCurrentPosition(encoderMotor)- initialPosition)) >= driveDistance) || (getRuntime() > time))
        if (((Math.abs(encoderMotor.getCurrentPosition() - initialPosition)) >= driveDistance) || (getRuntime() > time))
        {
            stopChassis();
            intake.intakeStop();

            setZeroBehavior("FLOAT");

            encoderMotor = LF;
            moving = false;
        }

        return !moving;
    }

    /**
     * Chassis - The pointTurn method turns the robot to a target heading, automatically picking the turn
     * direction that is the shortest distance to turn to arrive at the target.
     *
     * @param targetHeading  The direction in which the robot will move.
     * @param time  The maximum time the move can take before the code moves on.
     * @param power  The power at which the robot will move.
     * @return A boolean that tells use whether or not the robot is moving.
     */
    public boolean pointTurn(double power, double targetHeading, double time)
    {
        power = Math.abs(power);

        update();
//        telemetry.addData("currentRawHeading: ", currentRawHeading);
//        double currentRawHeading = getRawHeading();

        if (Math.abs(targetHeading) > 170 && currentRawHeading < 0.0)
        {
            currentRawHeading += 360;
        }

        if (!moving)
        {
            setZeroBehavior("BRAKE");

            initialHeading = currentRawHeading;
            error = targetHeading - initialHeading;

            if (error > 180)
            {
                error -= 360;
            }
            else if (error < -180)
            {
                error += 360;
            }

            if (error < 0)
            {
                directionalPower = -power;
            }
            else
            {
                directionalPower = power;
            }

            if ( Math.abs(error) < 60 )
            {
                directionalPower = error * 0.03;
                if (directionalPower > 0 )
                {
                    directionalPower = Range.clip( directionalPower, 0.25, power);
                }
                else
                {
                    directionalPower = Range.clip(directionalPower, -power, -0.25);
                }
            }

            setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);
            resetStartTime();
            moving = true;
        }
        telemetry.addData("error: ", error);
        telemetry.addData("directionalPower: ", directionalPower);

        joystickDrive(0.0, 0.0, directionalPower, 0.0, power);

        if(Math.abs(targetHeading - currentRawHeading) < 3.0 || getRuntime() > time)
        {
            stopChassis();

            setZeroBehavior("FLOAT");

            moving = false;
        }

        return !moving;
    }


    /**
     * Chassis Private Methods
     */

    /**
     * Chassis - setMode sets all four drive motors to a specified mode. There are three mode choices:
     * 1) RUN_USING_ENCODERS,
     * 2) RUN_WITHOUT_ENCODERS, and
     * 3) RUN_TO_POSITION.
     *
     * @param mode The type of mode the motors will will run with.
     */
    private void setMode(ExpansionHubMotor.RunMode mode)
    {
        if (RF != null) { RF.setMode(mode); }
        if (LF != null) { LF.setMode(mode); }
        if (RR != null)  { RR.setMode(mode); }
        if (LR != null)  { LR.setMode(mode); }
    }

    /**
     * Chassis - Tell the chassis motors what to do when power is set to 0
     * @param mode - which indicates the desired behavior
     */
    private void setZeroBehavior(String mode)
    {
        if(mode.equalsIgnoreCase("FLOAT"))
        {
            LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        else if(mode.equalsIgnoreCase("BRAKE"))
        {
            LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    /**
     * Chassis - check to ensure that all of the chassis motors are initialized properly.
     * @return boolean indicating good init (true) or not (false)
     */
    private boolean motorsValid()
    {
        boolean done = false;
        if(RF != null)
        {
            if(RR != null)
            {
                if(LF != null)
                {
                    if (LR != null)
                    {
                        done = true;
                    }
                }
            }
        }
        return done;
    }

    /**
     * Vision System Public Interface
     */

    /**
     * Vision System - get the current position of the SkyStone
     * @return String that describes the position of the 3 stones in view of the camera.
     */
    public String getSkyStonePosition()
    {
        if (useCamera)
        {
            return eyeOfSauron.getSkyStonePosition();
        }
        else
        {
            return "LEFT";
        }
    }

    /**
     * Vision System - Turn off the camera to save power.
     */
    public void stopCamera()
    {
        eyeOfSauron.stopCamera();
    }

    /**
     * Sets the camera to the imputed name.
     * @param device  The name the camera is set to.
     */
    public void setCameraDeviceName( String device )
    {
        eyeOfSauron.setCamDeviceName( device );
    }

    //
    // Foundation Fingers Public Interface
    //

    /**
     * Foundation Fingers - Set the fingers to a particuar position
     * @param position - the servo position to set the fingers
     * @return boolean to indicate that the fingers are at the commanded position
     */
    public boolean foundationDrive(double position)
    {

        if(rightFoundation != null)
        {
            rightFoundation.setPosition(position);
        }
        else
        {
            telemetry.addData("right foundation is null", "cannot use");
        }

        if(leftFoundation != null)
        {
            leftFoundation.setPosition(position);
        }
        else
        {
            telemetry.addData("right foundation is null", "cannot use");
        }

        if(rightFoundation.getPosition()==position && leftFoundation.getPosition()==position)
        {
            servoDone = true;
        }

        return true;
    }

    /**
     * Foundation Fingers - Pull back the fingers to release the Foundation
     */
    public void releaseFoundation()
    {
        if(rightFoundation != null)
        {
            rightFoundation.setPosition(.9);
        }
        else
        {
            telemetry.addData("right foundation is null", "cannot use");
        }

        if(leftFoundation != null)
        {
            leftFoundation.setPosition(.9);
        }
        else
        {
            telemetry.addData("right foundation is null", "cannot use");
        }
    }

    /**
     * Foundation Fingers - Extend the fingers to the position where the Foundation is engaged
     */
    public void grabFoundation()
    {
        if(rightFoundation != null)
        {
            rightFoundation.setPosition(.2);
        }
        else
        {
            telemetry.addData("right foundation is null", "cannot use");
        }

        if(leftFoundation != null)
        {
            leftFoundation.setPosition(.2);
        }
        else
        {
            telemetry.addData("right foundation is null", "cannot use");
        }
    }


    /** Update the current location of the robot.
     * @param currentPosition The position being added to location.
     * */
    public void updatePosition(PVector currentPosition)
    {
        location = PVector.add(location, currentPosition);
    }

    /**
     * Update the current velocity.
     * @param currentVelocity  The velocity being set.
     */
    public void updateVelocity(PVector currentVelocity)
    {
        velocity.set(currentVelocity.x, currentVelocity.y);
    }

    /**
     * Update the current angular velocity.
     * @param angularVelocity  The angular velocity being set.
     */
    public void updateAngularVelocity( double angularVelocity )
    {
        currentAngularVelocity = angularVelocity;
    }

    /**
     * Update the current heading.
     * @param heading  The heading being set.
     */
    public void updateHeading( double heading )
    {
        currentRawHeading = heading;
    }

    /**
     * Drives the intake motors to suck in -- combined with lights and distance sensors to show
     * where the stone is inside the robot.
     *
     * @param thePower  The power at which the intake wheels will spin (sign determines direction).
     * */
    public void intakeInWithLights(double thePower)
    {
        if(intake.rampSignal())
        {
//            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIME);
            intake.intakeIn(thePower, true, true);
        }
        else if(intake.backSignal() && intake.rampSignal())
        {
//            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            intake.intakeIn(thePower, true, false);
        }
        else
        {
//            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
            intake.intakeIn(thePower, true, true);

        }
    }
}
