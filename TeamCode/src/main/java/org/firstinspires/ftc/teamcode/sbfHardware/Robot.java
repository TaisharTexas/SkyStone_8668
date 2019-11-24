package org.firstinspires.ftc.teamcode.sbfHardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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



public class Robot
{

    // TODO: look at drive method imported; check reference to encoder ticks and other things.  Update
    // for the new bulk data transfer.
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    CameraVision eyeOfSauron = new CameraVision();
    boolean useCamera;

    RevBulkData bulkData;
    RevBulkData bulkDataAux;
    ExpansionHubEx expansionHub;
    ExpansionHubEx expansionHubAux;

    private ExpansionHubMotor RF = null;
    private ExpansionHubMotor RR = null;
    private ExpansionHubMotor LF = null;
    private ExpansionHubMotor LR = null;

    private DcMotor leftIntake = null;
    private DcMotor rightIntake = null;

    private Servo leftFoundation = null;
    private Servo rightFoundation = null;
    private boolean servoDone = false;



    /** The dc motor whose encoder is being used for distance measurements. */
    ExpansionHubMotor encoderMotor;

    private ExpansionHubMotor xEncoder = null;
    private ExpansionHubMotor yEncoder = null;
    private BNO055IMU gyro;

    private double xTicksPerRad;
    private double yTicksPerRad;
    private double xTicksPerSecond;
    private double yTicksPerSecond;
    private double xInPerSec;
    private double yInPerSec;
    private final double encoderWheelRadius = 1.0; //in inches
    private final double tickPerRotation = 2400;
    private final double distanceConstant = 195.5/192; //calibrated over 16' & 12' on foam tiles -- 9/13/19
    private final double inchesPerRotation = 2.0 * encoderWheelRadius * Math.PI * distanceConstant; // this is the encoder wheel distancd
    private final double gearRatio = 1.733333333;
    private final double ticksPerInch = tickPerRotation / ( inchesPerRotation );
    private double xPrev = 0;
    private double yPrev = 0;

    public int prevXEncoder = 0;
    public int prevYEncoder = 0;
    public int xEncoderChange = 0;
    public int yEncoderChange = 0;
    public double currentHeading = 0;


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
    /** The drive gear reduction currently being used on the robot drive modules.
     * Used in converting inches to encoder ticks. Allows the programmer to code in inches while
     * the motor measures in encoder ticks.*/
    final double DRIVE_GEAR_REDUCTION = 1.3;
    /** The wheel diameter of the mecanum wheel currently on the robot.
     * Used in converting inches to encoder ticks. Allows the programmer to code in inches while
     * the motor measures in encoder ticks.*/
    final double WHEEL_DIAMETER_INCHES = 4.0;
    /** The inches traveled per wheel rotation for the 4" diameter mecanum wheels currently on the robot.
     * Used in converting inches to encoder ticks. Allows the programmer to code in inches while
     * the motor measures in encoder ticks.*/
    final double INCH_PER_REV = WHEEL_DIAMETER_INCHES * 3.14159;
    /** The encoder ticks per inch ( (ticks per mtr rev*10)/(13*4*3.14159) ).
     * Used in converting inches to encoder ticks. Allows the programmer to code in inches while
     * the motor measures in encoder ticks.*/
    final double COUNTS_PER_INCH = (1120*10)/(17.333*4*Math.PI);

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
    double resetHeading;
    boolean moving;
    private long startTime = 0; // in nanoseconds
    /** A double that is the number of nanoseconds per second. */
    double NANOSECONDS_PER_SECOND = TimeUnit.SECONDS.toNanos(1);



    public void init(Telemetry telem, HardwareMap hwmap, boolean useVision )
    {
        telemetry = telem;
        hardwareMap = hwmap;
        useCamera = useVision;
        startTime = 0;

        if ( useCamera)
        {
            eyeOfSauron.init(hwmap);
        }

        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        expansionHubAux = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 3");
        /*
         * Setting ExpansionHub I2C bus speed
         */
//        expansionHub.setAllI2cBusSpeeds(ExpansionHubEx.I2cBusSpeed.FAST_400K);
//        telemetry.addLine("Setting speed of all I2C buses");

        /* TODO: add try/catch for the items in the hardware map */
        RF = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, "rf");
        RF.setDirection(DcMotorEx.Direction.FORWARD);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        RR = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, "rr");
        RR.setDirection(DcMotorEx.Direction.FORWARD);
        RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        LF = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, "lf");
//        LF.setDirection(DcMotorSimple.Direction.FORWARD);
        LF.setDirection(DcMotorEx.Direction.REVERSE);
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        LR = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, "lr");
//        LR.setDirection(DcMotorSimple.Direction.FORWARD);
        LR.setDirection(DcMotorEx.Direction.REVERSE);
        LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);




        xEncoder = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, "xEncoder");
       // xEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        xEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        xEncoder.setDirection((DcMotorEx.Direction.FORWARD));

        yEncoder = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, "yEncoder");
        //yEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        yEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        yEncoder.setDirection((DcMotorEx.Direction.FORWARD ));

        xTicksPerRad = xEncoder.getMotorType().getTicksPerRev() / 2.0 / Math.PI;
        yTicksPerRad = yEncoder.getMotorType().getTicksPerRev() / 2.0 / Math.PI;

        leftIntake = hardwareMap.get(DcMotor.class, "yEncoder");
        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntake = hardwareMap.get(DcMotor.class, "xEncoder");
        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        try
        {
            leftFoundation = hardwareMap.get(Servo.class, "leftFoundation");
            leftFoundation.setDirection(Servo.Direction.FORWARD);
//            leftFoundation.scaleRange();
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
//            rightFoundation.scaleRange();
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

    public void init_loop()
    {
//        telemetry.addData("gyro status ", gyro.getSystemStatus());
    }


    public void start()
    {

    }

    public void update()
    {
        /*
         * Update the sensor data using bulk transferes from the Rev Hubs
         */
        currentHeading = updateHeading();
        bulkData = expansionHub.getBulkInputData();
        bulkDataAux = expansionHubAux.getBulkInputData();

        /*
         * Update the velocity as read by the encoders
         */
        xTicksPerSecond = bulkDataAux.getMotorVelocity(xEncoder) / gearRatio;
        yTicksPerSecond = bulkDataAux.getMotorVelocity(yEncoder) / gearRatio;

        xInPerSec = xTicksPerSecond / ticksPerInch;
        yInPerSec = yTicksPerSecond / ticksPerInch;

        /*
         * Update the position change since the last time as read by the encoders
         */
        xEncoderChange = bulkDataAux.getMotorCurrentPosition(xEncoder) - prevXEncoder;
        yEncoderChange = bulkDataAux.getMotorCurrentPosition(yEncoder) - prevYEncoder;

        telemetry.addData("xEncoder", bulkDataAux.getMotorCurrentPosition(xEncoder));
        telemetry.addData("yEncoder", bulkDataAux.getMotorCurrentPosition(yEncoder));


        /* store the current value to use as the previous value the next time around */
        prevXEncoder = bulkDataAux.getMotorCurrentPosition(xEncoder);
        prevYEncoder = bulkDataAux.getMotorCurrentPosition(yEncoder);
    }

    /**
     * Used to get the robot's heading.
     *
     * @return  the robot's heading as an double
     */
    public double updateHeading()
    {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return -AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
    }


    public double getHeading()
    {
        return currentHeading;
    }

    public void getEncoderTelem()
    {
        getXChange();
        getYChange();
        getXLinearVelocity();
        getYLinearVelocity();
    }


    public float getXChange()
    {
        double inchesX = (((xEncoderChange) / tickPerRotation) * inchesPerRotation) * Math.cos(Math.toRadians(currentHeading)) +
                         (((yEncoderChange) / tickPerRotation) * inchesPerRotation) * Math.sin(Math.toRadians(currentHeading));
        return (float)inchesX;
    }

    public float getYChange()
    {
        double inchesY = ((-(xEncoderChange) / tickPerRotation) * inchesPerRotation) * Math.sin(Math.toRadians(currentHeading)) +
                         (((yEncoderChange) / tickPerRotation) * inchesPerRotation) * Math.cos(Math.toRadians(currentHeading));
        return (float)inchesY;
    }

    public PVector getLocationChange()
    {
        PVector location = new PVector(getXChange(), getYChange());
        return location;
    }

    public void updateMotors(PVector neededVelocity, double spin)
    {
        float rotation = (float)(90-currentHeading) - (float)Math.toDegrees(neededVelocity.heading());
        telemetry.addData("currHdg, needHdg, rotation ", "%.2f, %.2f, %.2f", 90-currentHeading, Math.toDegrees(neededVelocity.heading()), rotation);

        double x = neededVelocity.mag()*Math.sin(Math.toRadians(rotation)) / 40.0; //bot.maxSpeed; //max speed is 31.4 in/sec
        double y = neededVelocity.mag()*Math.cos(Math.toRadians(rotation)) / 40.0; // bot.maxSpeed;

        telemetry.addData("SbfJoystick x, y: ", "%.3f, %.3f", x, y );

        double turn = -spin / 343;

        joystickDrive(x, y, turn, 0, 1);
    }

    public void joystickDrive(double leftStickX, double leftStickY, double rightStickX, double rightStickY, double powerLimit)
    {
        /*
            These are the calculations needed to make a simple mecaccnum drive.
              - The left joystick controls moving straight forward/backward and straight sideways.
              - The right joystick control turning.
        */
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


        RF.setPower(rightFront);
        RR.setPower(rightRear);
        LF.setPower(leftFront);
        LR.setPower(leftRear);

    }


    public double getAngularVelocity()
    {
        AngularVelocity gyroReading;
        gyroReading = gyro.getAngularVelocity();
//        telemetry.addData("mp.rot rate: ", -gyroReading.xRotationRate);
        return -gyroReading.xRotationRate;
    }

    public float getXLinearVelocity()
    {
        double linearX = xInPerSec * Math.cos(Math.toRadians(currentHeading)) +
                yInPerSec * Math.sin(Math.toRadians(currentHeading));
        return (float)linearX;
    }

    public float getYLinearVelocity()
    {
        double linearY = -xInPerSec * Math.sin(Math.toRadians(currentHeading)) +
                ( yInPerSec ) * Math.cos(Math.toRadians(currentHeading));
        return (float)linearY;
    }

    public PVector getVelocity()
    {
        PVector velocity = new PVector(getXLinearVelocity(), getYLinearVelocity());
        return velocity;
    }

    public void stop()
    {
        RF.setPower(0.0);
        RR.setPower(0.0);
        LF.setPower(0.0);
        LR.setPower(0.0);

        if (useCamera)
        {
            eyeOfSauron.stopCamera();
        }

    }



    /**
     * The mecanumDrive method moves the four drive motors on the robot and will move the robot forward,
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
    public boolean drive(double power, double direction, double gain, double distance, double time, boolean intake)
    {
        double driveDistance = COUNTS_PER_INCH * distance;
        double correction;

        update();

        double actual = currentHeading;

        if(intake)
        {
            leftIntake.setPower(.5);
            rightIntake.setPower(.5);
        }

//        telemetry.addData( "Is RR-Diagonal?: ", direction ==  REVERSE_RIGHT_DIAGONAL);
//        telemetry.addData("Direction: ", direction);
//        telemetry.addData("RR-Diag: ", REVERSE_RIGHT_DIAGONAL);

        if (!moving)
        {
            setZeroBehavior("BRAKE");

            initialHeading = currentHeading;
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

        joystickDrive(lStickX, lStickY, -correction, 0.0, power);

//        if (((Math.abs(bulkData.getMotorCurrentPosition(encoderMotor)- initialPosition)) >= driveDistance) || (getRuntime() > time))
        if (((Math.abs(encoderMotor.getCurrentPosition() - initialPosition)) >= driveDistance) || (getRuntime() > time))
        {
            stop();
            intakeStop();

            setZeroBehavior("FLOAT");

            encoderMotor = LF;
            moving = false;
        }

        return !moving;
    }

    /**
     * The pointTurn method turns the robot to a target heading, automatically picking the turn
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
        telemetry.addData("currentHeading: ", currentHeading);
//        double currentHeading = getHeading();

        if (Math.abs(targetHeading) > 170  &&  currentHeading < 0.0)
        {
            currentHeading += 360;
        }

        if (!moving)
        {
            setZeroBehavior("BRAKE");

            initialHeading = currentHeading;
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
                directionalPower = error * 0.01;
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

        if(Math.abs(targetHeading - currentHeading) < 4.0 || getRuntime() > time)
        {
            stop();

            setZeroBehavior("FLOAT");

            moving = false;
        }

        return !moving;
    }



    /**
     * Get the number of seconds this op mode has been running
     * <p>
     * This method has sub millisecond accuracy.
     * @return number of seconds this op mode has been running
     */
    public double getRuntime()
    {
        return (System.nanoTime() - startTime) / NANOSECONDS_PER_SECOND;
    }

    /**
     * Reset the internal timer to zero.
     */
    public void resetStartTime()
    {
        startTime = System.nanoTime();
    }


    /**
     * setMode sets all four drive motors to a specified mode. There are three mode choices:
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
    public void stopCamera()
    {
        eyeOfSauron.stopCamera();
    }



    public void intakeOut(double power)
    {
//        xEncoder.setPower(-1.0);
        leftIntake.setPower(-power * .45);
//        yEncoder.setPower(1.0);
        rightIntake.setPower(power * .45);

    }

    public void intakeIn(double power)
    {
//        xEncoder.setPower(1.0);
        leftIntake.setPower(power * .85);
//        yEncoder.setPower(-1.0);
        rightIntake.setPower(-power * 1);

    }

    public void intakeStop()
    {
//        xEncoder.setPower(0.0);
        leftIntake.setPower(0.0);
//        yEncoder.setPower(0.0);
        rightIntake.setPower(0.0);

    }

    public boolean foundationDrive(double position)
    {

        rightFoundation.setPosition(position);
        leftFoundation.setPosition(position);

        if(rightFoundation.getPosition()==position && leftFoundation.getPosition()==position)
        {
            servoDone = true;
        }

        return true;
    }

    public void releaseFoundation()
    {
        rightFoundation.setPosition(.9);
        leftFoundation.setPosition(.9);
    }


    public void pointFive()
    {
        rightFoundation.setPosition(.5);
        leftFoundation.setPosition(.5);
    }

    public void grabFoundation()
    {
        rightFoundation.setPosition(.15);
        leftFoundation.setPosition(.15);
    }


    public void setZeroBehavior(String mode)
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




}
