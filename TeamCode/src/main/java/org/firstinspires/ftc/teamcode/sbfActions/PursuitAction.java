package org.firstinspires.ftc.teamcode.sbfActions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Pursuit;
import org.firstinspires.ftc.teamcode.sbfHardware.Robot;
import org.firstinspires.ftc.teamcode.Path;

import java.util.concurrent.TimeUnit;


/**
 * Loaded into the run map as an action that drives the robot. Each action is parameterized by the CSV file.
 *
 * @author Andrew, 8668 SBF
 * @see RobotAction
 * */
public class PursuitAction extends RobotAction
{
    Path thePath = new Path();

    /** The robot's power. */
    double thePower;
    /** The number by which the robot will correct error. */
//    double theGain;
    /** The distance the robot will drive. */
    double xPoint;
    double yPoint;
    double theHeading;
    Pursuit thePursuit;

    // internal time tracking
    private long startTime = 0; // in nanoseconds


    /** Creates a new object from the supplied parameters. */
    PursuitAction(String id, String nextAction, double duration, double power, double heading, double x, double y)
    {
        super(id, nextAction, duration);
        thePower = power;
//        theGain = .01;
        xPoint = x;
        yPoint = y;
        theHeading = heading;
//        timeout = duration;
//        theId = id;
//
//        if(nextAction.isEmpty())
//        {
//            theNextAction = null;
//        }
//        else
//        {
//            theNextAction = nextAction;
//        }
    }

    /** Takes the parameters from the CSV file, converts them appropriately, and calls the
     * parameterized constructor */
    PursuitAction(String[] params)
    {
        this(params[0],
             params[1],
             Double.parseDouble(params[2]),
             Double.parseDouble(params[3]),
             Double.parseDouble(params[4]),
             Double.parseDouble(params[5]),
             Double.parseDouble(params[6]));

        this.addPoint(params);
    }

    /** Placeholder for initialization. */
    @Override
    public void init(Telemetry telem, Robot theRobot)
    {
        thePursuit = new Pursuit((float)xPoint, (float)yPoint, telem);
//        thePath= new Path();
        super.init(telem, theRobot);

    }

    /** Placeholder for entry. */
    @Override
    public void entry()
    {
//        thePath.addPoint((float)xPoint, (float)yPoint, thePower, theHeading);
        thePursuit.location.x = robot.location.x;
        thePursuit.location.y = robot.location.y;
        resetStartTime();
        super.entry();
    }

    /** The body of the action to be executed. */
    @Override
    public boolean execute()
    {
//        telemetry.addData("distance ", theDistance);
        thePursuit.updateVelocity(robot.getVelocity());
        thePursuit.updatePosition(robot.getLocationChange());
        thePursuit.updateHeading(robot.getHeadingPursuit());
        thePursuit.updateAngularVelocity(robot.getAngularVelocity());

        telemetry.addData("current segment: ", thePursuit.currentSegment);


        thePursuit.elapsedTime = getRuntime();
        thePursuit.follow(thePath);
        robot.updateMotors(thePursuit.desiredVelocity.copy(), thePursuit.joystickAngularVelocity);
        return thePursuit.getDone() || super.execute();
//        return false;
    }


    /** Stops all the motors on the robot and calls the p
     * arent exit method. */
    @Override
    public void exit()
    {
        robot.stopChassis();
        super.exit();
    }

    public void addPoint(String[] params)
    {
        double x = Double.parseDouble(params[5]);
        double y = Double.parseDouble(params[6]);
        double maxSpeed = Double.parseDouble(params[3]);
        double heading = Double.parseDouble(params[4]);

        thePath.addPoint((float)x, (float)y, maxSpeed, heading);
    }

    /**
     * Get the number of seconds this op mode has been running
     * <p>
     * This method has sub millisecond accuracy.
        * @return number of seconds this op mode has been running
     */
    public double getRuntime() {
        final double NANOSECONDS_PER_SECOND = TimeUnit.SECONDS.toNanos(1);
        return (System.nanoTime() - startTime) / NANOSECONDS_PER_SECOND;
    }

    /**
     * Reset the start time to zero.
     */
    public void resetStartTime() {
        startTime = System.nanoTime();
    }

}
