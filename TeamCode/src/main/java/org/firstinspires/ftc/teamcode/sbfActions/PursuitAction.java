package org.firstinspires.ftc.teamcode.sbfActions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Pursuit;
import org.firstinspires.ftc.teamcode.sbfHardware.Robot;
import org.firstinspires.ftc.teamcode.Path;

import java.util.concurrent.TimeUnit;


/**

 * Loaded into the run map as an action that causes the robot to use Pursuit to follow a path and
 * possibly execute parallel actions along the way.  The segements for the path that will be followed
 * are loaded in from the CSV file.
 *
 * @author Andrew, 8668 Should Be Fine!
 * @see RobotAction
 * @see Path
 * @see Pursuit
 * */
public class PursuitAction extends RobotAction
{
    /**
     *  Holds the information which defines the path segments which the robot should follow using
     *  Pursuit algorithm.
     */
    Path thePath = new Path();

    /**
     * The robot's power.
     */
    double thePower;
    /** The number by which the robot will correct error. */
//    double theGain;
    /** Stores the x component of a path point. */
    double xPoint;
    /** Stores the y component of a path point. */
    double yPoint;
    /** The heading the robot needs to end a path segment at.*/
    double theHeading;
    /** The ID for any aux actions that need to be loaded along with the path points. */
    String theAuxiliaryAction;
    /**
     * Based on the robot's position and velocity, causes the robot to follow a path defined in the
     * Path object provided to it.
     */
    Pursuit thePursuit;

    // internal time tracking
//    private long startTime = 0; // in nanoseconds


    /**
     *  Creates a new object from the supplied parameters.
     * @param id  The unique identifier by which the action is identified.
     * @param nextAction  The id of the next action.
     * @param duration  The maximum time the action can take.
     * @param power  The max power the robot can use when driving.
     * @param heading  The heading the robot needs to be at by the end of a path segment.
     * @param x  The x position the robot needs to drive to.
     * @param y  The y position the robot needs to drive to.
     * @param auxiliaryAction  The id of the auxiliary action (if any) that needs to be loaded into the runmap.
     */
    PursuitAction(String id, String nextAction, double duration, double power, double heading, double x, double y, String auxiliaryAction)
    {
        super(id, nextAction, duration);
        thePower = power;
        xPoint = x;
        yPoint = y;
        theHeading = heading;
        theAuxiliaryAction = auxiliaryAction;
    }

    /**
     * Takes the parameters from the CSV file, converts them appropriately, and calls the
     * parameterized constructor
     *
     * @param params the parameters read from the CSV file
     */
    PursuitAction(String[] params)
    {
        this(params[0],
             params[1],
             Double.parseDouble(params[2]),
             Double.parseDouble(params[3]),
             Double.parseDouble(params[4]),
             Double.parseDouble(params[5]),
             Double.parseDouble(params[6]),
             params[7]);

        this.addPoint(params);
    }

    /**
     * Initialize the Pursuit Action by constructing the Pursuit object which will do the
     * heavy lifting of calculating the commands needed to control the robot along the Path.
     * @param telem the telemetry object used for passing data to the Drivers Station
     * @param theRobot the robot object that will be controlled when the action executes
     * @see Pursuit
     */
    @Override
    public void init(Telemetry telem, Robot theRobot)
    {
        thePursuit = new Pursuit((float)xPoint, (float)yPoint, telem);
        super.init(telem, theRobot);
    }

    /**
     * Called once when this action is loaded in preparation for executing.
     */
    @Override
    public void entry()
    {
        /**
         * Tell Pursuit object where it is located.
         */
        thePursuit.location.x = robot.location.x;
        thePursuit.location.y = robot.location.y;

        robot.setZeroBehavior("BRAKE");
        super.entry();
    }

    /**
     * Executes the pursuit of the path.
     * @return boolean that indicates the pursuit is complete.
     */
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
    }


    /**
     * Stops all of the drive motors so that the robot does not continue moving. Executes once as
     * the very last thing for the action.
     */
    @Override
    public void exit()
    {
        robot.stopChassis();
        super.exit();
        robot.setZeroBehavior("FLOAT");
    }

    /**
     * Adds a point to the Path object from the list of parameters which have been read in from the
     * CSV file.
     * @param params which are the values read in from the CSV file.
     */
    public void addPoint(String[] params)
    {
        double x = Double.parseDouble(params[5]);
        double y = Double.parseDouble(params[6]);
        double maxSpeed = Double.parseDouble(params[3]);
        double heading = Double.parseDouble(params[4]);
        String auxAction = params[7];

        thePath.addPoint((float)x, (float)y, maxSpeed, heading, auxAction);
    }

    /**
     * If the Pursuit has an auxiliary action to perfrom, this will provide the ID of the action.
     * @return String which is the ID of the action to run in parallel
     */
    @Override
    public String getAuxAction()
    {
        return thePursuit.auxAction;
    }

//    /**
//     * Get the number of seconds this op mode has been running
//     * <p>
//     * This method has sub millisecond accuracy.
//        * @return number of seconds this op mode has been running
//     */
//    public double getRuntime() {
//        final double NANOSECONDS_PER_SECOND = TimeUnit.SECONDS.toNanos(1);
//        return (System.nanoTime() - startTime) / NANOSECONDS_PER_SECOND;
//    }
//    /**
//     * Reset the start time to zero.
//     */
//    public void resetStartTime() {
//        startTime = System.nanoTime();
//    }

}
