package org.firstinspires.ftc.teamcode.sbfActions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.sbfHardware.Robot;

import java.util.concurrent.TimeUnit;

/**
 *
 *
 * @author Andrew, Error 404: Team Name Not Found
 * */
public class RobotAction
{
    String theId = "";
    double timeout = 0;
    Robot robot = null;
    boolean done = false;
    String theNextAction = "";
    Telemetry telemetry;
    boolean alreadyChecked = false;
    // internal time tracking
    private long startTime = 0; // in nanoseconds

    /** Creates a new object from the supplied parameters. */
    RobotAction( String anID, String next, double theTimeout)
    {
        theId = anID.toUpperCase();
        if(next.isEmpty())
        {
            theNextAction = null;
        }
        else
        {
            theNextAction = next.toUpperCase();
        }

        timeout = theTimeout;
    }

    /** Placeholder for initialization. Currently only calls the parent init method. */
    public void init(Telemetry telem, Robot theRobot)
    {
        telemetry = telem;
        robot = theRobot;
    }

    /** Sets the start time variable equal to the in-system timer.  */
    public void entry()
    {
        startTime = System.nanoTime();
    }

    /** The body of the action to execute. */
    public boolean execute()
    {
        return (getRuntime() >= timeout);
    }

    /** Called at the end of an action. */
    public void exit()
    {

    }

    public String getAuxAction()
    {
        return "NULL";
    }

    /**
     * Get the number of seconds this op mode has been running
     * <p>
     * This method has sub millisecond accuracy.
     * @return number of seconds this op mode has been running
     */
    public double getRuntime()
    {
        final double NANOSECONDS_PER_SECOND = TimeUnit.SECONDS.toNanos(1);
        return (System.nanoTime() - startTime) / NANOSECONDS_PER_SECOND;
    }

    /**
     * Reset the start time to zero.
     */
    public void resetStartTime()
    {
        startTime = System.nanoTime();
    }

}
