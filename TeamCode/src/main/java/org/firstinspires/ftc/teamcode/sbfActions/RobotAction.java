package org.firstinspires.ftc.teamcode.sbfActions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.sbfHardware.Robot;

import java.util.concurrent.TimeUnit;

/**
 * The model action that all other actions source from.
 *
 * @author Andrew, 8668 Should Be Fine!
 * */
public class RobotAction
{
    /** The unique identifier by which each action is identified. */
    String theId = "";
    /** The maximum time any one action can take. */
    double timeout = 0;
    /** A robot object that allows child actions to access robot hardware. */
    Robot robot = null;
    /** A boolean that marks whether or not an action is complete. */
    boolean done = false;
    /** The id for the next action to be loaded into the run map. */
    String theNextAction = "";
    /** A telemetry object that lets child actions use telemetry statements. */
    Telemetry telemetry;
    /** A boolean that can be used as a flag marking certain events. */
    boolean alreadyChecked = false;
    // internal time tracking
    /**Internal time tracking. */
    private long startTime = 0; // in nanoseconds

    /**
     *  Creates a new object from the supplied parameters.
     * @param anID  The unique identifier by which the action is identified.
     * @param next  The id of the next action.
     * @param theTimeout  The maximum time the action can take.
     */    RobotAction( String anID, String next, double theTimeout)
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

    /**
     * Placeholder for initialization. Currently only calls the parent init method.
     * @param telem  A telemetry object which is passed down from the opmode to where the
     *               hardware is actually used.
     * @param theRobot  A robot action which is passed down from the opmode.
     */    public void init(Telemetry telem, Robot theRobot)
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

    /** Used to access any auxiliary actions IDs
     *
     * @return returns the ID of the aux action (if any).*/
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
