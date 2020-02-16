package org.firstinspires.ftc.teamcode.sbfActions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.sbfHardware.Robot;

/**
 * Loaded into the run map as an action that drives the lift a specified distance.
 * Each action is parameterized by the CSV file.
 *
 * @author Andrew, 8668 Should Be Fine!
 * @see RobotAction
 * */
public class LiftAction extends RobotAction
{
    /** The power at which the lift is driven. */
    double thePower;
    /** The distance the lift is driven. */
    double thePosition;
    /** The maximum amount of time the action can take. */
    double timeout;

    /**
     *  Creates a new object from the supplied parameters.
     * @param id  The unique identifier by which the action is identified.
     * @param nextAction  The id of the next action.
     * @param duration  The maximum time the action can take.
     * @param power  The power at which the lift is driven.
     * @param position  The target position of the foundation fingers.
     */
    LiftAction(String id, String nextAction, double duration, double power, double position)
    {
        super(id, nextAction, duration);
        thePower = power;
        thePosition = position;
        timeout = duration;
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

    /**
     * Takes the parameters from the CSV file, converts them appropriately, and calls the
     *      * parameterized constructor
     * @param params  An array that stores all the parameters that define the action variables.
     */
    LiftAction(String[] params)
    {
        this(params[0],
             params[1],
             Double.parseDouble(params[2]),
             Double.parseDouble(params[3]),
             Double.parseDouble(params[4]));
    }

    /**
     * Placeholder for initialization. Currently only calls the parent init method.
     * @param telem  A telemetry object which is passed down from the opmode to where the
     *               hardware is actually used.
     * @param theRobot  A robot action which is passed down from the opmode.
     */
    @Override
    public void init(Telemetry telem, Robot theRobot)
    {
        super.init(telem, theRobot);
    }

    /** Placeholder for entry. Currently only resets the internal timers and calls the parent entry method.  */
    @Override
    public void entry()
    {
        resetStartTime();
        super.entry();
    }

    /** Calls the vLiftDrive method in lift. */
    @Override
    public boolean execute()
    {
        return robot.lift.vLiftDrive(thePower, thePosition, timeout);
//        return robot.liftDrive(thePower, thePosition, timeout);
    }

    /** Stops the lift motors and calls the parent exit method. */
    @Override
    public void exit()
    {
        robot.stopLift();
        super.exit();
    }
}
