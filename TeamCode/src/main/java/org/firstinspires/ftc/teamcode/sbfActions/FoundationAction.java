package org.firstinspires.ftc.teamcode.sbfActions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.sbfHardware.Robot;

/**
 * Loaded into the run map as an action that drives the foundation fingers.
 * Each action is parameterized by the CSV file.
 *
 * @author Andrew, 8668 Should Be Fine!
 * @see RobotAction
 * */
public class FoundationAction extends RobotAction
{
    /** The position that the foundation fingers will be driven to. */
    double thePosition;

    /**
     *  Creates a new object from the supplied parameters.
     * @param id  The unique identifier by which the action is identified.
     * @param nextAction  The id of the next action.
     * @param duration  The maximum time the action can take.
     * @param position  The target position of the foundation fingers.
     */
    FoundationAction(String id, String nextAction, double duration, double position)
    {
        super(id, nextAction, duration);

        thePosition = position;
    }

    /**
     * Takes the parameters from the CSV file, converts them appropriately, and calls the
     *      * parameterized constructor
     * @param params  An array that stores all the parameters that define the action variables.
     */
    FoundationAction(String[] params)
    {
        this(params[0],
             params[1],
             Double.parseDouble(params[2]),
             Double.parseDouble(params[3]));
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

    /** Placeholder for entry. Currently only calls the parent entry method.  */
    @Override
    public void entry()
    {
        super.entry();
    }

    /** The body of the action to be executed: Calls the foundationDrive() method in robot. */
    @Override
    public boolean execute()
    {
        return robot.foundationDrive(thePosition);
    }

    /** Calls the parent exit method. */
    @Override
    public void exit()
    {
        super.exit();
    }
}
