package org.firstinspires.ftc.teamcode.sbfActions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.sbfHardware.Robot;

/**
 * Loaded into the run map as an action that drives the intake wheels.
 * Each action is parameterized by the CSV file.
 *
 * @author Andrew, 8668 Should Be Fine!
 * @see RobotAction
 * */
public class IntakeAction extends RobotAction
{
    double thePower;
    boolean theDirection;

    /** Creates a new object from the supplied parameters.
     * @param id  The unique name for the action
     * @param nextAction  The next action the runmap needs to load.
     * @param duration  The maximum time the action can take before being terminated.
     * @param power  The power that the intake wheels will spin at (the sign determines the direction).
     * */
    IntakeAction(String id, String nextAction, double duration, double power)
    {
        super(id, nextAction, duration);
        thePower = power;
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

    /**
     * Takes the parameters from the CSV file, converts them appropriately, and calls the
     *      * parameterized constructor
     * @param params  An array that stores all the parameters that define the action variables.
     */
    IntakeAction(String[] params)
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

    /** Drives the intake motors at the specified speed and calls the super entry method.  */
    @Override
    public void entry()
    {

        robot.intakeInWithLights(thePower);
        super.entry();
    }

    /** Calls the super execute method. */
    @Override
    public boolean execute()
    {
        return super.execute();
    }

    /** Stops the intake motors and calls the super exit method. */
    @Override
    public void exit()
    {
        robot.stopIntake();
        super.exit();
    }
}
