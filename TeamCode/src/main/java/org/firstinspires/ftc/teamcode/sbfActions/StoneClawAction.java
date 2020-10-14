package org.firstinspires.ftc.teamcode.sbfActions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.sbfHardware.Robot;
import org.firstinspires.ftc.teamcode.sbfHardware.StoneClaw;

/**
 * Loaded into the run map as an action that drives the stone claw.
 * Each action is parameterized by the CSV file.
 *
 * @author Andrew, 8668 Should Be Fine!
 * @see RobotAction
 * */
public class StoneClawAction extends RobotAction
{
    StoneClaw.stoneClawPositions thePosition;
    boolean theDirection;

    /** Creates a new object from the supplied parameters.
     * @param id  The unique name for the action
     * @param nextAction  The next action the runmap needs to load.
     * @param duration  The maximum time the action can take before being terminated.
     * @param position  The position the arm needs to move to).
     * */
    StoneClawAction(String id, String nextAction, double duration, String position)
    {
        super(id, nextAction, duration);

        if(position.toUpperCase().equals(StoneClaw.stoneClawPositions.HOME.toString().toUpperCase()))
        {
            thePosition = StoneClaw.stoneClawPositions.HOME;
        }
        else if(position.toUpperCase().equals(StoneClaw.stoneClawPositions.TRANSPORT.toString().toUpperCase()))
        {
            thePosition = StoneClaw.stoneClawPositions.TRANSPORT;
        }
        else if(position.toUpperCase().equals(StoneClaw.stoneClawPositions.GRAB.toString().toUpperCase()))
        {
            thePosition = StoneClaw.stoneClawPositions.GRAB;
        }
        else if(position.toUpperCase().equals(StoneClaw.stoneClawPositions.RELEASE.toString().toUpperCase()))
        {
            thePosition = StoneClaw.stoneClawPositions.RELEASE;
        }
        else
        {
            telemetry.addData("invalid stone arm position", " cannot complete");
        }
    }

    /**
     * Takes the parameters from the CSV file, converts them appropriately, and calls the
     *      * parameterized constructor
     * @param params  An array that stores all the parameters that define the action variables.
     */
    StoneClawAction(String[] params)
    {
        this(params[0],
             params[1],
             Double.parseDouble(params[2]),
             params[3]);
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

        super.entry();
    }

    /** Calls the super execute method. */
    @Override
    public boolean execute()
    {
        robot.stoneClaw.stoneDrive(thePosition);
        return super.execute();
    }

    /** Stops the intake motors and calls the super exit method. */
    @Override
    public void exit()
    {
        super.exit();
    }
}
