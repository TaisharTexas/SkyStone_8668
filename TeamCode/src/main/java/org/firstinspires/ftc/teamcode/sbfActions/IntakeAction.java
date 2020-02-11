package org.firstinspires.ftc.teamcode.sbfActions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.sbfHardware.Robot;

/**
 * Loaded into the run map as an action that turns the robot. Each action is parameterized by the CSV file.
 *
 * @author Andrew, Error 404: Team Name Not Found
 * @see RobotAction
 * */
public class IntakeAction extends RobotAction
{
    double thePower;
    boolean theDirection;

    /** Creates a new object from the supplied parameters. */
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

    /** Takes the parameters from the CSV file, converts them appropriately, and calls the
     * parameterized constructor */
    IntakeAction(String[] params)
    {
        this(params[0],
             params[1],
             Double.parseDouble(params[2]),
             Double.parseDouble(params[3]));
    }

    /** Placeholder for initialization. Currently only calls the parent init method. */
    @Override
    public void init(Telemetry telem, Robot theRobot)
    {
        super.init(telem, theRobot);
    }

    /** Placeholder for entry. Currently only calls the parent entry method.  */
    @Override
    public void entry()
    {

        robot.intake.intakeDrive(thePower);
        super.entry();
    }

    /** Calls the pointTurn() method in MecanumChassis. */
    @Override
    public boolean execute()
    {
        return super.execute();
    }

    /** Stops all the motors and calls the parent exit method. */
    @Override
    public void exit()
    {
        robot.stopIntake();
        super.exit();
    }
}
