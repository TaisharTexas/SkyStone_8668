package org.firstinspires.ftc.teamcode.sbfActions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;

/**
 * Loaded into the run map as an action that drives the robot. Each action is parameterized by the CSV file.
 *
 * @author Andrew, 8668 SBF
 * @see RobotAction
 * */
public class PursuitAction extends RobotAction
{
    /** The robot's power. */
    double thePower;
    /** The number by which the robot will correct error. */
    double theGain;
    /** The distance the robot will drive. */
    double xPoint;
    double yPoint;
    double theHeading;

    /** Creates a new object from the supplied parameters. */
    PursuitAction(String id, String nextAction, double power, double heading, double duration, double x, double y)
    {
        super(id, nextAction, duration);

        thePower = power;
        theGain = .01;
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
        super.entry();
    }

    /** The body of the action to be executed: Calls the drive() method in MecanumChassis. */
    @Override
    public boolean execute()
    {
//        telemetry.addData("distance ", theDistance);
//        return robot.drive(thePower, theDirection, theGain, theDistance, timeout);
        return true;  // FIXME
    }


    /** Stops all the motors on the robot and calls the parent exit method. */
    @Override
    public void exit()
    {
//        robot.stopMotors();
        super.exit();
    }

}
