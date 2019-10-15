package org.firstinspires.ftc.teamcode.sbfActions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sbfActions.RobotAction;

/**
 * Loaded into the run map as an action that drives the robot. Each action is parameterized by the CSV file.
 *
 * @author Andrew, Error 404: Team Name Not Found
 * @see RobotAction
 * */
public class DriveAction extends RobotAction
{
    /** The robot's power. */
    double thePower;
    /** The robot's direction. */
    double theDirection;
    /** The number by which the robot will correct error. */
    double theGain;
    /** The distance the robot will drive. */
    double theDistance;

    /** Creates a new object from the supplied parameters. */
    DriveAction(String id, String nextAction, double duration, double power, double direction, double distance)
    {
        super(id, nextAction, duration);

        thePower = power;
        theDirection = direction;
        theGain = .01;
        theDistance = distance;
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
    DriveAction(String[] params)
    {
        this(params[0],
             params[1],
             Double.parseDouble(params[2]),
             Double.parseDouble(params[3]),
             Double.parseDouble(params[4]),
             Double.parseDouble(params[5]));
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
