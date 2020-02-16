package org.firstinspires.ftc.teamcode.sbfActions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.sbfHardware.Robot;

/**
 * Loaded into the run map as an action that drives the robot. Each action is parameterized by the CSV file.
 *
 * @author Andrew, 8668 Should Be Fine!
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
    /** The power at which the intake motors will spin. */
    double theIntake;

    /**
     *  Creates a new object from the supplied parameters.
     * @param id  The unique identifier by which the action is identified.
     * @param nextAction  The id of the next action.
     * @param duration  The maximum time the action can take.
     * @param power  The power at which the robot drives.
     * @param direction  The direction in which the robot will drive.
     * @param distance  The distance the robot will drive.
     * @param intake  The power at which the intake will drive.
     */
    DriveAction(String id, String nextAction, double duration, double power, double direction,
                double distance, double intake)
    {
        super(id, nextAction, duration);

        thePower = power;
        theDirection = direction;
        theGain = .03;
        theDistance = distance;
        theIntake = intake;
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
    DriveAction(String[] params)
    {
        this(params[0],
             params[1],
             Double.parseDouble(params[2]),
             Double.parseDouble(params[3]),
             Double.parseDouble(params[4]),
             Double.parseDouble(params[5]),
             Double.parseDouble(params[6]));
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

    /** The body of the action to be executed: Calls the drive() method in robot. */
    @Override
    public boolean execute()
    {
//        telemetry.addData("distance ", theDistance);
//        robot.updateVelocity(robot.getVelocity());
//        robot.updatePosition(robot.getLocationChange());
//        robot.updateHeading(robot.getHeadingPursuit());
//        robot.updateAngularVelocity(robot.getAngularVelocity());
        return robot.drive(thePower, theDirection, theGain, theDistance, timeout, theIntake);
    }


    /** Stops the drive and intake motors and calls the parent exit method. */
    @Override
    public void exit()
    {
        robot.stopChassis();
        robot.stopIntake();
        super.exit();
    }

}
