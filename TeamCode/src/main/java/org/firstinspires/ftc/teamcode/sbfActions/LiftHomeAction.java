package org.firstinspires.ftc.teamcode.sbfActions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.sbfHardware.Robot;

/**
 * Loaded into the run map as an action that retracts the horizontal slide and drops the vertical lift.
 * Each action is parameterized by the CSV file.
 *
 * @author Andrew, 8668 Should Be Fine!
 * @see RobotAction
 * */
public class LiftHomeAction extends RobotAction
{
    /** A variable that stores the time elapsed since the last time snapshot. */
    double timePassed;
    /** Used to mark when an action starts. Used to determine the change in time since the snapshot. */
    double timeSnapshot;
    /** A boolean that marks whether or not the action has completed successfully. */
    boolean done;
    /** An int that records the current state that the state machine is currently in. */
    int state;

    /**
     *  Creates a new object from the supplied parameters.
     * @param id  The unique identifier by which the action is identified.
     * @param nextAction  The id of the next action.
     * @param duration  The maximum time the action can take.
     */    LiftHomeAction(String id, String nextAction, double duration)
    {
        super(id, nextAction, duration);

    }

    /**
     * Takes the parameters from the CSV file, converts them appropriately, and calls the
     *      * parameterized constructor
     * @param params  An array that stores all the parameters that define the action variables.
     */
    LiftHomeAction(String[] params)
    {
        this(params[0],
             params[1],
             Double.parseDouble(params[2]));
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

    /** Calls the parent method, sets state to zero, sets done to false, and sets timeSnapshot to
     * the current time.  */
    @Override
    public void entry()
    {
        super.entry();
        state = 0;
        done = false;
        timeSnapshot = getRuntime();
    }

    /** Starts a state machine that brings the lift mechanism home in preparation for collecting
     * another stone. */
    @Override
    public boolean execute()
    {
        telemetry.addData("state: ", state);
        telemetry.addData("done: ", done);
        telemetry.addData("time passed: ", timePassed);
        switch(state)
        {
            //horizontal home
            case 0:
                robot.lift.horizontalDrive(.29);
                if(getRuntime()>1.7)
                {
                    state++;
                }
                break;

            case 1:
                if(robot.lift.vLiftDrive(-1,5.5,2.5))
                {
                    timeSnapshot = getRuntime();
                    state++;
                }
                break;

                //claw open
            case 2:
                robot.lift.releaseClaw();
                timePassed = getRuntime()-timeSnapshot;
                if(timePassed >= 1)
                {
                    state++;
                    done = true;
                }
                break;

            default:
                break;


        }
        return done || super.execute();
    }

    /** Stops the lift motors and calls the parent exit method. */
    @Override
    public void exit()
    {
        robot.stopLift();
        super.exit();
    }
}
