package org.firstinspires.ftc.teamcode.sbfActions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.sbfHardware.Robot;

/**
 * Loaded into the run map as an action that grips a stone and deposits it onto the foundation.
 * Each action is parameterized by the CSV file.
 *
 * @author Andrew, 8668 Should Be Fine!
 * @see RobotAction
 * */
public class DeployStoneAction extends RobotAction
{
    /** A variable that stores the time elapsed since the last time snapshot. */
    double timePassed;
    /** Used to mark when an action starts. Used to determine the change in time since the snapshot. */
    double timeSnapshot;
    /** A boolean that marks whether or not the action has completed successfully. */
    boolean done;
    /** An int that records the current state that the state machine is currently in. */
    int state;

    /** Creates a new object from the supplied parameters. */
    DeployStoneAction(String id, String nextAction, double duration)
    {
        super(id, nextAction, duration);

    }

    /** Takes the parameters from the CSV file, converts them appropriately, and calls the
     * parameterized constructor */
    DeployStoneAction(String[] params)
    {
        this(params[0],
             params[1],
             Double.parseDouble(params[2]));
    }

    /** Placeholder for initialization. Currently only calls the parent init method. */
    @Override
    public void init(Telemetry telem, Robot theRobot)
    {
        super.init(telem, theRobot);
    }

    /** Sets state, timeSnapshot, and timePassed to zero. Sets done equal to false and calls the
     * parent entry method.  */
    @Override
    public void entry()
    {
        state = 0;
        timeSnapshot=0;
        timePassed=0;
        done = false;
        super.entry();
    }

    /** Starts a state machine that steps through lifting up and depositing the stone onto the foundation. */
    @Override
    public boolean execute()
    {
        switch(state)
        {
            //intake up
            case 0:
                robot.lift.grabClaw();
                if(robot.lift.vLiftDrive(1,5,2))
                {
                    timeSnapshot = getRuntime();
                    state++;
                }
//                if(robot.location.x > 72)
//                {
//                    if(robot.lift.vLiftDrive(1,5,2))
//                    {
//                        timeSnapshot = getRuntime();
//                        state++;
//                    }
//
//                }
                break;

                //horizontal out
            case 1:
                robot.lift.horizontalDrive(.55);
                timePassed = getRuntime()-timeSnapshot;
                if(timePassed >= 2)
                {
                    state++;

                }
                break;

                //lift down
            case 2:

                if(robot.lift.vLiftDrive(-1,5,2))
                {
                    robot.lift.releaseClaw();
                    timeSnapshot = getRuntime();
                    state++;
                }
                break;

                //lift up
            case 3:

                timePassed = getRuntime()-timeSnapshot;
                if(timePassed >= 1)
                {
                    if(robot.lift.vLiftDrive(1,1.5,1.5))
                    {
                        state++;
                        done = true;
                    }
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
