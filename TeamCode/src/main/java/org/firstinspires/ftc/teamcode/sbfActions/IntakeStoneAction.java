package org.firstinspires.ftc.teamcode.sbfActions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.sbfHardware.Robot;

/**
 * Loaded into the run map as an action that steps through sucking in a stone, lifting and retracting
 * the lift to allow the stone in, and gripping the stone.
 * Each action is parameterized by the CSV file.
 *
 * @author Andrew, 8668 Should Be Fine!
 * @see RobotAction
 * */
public class IntakeStoneAction extends RobotAction
{
    double thePower;
    double timePassed;
    double timeSnapshot;
    boolean done;
    int state;
    boolean theDirection;

    /**
     *  Creates a new object from the supplied parameters.
     * @param id  The unique identifier by which the action is identified.
     * @param nextAction  The id of the next action.
     * @param duration  The maximum time the action can take.
     * @param power  The power that the intake is driven at.
     */    IntakeStoneAction(String id, String nextAction, double duration, double power)
    {
        super(id, nextAction, duration);
        thePower = power;

    }

    /**
     * Takes the parameters from the CSV file, converts them appropriately, and calls the
     *      * parameterized constructor
     * @param params  An array that stores all the parameters that define the action variables.
     */
    IntakeStoneAction(String[] params)
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

    /** Calls the parent entry method, starts the intake, opens the claw, sets state to zero, and
     * sets done to false.  */
    @Override
    public void entry()
    {

        super.entry();
        robot.intake.intakeDrive(thePower);
        robot.lift.releaseClaw();
        state = 0;
        done = false;
    }

    /** Starts a state machine that steps through sucking a stone and lifting/dropping the lift to
     * settle the stone in the right position. */
    @Override
    public boolean execute()
    {
        switch(state)
        {
            case 0:
                if(robot.intake.rampSignal() && robot.location.x < 72)
                {
                    if(robot.lift.vLiftDrive(1,2.5, 2))
                    {
                        timeSnapshot = getRuntime();
                        state++;
                    }

                }
                break;

            case 1:
                timePassed = getRuntime()-timeSnapshot;
                if(robot.intake.backSignal() || timePassed>=2.5)
                {
                    if(robot.lift.vLiftDrive(-1, 3.5, 3.2))
                    {
                        timeSnapshot = getRuntime();
                        state++;
                    }
                }
                break;

            case 2:
                robot.lift.grabClaw();
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

    /** Stops the intake and lift motors and calls the parent exit method. */
    @Override
    public void exit()
    {
        robot.stopIntake();
        robot.stopLift();
        super.exit();
    }
}
