package org.firstinspires.ftc.teamcode.sbfActions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.sbfHardware.Robot;

/**
 * Loaded into the run map as an action that turns the robot. Each action is parameterized by the CSV file.
 *
 * @author Andrew, Error 404: Team Name Not Found
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

    /** Creates a new object from the supplied parameters. */
    IntakeStoneAction(String id, String nextAction, double duration, double power)
    {
        super(id, nextAction, duration);
        thePower = power;

    }

    /** Takes the parameters from the CSV file, converts them appropriately, and calls the
     * parameterized constructor */
    IntakeStoneAction(String[] params)
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

        super.entry();
        robot.intake.intakeDrive(thePower, true, true);
        robot.lift.releaseClaw();
        state = 0;
        done = false;
    }

    /** Calls the pointTurn() method in MecanumChassis. */
    @Override
    public boolean execute()
    {
        switch(state)
        {
            case 0:
                if(robot.intake.rampSignal() /*&& robot.location.x < 72*/)
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

    /** Stops all the motors and calls the parent exit method. */
    @Override
    public void exit()
    {
        robot.stopIntake();
        robot.stopLift();
        super.exit();
    }
}
