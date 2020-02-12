package org.firstinspires.ftc.teamcode.sbfActions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.sbfHardware.Robot;

/**
 * Loaded into the run map as an action that turns the robot. Each action is parameterized by the CSV file.
 *
 * @author Andrew, Error 404: Team Name Not Found
 * @see RobotAction
 * */
public class LiftHomeAction extends RobotAction
{
    double timePassed;
    double timeSnapshot;
    boolean done;
    int state;
    boolean theDirection;

    /** Creates a new object from the supplied parameters. */
    LiftHomeAction(String id, String nextAction, double duration)
    {
        super(id, nextAction, duration);

    }

    /** Takes the parameters from the CSV file, converts them appropriately, and calls the
     * parameterized constructor */
    LiftHomeAction(String[] params)
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

    /** Placeholder for entry. Currently only calls the parent entry method.  */
    @Override
    public void entry()
    {

        state = 0;
        done = false;
        timeSnapshot = getRuntime();
        super.entry();
    }

    /** Calls the pointTurn() method in MecanumChassis. */
    @Override
    public boolean execute()
    {
        switch(state)
        {
            //horizontal home
            case 0:
                robot.lift.horizontalDrive(.32);
                timePassed = getRuntime()-timeSnapshot;
                if(timePassed >= 2)
                {
                    timeSnapshot = getRuntime();
                    state++;

                }
                break;

                //lift home
            case 1:
                robot.lift.grabClaw();

                if(robot.lift.vLiftDrive(1,0,2))
                {
                    state++;
                }
                break;

                //claw open
            case 2:
                robot.lift.releaseClaw();
                if(timePassed >= 1)
                {
                    timeSnapshot = getRuntime();
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
        robot.stopLift();
        super.exit();
    }
}
