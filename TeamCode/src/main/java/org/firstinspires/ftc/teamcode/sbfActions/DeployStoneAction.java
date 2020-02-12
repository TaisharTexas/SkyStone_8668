package org.firstinspires.ftc.teamcode.sbfActions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.sbfHardware.Robot;

/**
 * Loaded into the run map as an action that turns the robot. Each action is parameterized by the CSV file.
 *
 * @author Andrew, Error 404: Team Name Not Found
 * @see RobotAction
 * */
public class DeployStoneAction extends RobotAction
{
    double timePassed;
    double timeSnapshot;
    boolean done;
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

    /** Placeholder for entry. Currently only calls the parent entry method.  */
    @Override
    public void entry()
    {
        state = 0;
        timeSnapshot=0;
        timePassed=0;
        done = false;
        super.entry();
    }

    /** Calls the pointTurn() method in MecanumChassis. */
    @Override
    public boolean execute()
    {
        switch(state)
        {
            //intake up
            case 0:
                robot.lift.grabClaw();
                if(robot.location.x > 72)
                {

                    if(robot.lift.vLiftDrive(1,5,2))
                    {
                        timeSnapshot = getRuntime();
                        state++;
                    }

                }
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

                if(robot.lift.vLiftDrive(1,0,2))
                {
                    robot.lift.releaseClaw();
                    state++;
                }
                break;

                //lift up
            case 3:

                if(robot.lift.vLiftDrive(1,1.5,1.5))
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
        robot.stopLift();
        super.exit();
    }
}
