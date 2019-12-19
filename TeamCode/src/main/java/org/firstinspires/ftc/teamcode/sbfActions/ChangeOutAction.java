package org.firstinspires.ftc.teamcode.sbfActions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SBF_Autonomous;
import org.firstinspires.ftc.teamcode.sbfHardware.Robot;

/**
 * Loaded into the run map as an action that uses the camera. Each action is parameterized by the CSV file.
 *
 * @author Andrew, Error 404: Team Name Not Found
 * @see RobotAction
 * */
public class ChangeOutAction extends RobotAction
{
    /** The location of the gold mineral. */
    String ssPosition = "LEFT";
    /** A truth value that is whether or not the method is done. */
    boolean done = false;
    /** An int that is the number of times a method has cycled through. */
    int count;

    /** Creates a new object from the supplied parameters. */
    ChangeOutAction(String id, String nextAction, double timeout) //default nextAction to center
    {
        super( id, nextAction, timeout);

    }

    /** Takes the parameters from the CSV file, converts them appropriately, and calls the
     * parameterized constructor */
    ChangeOutAction(String[] params)
    {
        this(params[0],
             params[1],
             Double.parseDouble(params[2]));
    }


    /** Placeholder for initialization. Currently only calls the parent init method. */
    @Override
    public void init(Telemetry telem, Robot theRobot)
    {
        ssPosition = SBF_Autonomous.skyStonePosition;
        super.init(telem, theRobot);
    }

    /** Sets the count variable to zero and calls the parent init method.  */
    public void entry()
    {
        //robot.start();
        count = 0;
        super.entry();
    }

    /** The body of the action to be executed: Based on the location of the gold mineral returned by
     * the ssPosition() method, sets the next action
     * */
    @Override
    public boolean execute()
    {

        if(ssPosition.equals("RIGHT"))
        {
            theNextAction = "rightPositionTwo";
            telemetry.addData("SS Position: ", ssPosition);
            done = true;
        }
        else if(ssPosition.equals("CENTER"))
        {
            theNextAction = "centerPositionTwo";
            telemetry.addData("SS Position: ", ssPosition);
            done = true;
        }
        else if(ssPosition.equals("LEFT"))
        {
            theNextAction = "leftPositionTwo";
            telemetry.addData("SS Position: ", ssPosition);
            done = true;
        }
        else
        {
            done = false;
        }
        telemetry.addData("SS Position: ", ssPosition);
        telemetry.addData("Camera Action done? ", done);

        return done || super.execute();  // the super.execute is temporary so that we have time to see what is going on.

    }

    /** Shuts down the camera and calls the parent exit method. */
    @Override
    public void exit()
    {
        super.exit();
    }
}


