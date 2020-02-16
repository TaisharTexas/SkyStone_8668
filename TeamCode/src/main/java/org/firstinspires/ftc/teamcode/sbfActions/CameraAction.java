package org.firstinspires.ftc.teamcode.sbfActions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.sbfHardware.Robot;

/**
 * Loaded into the run map as an action that uses the camera. Each action is parameterized by the CSV file.
 *
 * @author Andrew, 8668 Should Be Fine!
 * @see RobotAction
 * */
public class CameraAction extends RobotAction
{
    /** The location of the sky stone. */
    String ssPosition = "null";
    /** A truth value that is whether or not the method is done. */
    boolean done = false;
    /** A truth value that is whether or not to shutdown the camera. */
    boolean theShutdown = false;
    /** An int that is the number of times a method has cycled through. */
    int count;

    /** Creates a new object from the supplied parameters. */
    CameraAction(String id, String nextAction, double timeout, Boolean shutdown) //default nextAction to center
    {

        super( id, nextAction, timeout);
        theShutdown = shutdown;

    }

    /** Takes the parameters from the CSV file, converts them appropriately, and calls the
     * parameterized constructor */
    CameraAction(String[] params)
    {
        this(params[0],
             params[1],
             Double.parseDouble(params[2]),
             Boolean.parseBoolean(params[3]));
    }


    /** Placeholder for initialization. Currently only calls the parent init method. */
    @Override
    public void init(Telemetry telem, Robot theRobot)
    {
        super.init(telem, theRobot);
    }

    /** Sets the count variable to zero and calls the parent init method.  */
    public void entry()
    {
        //robot.start();
        count = 0;
        super.entry();
    }

    /** The body of the action to be executed: Based on the location of the sky stone returned by
     * the ssPosition variable, sets the next action
     * */
    @Override
    public boolean execute()
    {
        try
        {
            ssPosition = robot.getSkyStonePosition();
        }
        catch(Exception e)
        {
            e.printStackTrace();
        }

        if(ssPosition.equals("RIGHT"))
        {
            theNextAction = "rightPosition";
            telemetry.addData("SS Position: ", ssPosition);
            done = true;
        }
        else if(ssPosition.equals("CENTER"))
        {
            theNextAction = "centerPosition";
            telemetry.addData("SS Position: ", ssPosition);
            done = true;
        }
        else if(ssPosition.equals("LEFT"))
        {
            theNextAction = "leftPosition";
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

    /** Closes the camera and calls the parent exit method. */
    @Override
    public void exit()
    {
        if(theShutdown)
        {
            robot.stopCamera();
        }
        super.exit();
    }
}


