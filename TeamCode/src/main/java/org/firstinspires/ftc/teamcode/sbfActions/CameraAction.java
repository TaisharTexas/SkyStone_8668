package org.firstinspires.ftc.teamcode.sbfActions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sbfActions.RobotAction;

/**
 * Loaded into the run map as an action that uses the camera. Each action is parameterized by the CSV file.
 *
 * @author Andrew, Error 404: Team Name Not Found
 * @see RobotAction
 * */
public class CameraAction extends RobotAction
{
    /** The location of the gold mineral. */
    String goldPosition = "null";
    /** A truth value that is whether or not the method is done. */
    boolean done = false;
    /** A truth value that is whether or not to shutdown the camera. */
    boolean theShutdown = false;
    /** An int that is the number of times a method has cycled through. */
    int count;

    /** Creates a new object from the supplied parameters. */
    CameraAction(String id, String nextAction, Boolean shutdown) //default nextAction to center
    {
        super( id, nextAction, 2.5);
        theShutdown = shutdown;
    }

    /** Takes the parameters from the CSV file, converts them appropriately, and calls the
     * parameterized constructor */
    CameraAction(String[] params)
    {
        this(params[0], params[1], Boolean.parseBoolean(params[3]));
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

    /** The body of the action to be executed: Based on the location of the gold mineral returned by
     * the goldPosition() method, sets the next action
     * */
    @Override
    public boolean execute()
    {
        try
        {
//            goldPosition = robot.goldPosition();
        }
        catch(Exception e)
        {
            e.printStackTrace();
        }

//        if(goldPosition.equals("right"))
//        {
//            theNextAction = "rightPosition";
//            telemetry.addData("Gold Position: ", goldPosition);
//            done = true;
//        }
        if(goldPosition.equals("center"))
        {
            theNextAction = "centerPosition";
            telemetry.addData("Gold Position: ", goldPosition);
            done = true;
        }
        else if(goldPosition.equals("left"))
        {
            theNextAction = "leftPosition";
            telemetry.addData("Gold Position: ", goldPosition);
            done = true;
        }
//        else if(goldPosition.equals("tweakRight"))
//        {
//            theNextAction = "tweakRight";
//            done = false;
//        }
//        else if(goldPosition.equals("tweakLeft"))
//        {
//            theNextAction = "tweakLeft";
//            done  =  false;
//        }
//        else if(goldPosition.equals("checkRight") && !alreadyChecked)
//        {
//            theNextAction = "checkRight";
//            telemetry.addData("Gold Position: ", goldPosition);
//            done = true;
//            if(theShutdown)
//            {
//                theShutdown = false;
//            }
//            alreadyChecked = true;
//        }
//        else if(alreadyChecked && goldPosition.equals("checkRight"))
//        {
//            telemetry.addData("Gold Position: ", goldPosition);
//            theNextAction = "centerPosition";
//            done = true;
//        }
        else
        {
            done = false;
        }
        telemetry.addData("Gold Position: ", goldPosition);
        telemetry.addData("Camera Action done? ", done);

        return done || super.execute();  // the super.execute is temporary so that we have time to see what is going on.

    }

    /** Shuts down the camera and calls the parent exit method. */
    @Override
    public void exit()
    {
        if(theShutdown)
        {
//            robot.tfodShutdown();
        }
        super.exit();
    }
}


