package org.firstinspires.ftc.teamcode.sbfAutonomousClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.SBF_Autonomous;

import java.io.File;


@Autonomous(name="RedPursuitQuarry", group="Pursuit")
/** The autonomous class that uses pursuit to deliver both skystones, move the foundation, and park
 * under the bridge next to the skybridge.
 *
 * @author Andrew, 8668 Should Be Fine!
 * @see SBF_Autonomous
 * */
public class RedPursuitQuarry extends SBF_Autonomous
{

    /** Locates the Red Pursuit Quarry CSV file and sets the robot's starting position. */
    public RedPursuitQuarry()
    {
        autoFile = new File("/storage/9016-4EF8/RedPursuitQuarry.csv");
        startX = 39;
        startY = 9;
    }

    /** Called once after the init button is pressed.
     *
     * Sets elapsed time to the current time, sets the heading offset, selects the correct camera,
     * and calls the parent method.*/
    @Override
    public void init()
    {
        offset = -90;
//        offset = 0.0;
        whichCamera = "rightCam";
        super.init();
    }
}
