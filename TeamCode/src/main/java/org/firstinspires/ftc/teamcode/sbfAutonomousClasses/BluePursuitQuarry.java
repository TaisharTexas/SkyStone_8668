package org.firstinspires.ftc.teamcode.sbfAutonomousClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.SBF_Autonomous;

import java.io.File;


@Autonomous(name="BluePursuitQuarry", group="Pursuit")
/** The autonomous class that uses pursuit to deliver both skystones, move the foundation, and park
 * under the bridge next to the skybridge.
 *
 * @author Andrew, 8668 Should Be Fine!
 * @see SBF_Autonomous
 * */
public class BluePursuitQuarry extends SBF_Autonomous
{
    /** Locates the Blue Pursuit Quarry CSV file and sets the robot's starting position. */
    public BluePursuitQuarry()
    {
        autoFile = new File("/storage/9016-4EF8/BluePursuitQuarry.csv");
        startX = 105;
        startY = 9;

    }
    /** Called once after the init button is pressed.
     *
     * Sets elapsed time to the current time, sets the heading offset, selects the correct camera,
     * and calls the parent method.*/
    @Override
    public void init()
    {
        offset = 90;
//        offset = 0.0;
        whichCamera = "leftCam";
        super.init();
    }
}
