package org.firstinspires.ftc.teamcode.sbfAutonomousClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.SBF_Autonomous;

import java.io.File;

@Disabled
@Autonomous(name="RedFoundationRavens", group="Zombie")
/** The autonomous class that starts on the audience side and moves the foundation before parking
 * under the bridge next to the skybridge.
 *
 * @author Andrew, 8668 Should Be Fine!
 * @see SBF_Autonomous
 * */
public class RedFoundationRavens extends SBF_Autonomous
{
    /** Locates the Red Foundation Ravens CSV file and sets the robot's starting position. */
    public RedFoundationRavens()
    {
        this.autoFile = new File("/storage/9016-4EF8/RedFoundationRavens.csv");
        startX = 39;
        startY = 9;
    }

    /** Called once after the init button is pressed.
     *
     * Sets the heading offset, selects the correct camera, and calls the parent method.*/
    @Override
    public void init()
    {
        offset = 90;
        whichCamera = "rightCam";
        super.init();
    }
}
