package org.firstinspires.ftc.teamcode.sbfAutonomousClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.SBF_Autonomous;

import java.io.File;

@Autonomous(name="RedFoundationRavens", group="Zymbie")
/** The autonomous class that handles autonomous from the crater side of the lander.
 * This class loads data from a spreadsheet and uses the data to create a sequential list of robot
 * actions.  It lands, samples, goes to the depot, drops the marker, and comes back to the crater.
 *
 * @author Andrew, SBF Robotics
 * @see SBF_Autonomous
 * */
public class RedFoundationRavens extends SBF_Autonomous
{
    /** Calls the init methods for needed classes and locates the correct file path to the CSV file
     * for the crater face drive path. */
    public RedFoundationRavens()
    {
        this.autoFile = new File("/storage/9016-4EF8/RedFoundationRavens.csv");
    }

    @Override
    public void init()
    {
        offset = 90;
        whichCamera = "rightCam";
        super.init();
    }
}