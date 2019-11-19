package org.firstinspires.ftc.teamcode.sbfAutonomousClasses;

import org.firstinspires.ftc.teamcode.SBF_Autonomous;
import java.io.File;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="RedNav", group="Zombie")
/** The autonomous class that handles autonomous from the crater side of the lander.
 * This class loads data from a spreadsheet and uses the data to create a sequential list of robot
 * actions.  It lands, samples, goes to the depot, drops the marker, and comes back to the crater.
 *
 * @author Andrew, SBF Robotics
 * @see SBF_Autonomous
 * */
public class RedNav extends SBF_Autonomous
{
    /** Calls the init methods for needed classes and locates the correct file path to the CSV file
     * for the crater face drive path. */
    public RedNav()
    {
        autoFile = new File("/storage/9016-4EF8/RedNav.csv");
    }
}
