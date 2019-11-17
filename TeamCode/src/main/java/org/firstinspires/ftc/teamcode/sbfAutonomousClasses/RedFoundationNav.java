package org.firstinspires.ftc.teamcode.sbfAutonomousClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoPursuit;

import java.io.File;

@Autonomous(name="RedFoundationNav", group="Zombie")
/** The autonomous class that handles autonomous from the crater side of the lander.
 * This class loads data from a spreadsheet and uses the data to create a sequential list of robot
 * actions.  It lands, samples, goes to the depot, drops the marker, and comes back to the crater.
 *
 * @author Andrew, SBF Robotics
 * @see AutoPursuit
 * */
public class RedFoundationNav extends AutoPursuit
{
    /** Calls the init methods for needed classes and locates the correct file path to the CSV file
     * for the crater face drive path. */
    public RedFoundationNav()
    {
        this.autoFile = new File("/storage/9016-4EF8/RedFoundationNav.csv");
    }
}
