package org.firstinspires.ftc.teamcode.sbfAutonomousClasses;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.SBF_Autonomous;

import java.io.File;

@Autonomous(name="autoRub", group="Zombie")
/** The autonomous class that handles autonomous from the crater side of the lander.
 * This class loads data from a spreadsheet and uses the data to create a sequential list of robot
 * actions.  It lands, samples, goes to the depot, drops the marker, and comes back to the crater.
 *
 * @author Andrew, SBF Robotics
 * @see SBF_Autonomous
 * */
public class autoRun extends SBF_Autonomous
{

    FtcDashboard dashboard;
    TelemetryPacket packet;
    double elapsedTime = 0;

    /** Calls the init methods for needed classes and locates the correct file path to the CSV file
     * for the crater face drive path. */
    public autoRun()
    {
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        autoFile = new File("/storage/9016-4EF8/autoRun.csv");
        startX = 39;
        startY = 9;

    }

    @Override
    public void init()
    {
        elapsedTime = getRuntime();

        offset = -90;
        whichCamera = "rightCam";
        super.init();
    }

    @Override
    public void loop()
    {
        super.loop();

        if (getRuntime() - elapsedTime > 0.1)
        {
            elapsedTime = getRuntime();

            packet.fieldOverlay()
                    .setFill("navy")
                    .fillCircle(robot.location.x - 60, robot.location.y - 60, 4.0);

            dashboard.sendTelemetryPacket(packet);
        }


    }
}
