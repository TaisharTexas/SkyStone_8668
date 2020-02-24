package org.firstinspires.ftc.teamcode.sbfAutonomousClasses;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.SBF_Autonomous;

import java.io.File;

@Autonomous(name="autoRun", group="Zombie")
/** The autonomous class that is used to test new actions and drive paths.
 *
 * @author Andrew, 8668 Should Be Fine!
 * @see SBF_Autonomous
 * */
public class autoRun extends SBF_Autonomous
{
    /** A dashboard object used to project data and static variables via the robot controller's
     * wifi direct channel to a web browser. */
    FtcDashboard dashboard;
    /** */
    TelemetryPacket packet;
    /** Stores the time elapsed since a specific mark. */
    double elapsedTime = 0;

    /** Locates the correct file path to the desired CSV file and sets the robot's starting position. */
    public autoRun()
    {
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        autoFile = new File("/storage/9016-4EF8/autoRun.csv");
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
        elapsedTime = getRuntime();

        offset = -90;
        whichCamera = "rightCam";
        super.init();
    }

    /** Runs repeatedly until the stop button is pressed. */
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
