package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.sbfActions.ActionMaster;
import org.firstinspires.ftc.teamcode.sbfHardware.Robot;

import java.io.File;

@Autonomous(name="SBF Autonomous", group="Yeltron")
@Config
public class SBF_Autonomous extends OpMode
{
    public double startX;
    public double startY;

    ActionMaster theMaster = new ActionMaster();

    Pursuit pursuit = new Pursuit((float)startX, (float)startY, telemetry);
//    Pursuit pursuit = new Pursuit((float)0.0, (float)0.0, telemetry);
    public Robot robot = new Robot();
    Path drivePath = new Path();

    public File autoFile = null;
    public String whichCamera;
    public static String skyStonePosition = "leftPositionTwo";


    public String ssPos = "One";
    public double offset;

    public boolean isOff;

    public SBF_Autonomous()
    {

    }


    public void init()
    {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.whichCamera = this.whichCamera;
        robot.init(telemetry, hardwareMap, true, offset);
        robot.location.x = (float)startX;
        robot.location.y = (float)startY;

        isOff = false;

//        autoFile = new File("/storage/9016-4EF8/BluePursuitQuarry.csv");
//        autoFile = new File ("/storage/9016-4EF8/RedFoundationNav.csv");

        telemetry.addData("autofile: ", autoFile);

        theMaster.init(telemetry, autoFile, robot);

        robot.getEncoderTelem();

    }

    @Override
    public void init_loop()
    {
        robot.init_loop();
        super.init_loop();
//        telemetry.addData("which camera: ", whichCamera);
        ssPos = robot.getSkyStonePosition();
//        telemetry.addData("stone position, ", ssPos);
//        telemetry.addData("pursuit heading ", robot.getHeadingPursuit());


    }

    public void start()
    {
        skyStonePosition = ssPos;
        theMaster.setFirstAction(ssPos);

        resetStartTime();
        pursuit.elapsedTime = 0;
        robot.start();
//        robot.stopCamera();
    }

    public void loop()
    {
        if(!isOff)
        {
            robot.stopCamera();
            isOff = true;
            resetStartTime();
        }

        robot.update();
        telemetry.addData("Robot Heading: ", robot.getHeadingPursuit());

        theMaster.execute();

    }

    @Override
    public void stop()
    {
        robot.stop();
        super.stop();
    }
}
