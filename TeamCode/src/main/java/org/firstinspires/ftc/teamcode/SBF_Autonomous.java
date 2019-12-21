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
    ActionMaster theMaster = new ActionMaster();

    Pursuit pursuit = new Pursuit((float)39.0, (float)9.0, telemetry);
//    Pursuit pursuit = new Pursuit((float)0.0, (float)0.0, telemetry);
    Robot robot = new Robot();
    Path drivePath = new Path();

    public File autoFile = null;
    public String whichCamera;
    public static String skyStonePosition = "leftPositionTwo";


    public String ssPos = "One";
    public double offset;

    public SBF_Autonomous()
    {

    }


    public void init()
    {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.whichCamera = this.whichCamera;
        robot.init(telemetry, hardwareMap, true, offset);

//        autoFile = new File("/storage/9016-4EF8/autoRun.csv");
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
        theMaster.setFirstAction(ssPos);
        skyStonePosition = ssPos;

//        theMaster.setFirstAction("One");
        resetStartTime();
        pursuit.elapsedTime = 0;
        robot.start();
        robot.stopCamera();
    }

    public void loop()
    {
        robot.update();
        telemetry.addData("Robot Heading: ", robot.getHeadingPursuit());


        ////////////////////////////////////////////////////////////////////////////////////////////

//        pursuit.updatePosition(robot.getLocationChange());
//        pursuit.updateVelocity(robot.getVelocity());
//
//
////        telemetry.addData("mp.global velocity: ", bot.velocity);
//        pursuit.currentHeading = robot.getHeading();
//        pursuit.currentAngularVelocity = robot.getAngularVelocity();
//
////        telemetry.addData("mp.currentAngularVelocity: ", bot.currentAngularVelocity);
//
//        pursuit.elapsedTime = getRuntime();
//        pursuit.follow(drivePath);
//
//        robot.updateMotors(pursuit.desiredVelocity.copy(), pursuit.joystickAngularVelocity);

        ////////////////////////////////////////////////////////////////////////////////////////////

        theMaster.execute();

    }

    @Override
    public void stop()
    {
        robot.stop();
        super.stop();
    }
}
