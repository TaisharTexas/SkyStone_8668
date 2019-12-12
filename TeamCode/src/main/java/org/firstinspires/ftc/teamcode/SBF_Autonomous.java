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

    String ssPos = "One";

    public SBF_Autonomous()
    {

    }


    public void init()
    {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(telemetry, hardwareMap, false);

//        autoFile = new File("/storage/9016-4EF8/autoRun.csv");
//        autoFile = new File ("/storage/9016-4EF8/RedFoundationNav.csv");

        telemetry.addData("autofile: ", autoFile);

        theMaster.init(telemetry, autoFile, robot);

        robot.getEncoderTelem();

        // TODO: get rid of the Path object here and use the PursuitAction

        // Set up path
//        drivePath.addPoint(0,0,30,0);
//        drivePath.addPoint(0,45,15,90);
//        drivePath.addPoint(25,45,15,90);
//        drivePath.addPoint(15,15,30,45);
//        drivePath.addPoint(0,0,15,0);

       // Attempt to map out getting 1 stone in red alliance autonomous
//        drivePath.addPoint(39,9,25,-90);
//        drivePath.addPoint( 43,55,20,-90);
//        drivePath.addPoint( 30,55,20,-90);
//        drivePath.addPoint( 60,18,25,-90);
//        drivePath.addPoint(90, 25, 30, -45);
//        drivePath.addPoint( 121,40,20,0);
    }

    @Override
    public void init_loop()
    {
        robot.init_loop();
        super.init_loop();
        ssPos = robot.getSkyStonePosition();

    }

    public void start()
    {
        theMaster.setFirstAction(ssPos);
//        theMaster.setFirstAction("One");
        resetStartTime();
        pursuit.elapsedTime = 0;
        robot.start();
    }

    public void loop()
    {

        // TODO: get rid of the pursuit items here and use the PursuitAction and the ActionMaster.
        robot.update();
        telemetry.addData("Robot Heading: ", robot.getHeading());


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
