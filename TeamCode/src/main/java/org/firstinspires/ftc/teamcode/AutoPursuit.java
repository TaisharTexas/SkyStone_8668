package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.sbfActions.ActionMaster;

import java.io.File;

@Autonomous(name="pursuit run", group="pure")
@Config
public class AutoPursuit extends OpMode
{
    ActionMaster theMaster = new ActionMaster();

//    Pursuit pursuit = new Pursuit((float)39.0, (float)9.0, telemetry);
    Pursuit pursuit = new Pursuit((float)0.0, (float)0.0, telemetry);
    Robot robot = new Robot();
    Path drivePath = new Path();


    public void init()
    {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(telemetry, hardwareMap, false);

        File autoFile = new File("/storage/9016-4EF8/auto.csv");
        theMaster.init(telemetry, autoFile, robot);

        robot.getEncoderTelem();

        // TODO: get rid of hte Path object here and use the PursuitAction

        // Set up path
        drivePath.addPoint(0,0,30,0);
        drivePath.addPoint(0,45,15,90);
        drivePath.addPoint(25,45,15,90);
        drivePath.addPoint(15,15,30,45);
        drivePath.addPoint(0,0,15,0);

       // Attempt to map out getting 1 stone in red alliance autonomous
//        drivePath.addPoint(39,9,25,-90);
//        drivePath.addPoint( 43,50,15,-90);
//        drivePath.addPoint( 34,50,10,-90);
//        drivePath.addPoint( 60,36,30,-90);
//        drivePath.addPoint( 121,36,30,0);


    }

    public void start()
    {
        resetStartTime();
        pursuit.elapsedTime = 0;
        robot.start();
    }

    public void loop()
    {

        // TODO: get rid of the pursuit items here and use the PursuitAction and the ActionMaster.
        robot.update();
        telemetry.addData("Robot Heading: ", robot.currentHeading);

        pursuit.updatePosition(robot.getLocationChange());
        pursuit.updateVelocity(robot.getVelocity());


//        telemetry.addData("mp.global velocity: ", bot.velocity);
        pursuit.currentHeading = robot.getHeading();
        pursuit.currentAngularVelocity = robot.getAngularVelocity();

//        telemetry.addData("mp.currentAngularVelocity: ", bot.currentAngularVelocity);

        pursuit.elapsedTime = getRuntime();
        pursuit.follow(drivePath);


        theMaster.execute();
//        robot.updateMotors(pursuit.desiredVelocity.copy(), pursuit.joystickAngularVelocity);

    }

    @Override
    public void stop()
    {
        robot.stop();
        super.stop();
    }
}
