package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.sbfActions.ActionMaster;
import org.firstinspires.ftc.teamcode.sbfHardware.Robot;

import java.io.File;

/**
 * Contains the top level methods and variables associated with driver controller button presses --
 * runs whatever autonomous drive path is selected and stops after 30 seconds.
 *
 * @author Andrew, 8668 Should Be Fine!
 * @see OpMode
 * */
//@Autonomous(name="SBF Autonomous", group="Yeltron")
@Config
public class SBF_Autonomous extends OpMode
{
    /** The robot's starting x-position -- set by the child autonomous class. */
    public double startX;
    /** The robot's starting y-position -- set by the child autonomous class. */
    public double startY;

    /** Declares an instance of the action master (which is responsible for cataloging the selected csv file. */
    ActionMaster theMaster = new ActionMaster();

    /** Declares an instance of pursuit -- takes a starting x,y position and a telemetry object as inputs.*/
    Pursuit pursuit = new Pursuit((float)startX, (float)startY, telemetry);
    /** Declares an instance of robot. */
    public Robot robot = new Robot();
    /** Declares a new path. */
    Path drivePath = new Path();

    /** The autonomous file to be used. */
    public File autoFile = null;
    /** Selects which camera is to be used (LEFT or RIGHT). */
    public String whichCamera;
    /** The position of the skystone. */
    public static String skyStonePosition = "leftPositionTwo";
    /** The heading offset for the robot (ie, instead of starting at zero, the robot starts at -90)
     * -- set by the child classes. */
    public double offset;

    /** Class constructor. */
    public SBF_Autonomous()
    {

    }

    /** Runs once when INIT is pressed on the driver station.
     * Initializes all the robot hardware, the cameras, pursuit, and the action master.
     * Sets the robot starting position.
     * Sets all the servos to their starting positions.
     * */
    public void init()
    {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.whichCamera = this.whichCamera;
        robot.init(telemetry, hardwareMap, true, offset, false);
        robot.location.x = (float)startX;
        robot.location.y = (float)startY;

//        autoFile = new File("/storage/9016-4EF8/BluePursuitQuarry.csv");
//        autoFile = new File ("/storage/9016-4EF8/RedFoundationNav.csv");

        telemetry.addData("autofile: ", autoFile);

        theMaster.init(telemetry, autoFile, robot);

        robot.getEncoderTelem();

    }

    /** Runs repeatedly after init() is done but before START is pressed -- locates the skystone
     * and stores it to a variable. */
    @Override
    public void init_loop()
    {
        robot.init_loop();
        super.init_loop();
//        telemetry.addData("which camera: ", whichCamera);
        skyStonePosition = robot.getSkyStonePosition();
//        telemetry.addData("stone position, ", ssPos);
//        telemetry.addData("pursuit heading ", robot.getHeadingPursuit());


    }

    /** Runs once when START is pressed on the driver station.
     * Calls the robot start, resets all the timers, and sets the first action based on where the
     * skystone is located.
     * Upon completion initiates loop().*/
    public void start()
    {
        theMaster.setFirstAction(skyStonePosition);
        robot.stopCamera();

        resetStartTime();
        pursuit.elapsedTime = 0;
        robot.start();

    }

    /** Run repeatedly until STOP is pressed.
     * Updates the robot position and calls execute on any actions in the runmap. */
    public void loop()
    {
        try {
            robot.update();
            telemetry.addData("Robot Heading: ", robot.getHeadingPursuit());

            theMaster.execute();
        }
        catch (IndexOutOfBoundsException e)
        {
//            telemetry.addData("Ugh: %s, %s, %s",e.getStackTrace()[1], e.getStackTrace()[2], e.getStackTrace()[3]);
            telemetry.addData("Ugh: ", "%s, %s, %s", e.getStackTrace()[1], e.getStackTrace()[2], e.getStackTrace()[3]);
            robot.stopChassis();
        }
    }

    /** Runs once when STOP is pressed on the driver station.
     * Stops all motors on the robot and terminates any running programs/processes.
     */
    @Override
    public void stop()
    {
        robot.stopEverything();
        super.stop();
    }
}
