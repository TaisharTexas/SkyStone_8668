package org.firstinspires.ftc.teamcode.sbfHardware;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.revextensions2.ExpansionHubMotor;
import java.util.Locale;


/**
 * Contains the hardware and usage methods for the Intake mechanism.
 * @author Andrew, 8668 Should Be Fine!
 * */
public class Intake
{
    /** A telemetry object passed down from the opmode. */
    private Telemetry telemetry;
    /** A hardware map object passed down from the opmode. */
    private HardwareMap hardwareMap;

    /** The left intake motor -- initialized as an expansion hub motor. */
    public ExpansionHubMotor leftIntake = null;
    /** The right intake motor -- initialized as an expansion hub motor. */
    public ExpansionHubMotor rightIntake = null;
    /** The ramp distance sensor -- senses when a stone is on the ramp. */
    private DistanceSensor rampSignalD = null;
    /** The back distance sensor -- senses when a stone is ready to be deployed. */
    private DistanceSensor backSignal = null;
    /** The electrical current a motor uses when stalled at full speed. */
    private double stallCurrent = 5100;
    /** A speed gain for the left intake motor in teleop. */
    private static double leftMaxIntakeSpd = 0.95;
    /** A speed gain for the right intake motor in teleop. */
    private static double rightMaxIntakeSpd = 0.9;
    /** A speed gain for the left intake motor in autonomous. */
    private static double leftMaxIntakeSpdAuto = 1;
    /** A speed gain for the right intake motor in autonomous. */
    private static double rightMaxIntakeSpdAuto = .9;
    /** The reading the front sensor returns when a stone is on the ramp. */
    private double stoneDistanceFront = 6;
    /** The reading the rear sensor returns when a stone is positioned for deployment. */
    private double stoneDistanceBack = 6;

    /**
     * Runs once when the init button is pressed on the driver station. Initializes all the hardware
     * used by the class, initiates the telemetry and hardware map objects, and sets any needed
     * variables to their correct starting values.
     * @param telem  A telemetry object passed down from the opmode.
     * @param hwmap  A hardware object passed down from the opmode.
     */
    public void init(Telemetry telem, HardwareMap hwmap)
    {
        telemetry = telem;
        hardwareMap = hwmap;

        try
        {
            leftIntake = hardwareMap.get(ExpansionHubMotor.class, "yEncoder");
            leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        catch (Exception p_exception)
        {
            telemetry.addData("left intake not found in config file","");
            leftIntake = null;
        }
        try
        {
            rightIntake = hardwareMap.get(ExpansionHubMotor.class, "xEncoder");
            rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        catch (Exception p_exception)
        {
            telemetry.addData("right intake not found in config file", "");
            rightIntake = null;
        }


        try
        {
            rampSignalD = hardwareMap.get(DistanceSensor.class, "rampSignal");
        }
        catch (Exception p_exception)
        {
            telemetry.addData("rampSignalD not found in config file", 0);
            rampSignalD = null;
        }
        try
        {
            backSignal = hardwareMap.get(DistanceSensor.class, "backSignal");
        }
        catch (Exception p_exception)
        {
            telemetry.addData("backSignal not found in config file", 0);
            backSignal = null;
        }

    }

    /**
     * Intake - Rotate the intake wheels to eject a stone out of the robot.
     * @param power  The power the intake motors are driven at.
     */
    public void intakeOut(double power)
    {
        intakeDrive(-power);
    }

    /**
     * A method that monitors whether or not a stone is on the intake ramp.
     * @return Returns whether or not the ramp sensor has been triggered.
     */
    public boolean rampSignal()
    {
        // send the info back to driver station using telemetry function.
        telemetry.addData("Distance (cm)", String.format(Locale.US, "%.02f", rampSignalD.getDistance(DistanceUnit.CM)));

        if(!(Double.isNaN(rampSignalD.getDistance(DistanceUnit.CM))))
        {
            if(rampSignalD.getDistance(DistanceUnit.CM) <= stoneDistanceFront)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }

    }

    /**
     * A method that monitors whether or not a stone is in posiiton to be grabbed by the claw.
     * @return Returns whether or not the back distance sensor has been triggered.
     */
    public boolean backSignal()
    {
        // send the info back to driver station using telemetry function.
        telemetry.addData("Distance (cm)", String.format(Locale.US, "%.02f", backSignal.getDistance(DistanceUnit.CM)));

        if(!(Double.isNaN(backSignal.getDistance(DistanceUnit.CM))))
        {
            if(backSignal.getDistance(DistanceUnit.CM) <= stoneDistanceBack)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }

    }


    /**
     * Intake - Rotate the intake wheels to take in a stone into the intake.
     * @param power  The power the intake motors are driven at.
     */
    public void intakeIn(double power)
    {
        intakeDrive(power);
    }



//    private boolean isLeftStalled()
//    {
//
//        if(leftIntake.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS) > stallCurrent)
//        {
//            return true;
//        }
//        else
//        {
//            return false;
//        }
//    }
//
//    private boolean isRightStalled()
//    {
//
//        if(rightIntake.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS) > stallCurrent)
//        {
//            return true;
//        }
//        else
//        {
//            return false;
//        }
//    }

    /** Stops the intake motors. */
    public void intakeStop()
    {
//        xEncoder.setPower(0.0);
        if(leftIntake != null)
        {
            leftIntake.setPower(0.0);
//            leftInSupport2.setPower(0.0);

        }
        else
        {
            telemetry.addData("left intake is null", "cannot use");
        }
//        yEncoder.setPower(0.0);
        if(rightIntake != null)
        {
            rightIntake.setPower(0.0);
//            rightInSupport2.setPower(0.0);
        }
        else
        {
            telemetry.addData("right intake is null", "cannot use");
        }

    }

    /**
     * Drives intake motors & servos.
     * @param power  the power at which the motors drive
     *               - power = eject
     *               + power = intake
     * */
    public void intakeDrive(double power)
    {

            leftIntake.setPower(-power*leftMaxIntakeSpdAuto);
            rightIntake.setPower(-power*rightMaxIntakeSpdAuto);

    }


}

