package org.firstinspires.ftc.teamcode.sbfHardware;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * Contains the hardware and usage methods for the stone claw mechanism.
 * @author Andrew, 8668 Should Be Fine!
 * */
public class StoneClaw
{
    /** A telemetry object passed down from the opmode. */
    private Telemetry telemetry;
    /** A hardware map object passed down from the opmode. */
    private HardwareMap hardwareMap;

    public Servo stoneArm = null;
    public Servo stoneClaw = null;

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
            stoneArm = hardwareMap.get(Servo.class, "stoneArm");

        }
        catch(Exception p_exception)
        {
            telemetry.addData("stoneArm ", "not found in config file");
            stoneArm = null;
        }
        try
        {
            stoneClaw = hardwareMap.get(Servo.class, "stoneClaw");

        }
        catch(Exception p_exception)
        {
            telemetry.addData("stoneClaw ", "not found in config file");
            stoneClaw = null;
        }

    }

    public enum stoneClawPositions
    {
        HOME(1,.28),
        TRANSPORT(2,.32),
        GRAB(3,.32),
        RELEASE(4,.54);

        public final double stoneArm;
        public final double stoneClaw;

        stoneClawPositions(double theStoneArm, double theStoneClaw)
        {

            this.stoneArm = theStoneArm;
            this.stoneClaw = theStoneClaw;
        }
    }

    public void stoneDrive(stoneClawPositions armPosition)
    {
        if(stoneArm!=null)
        {
            stoneArm.setPosition(armPosition.stoneArm);
        }
        if(stoneClaw!=null)
        {
            stoneClaw.setPosition(armPosition.stoneClaw);
        }
    }



}

