package org.firstinspires.ftc.teamcode.sbfUtil;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 * Contains all the vision code for identifying SkyStones during autonomous
 *
 * @author  Andrew, 8668 SBF
 * */
public class FieldVision
{

    //standard vuforia stuff
    Telemetry telemetry;

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    String returnValue = "null";

    /** The user-unique vuforia license key which allows the use of the vuforia sofware. */
    private static final String VUFORIA_KEY = "Ad6PrLn/////AAABmcL6fnsAN0NVnKxQV1Ko3bJpu5A7aDKat+BYcDQbPbdYejMpWXxDoqI5CO5fBqZtYHbBhcG1jjPL4bS/SvDqwI1He1kkqtw4YnZex3qhNUDsABTRdBeaiTyARIf5fGihCakVaCwzGHcPX6tdmJDofA/Q397J9cndk946HOeSqVAtj5/N8lJIXIyaW8s8rXULNgU7XQvQ0v+CC1O6yecH4/kDIYlXjGREV734h4JAKHFeVNuOB3/y8spjIcRCXRc3WPR80d9dAbs5ZB+NsITpCqjkxHGJOKBGDCI4xbQzDJs1JMTRAUWi+GhlIY2AfLWiNWX1d/R/J9+lq5C7UuqnMiyojSk+gJDD37c5H3D2Q/Ni";

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    /** The camera that vuforia will use. There are two choices for the phone: BACK and FRONT. */
//    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    //initializing all the tensor flow and vuforia navigation stuff
    /**
     * Initializes all the hardware and assets used by vuforia and tensor flow.
     *
     * @param hwMap  An instnce of the FIRST-provided HardwareMap which allows for hardware to be
     *               initilized to the robot.
     * @param telem  An instance of Telemetry which allows this class to use Telemetry.
     * */
    public void init(HardwareMap hwMap, Telemetry telem)
    {
        telemetry = telem;
        initVuforia( hwMap );

        if (ClassFactory.getInstance().canCreateTFObjectDetector())
        {
            initTfod(hwMap);
        }
        else
        {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
    }

    public void start()
    {
        if (tfod != null)
        {
            /** Start tracking the data sets we care about. */
            tfod.activate();
        }
    }



    /**
     *  the tensor flow code
     * @return  a string that tells the location of the gold mineral
     */
    /** Sourced from the FIRST-provided tensor flow example code, this method recognizes minerals
     * in its field of vision and identifies the position of the gold mineral (LEFT, RIGHT, CENTER)
     * by using the positions of the silver minerals and returns that value as a string.
     *
     * Modified by Error 404: Looks for the gold mineral (ie, no longer needs to see silver minerals
     * to calculate the gold mineral position). If it sees a gold mineral that matches the specified
     * height perameter, it then calculates the gold position using the x-axis value of the gold mineral.
     *
     * @return The gold location on the field.
     * */
    public String tensorFlowMineralDetection()
    {

        if (tfod != null)
        {

            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null)
            {
                returnValue = "Did not equal null and is returning something";

                telemetry.addData("# Object Detected", updatedRecognitions.size());

                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions)
                {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f", recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f", recognition.getRight(), recognition.getBottom());
                }
            }
        }

        return returnValue;
    }


    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia( HardwareMap hwMap ) {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod( HardwareMap hwMap ) {
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    /** Shuts down the tensor flow algorithm. Turning it off when it's not needed preserves the
     * robot controller's battery */
    public void tfodShutdown()
    {
        tfod.shutdown();
    }


}
