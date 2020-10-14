package org.firstinspires.ftc.teamcode.sbfHardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.concurrent.TimeUnit;

/**
 * Manages opening and using the webcams for detecting SkyStones.
 *
 * @author Andrew, 8668 Should Be Fine!
 */
public class CameraVision
{
    /**
     * The webcam object which will grab the frames for us
     */
    OpenCvCamera webCam;
    /**
     * Local reference to the telemetry object for displaying debug info
     */
    Telemetry telemetry;
    /**
     * Implements the SkyStone processing alrogithm.
     */
    CameraVision.StageSwitchingPipeline stageSwitchingPipeline;
    /**
     * The ID for the camera we are using (there are 2 on the robot)
     */
    public String camDeviceName = "webcam";

    /**
     * Initialize the vision hardware and provide the telemetry object for debugging.
     * @param hwMap  hardware map for connecting to camera
     * @param whichCam string which identifies the camera to load from the hardware map (left or right)
     * @param telem telemetry object which can be used for debugging
     */
    public void init(HardwareMap hwMap, String whichCam, Telemetry telem)
    {
        telemetry = telem;

        camDeviceName = whichCam;
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, camDeviceName), cameraMonitorViewId);
        webCam.openCameraDevice();

        stageSwitchingPipeline = new CameraVision.StageSwitchingPipeline();
        webCam.setPipeline(stageSwitchingPipeline);
        webCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        stageSwitchingPipeline.elapsedTime.reset();
    }

    /**
     * Implements a SkyStone detection algorithm as an OpenCV pipeline.
     */
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat Cb = new Mat();
        double leftSum = 0;
        double centerSum = 0;
        double rightSum = 0;

        Deadline elapsedTime = new Deadline(250, TimeUnit.MILLISECONDS);

        String SSposition = "null";

        enum Stage
        {
            YCbCr_CHAN2,
            THRESHOLD,
            RAW_IMAGE,
        }

        private CameraVision.StageSwitchingPipeline.Stage stageToRenderToViewport = Stage.RAW_IMAGE;
        private CameraVision.StageSwitchingPipeline.Stage[] stages = CameraVision.StageSwitchingPipeline.Stage.values();

        /**
         * Cycle through which stage of the pipeline is displayed on the screen when the screen is tapped.
         */
        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        /**
         * Called when a frame from the webcam is returned.  Processing and looking for the SkyStone
         * happens in this method.
         * <ol>
         *     <li>Convert the colorspace from RGB to YCrCb so that we can focus on the relative brightness o the stones.</li>
         *     <li>Select the Cb channel to focus on.  </li>
         *     <li>Run a threshold on the Cb channel to isolate the SkyStone.</li>
         *     <li>Sum the pixel values for each region of the frame where a stone exists.</li>
         *     <li>Choose the sum with the lowest value as the SkyStone.  This is due to the darkness
         *         that the SkyStone exhibits with its dark colored label.</li>
         * </ol>
         * @param input the Mat object which is the frame of video
         * @return a Mat object which has been updated with the results of the calculation for the SkyStone.
         */
        @Override
        public Mat processFrame(Mat input)
        {

            Scalar green = new Scalar(0,250,0);

            int leftL = 24;
            int leftR = 118;
            int rightL = 202;
            int rightR = 288;


            if ( elapsedTime.hasExpired() )
            {
                elapsedTime.reset();

                Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);
                Core.extractChannel(yCbCrChan2Mat, Cb, 2);
                Imgproc.threshold(Cb, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

                leftSum = Core.sumElems(thresholdMat.submat(96, 144, leftL, leftR)).val[0];
                rightSum = Core.sumElems(thresholdMat.submat(96, 144, rightL, rightR)).val[0];
                centerSum = Core.sumElems(thresholdMat.submat(96, 144, leftR, rightL)).val[0];
            }

            if (leftSum < rightSum && leftSum < centerSum)
            {
                SSposition = "LEFT";
                Imgproc.rectangle(
                        input,
                        new Point(
                                leftL,
                                96),
                        new Point(
                                leftR,
                                144),
                        green, 4);
            }
            else if (rightSum < leftSum && rightSum < centerSum)
            {
                SSposition = "RIGHT";
                Imgproc.rectangle(
                        input,
                        new Point(
                                rightL,
                                96),
                        new Point(
                                rightR,
                                144),
                        green, 4);
            }
            else
            {
                SSposition = "CENTER";
                Imgproc.rectangle(
                        input,
                        new Point(
                                leftR,
                                96),
                        new Point(
                                rightL,
                                144),
                        green, 4);
            }

            switch (stageToRenderToViewport)
            {
                case YCbCr_CHAN2:
                {
                    return yCbCrChan2Mat;
                }

                case THRESHOLD:
                {
                    return thresholdMat;
                }
                default:
                {
                    return input;
                }
            }
        }
    }

    /**
     * Provide the caller with the position that has been detected for the SkyStone.
     * @return a string which indicates LEFT, CENTER, or RIGHT for the position.
     */
    public String getSkyStonePosition()
    {
        return stageSwitchingPipeline.SSposition;
    }

    /**
     * Causes the camera to stop steaming video so that the CPU and battery can be preserved.  It is
     * unclear how long this process takes, and we have observed it taking more than 5 seconds to
     * complete, so we spin up another thread to take care of it.
     */
    public void stopCamera()
    {
        if(webCam!=null)
        {
            Runnable r = new Runnable()
            {
                @Override
                public void run()
                {
                    webCam.stopStreaming();
                }
            };

            try
            {
                Thread th = new Thread(r);
                th.start();
            }
            catch (Exception e)
            {
                telemetry.addData("Exception thrown when closing the webcam: ", e.getMessage());

            }
        }

    }

    /**
     * Sets the name of a imported camera.
     * @param camDeviceName  The name the camera is set to.
     */
    public void setCamDeviceName(String camDeviceName)
    {
        this.camDeviceName = camDeviceName;
    }
}
