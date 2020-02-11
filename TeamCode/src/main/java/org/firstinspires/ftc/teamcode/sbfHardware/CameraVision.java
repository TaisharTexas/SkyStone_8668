package org.firstinspires.ftc.teamcode.sbfHardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
//import org.openftc.easyopencv.OpenCvWebcam;

public class CameraVision
{
    OpenCvCamera webCam;
    Telemetry telemetry;

    CameraVision.StageSwitchingPipeline stageSwitchingPipeline;

    public String camDeviceName = "leftCam";

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

        stageSwitchingPipeline.elapsedTime = System.currentTimeMillis();
    }

    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat Cb = new Mat();

        double elapsedTime = 0;

        String SSposition = "null";

        enum Stage
        {
            YCbCr_CHAN2,
            THRESHOLD,
            RAW_IMAGE,
        }

        private CameraVision.StageSwitchingPipeline.Stage stageToRenderToViewport = Stage.RAW_IMAGE;
        private CameraVision.StageSwitchingPipeline.Stage[] stages = CameraVision.StageSwitchingPipeline.Stage.values();

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

        @Override
        public Mat processFrame(Mat input)
        {
            double leftSum = 0;
            double centerSum = 0;
            double rightSum = 0;

            Scalar green = new Scalar(0,250,0);

            int leftL = 24;
            int leftR = 118;
            int rightL = 202;
            int rightR = 288;


            if (System.currentTimeMillis() - elapsedTime > 250 )
            {
                elapsedTime = System.currentTimeMillis();

                Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);
                Core.extractChannel(yCbCrChan2Mat, Cb, 2);
                Imgproc.threshold(Cb, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

                leftSum = Core.sumElems(thresholdMat.submat(96, 144, leftL, leftR)).val[0];
                rightSum = Core.sumElems(thresholdMat.submat(96, 144, rightL, rightR)).val[0];
                centerSum = Core.sumElems(thresholdMat.submat(96, 144, leftR, rightL)).val[0];


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

    public String getSkyStonePosition()
    {
        return stageSwitchingPipeline.SSposition;
    }

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

            Thread th = new Thread(r);
            th.start();
        }

    }

    public void setCamDeviceName(String camDeviceName)
    {
        this.camDeviceName = camDeviceName;
    }
}
