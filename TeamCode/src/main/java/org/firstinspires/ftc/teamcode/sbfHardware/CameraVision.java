package org.firstinspires.ftc.teamcode.sbfHardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

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

    CameraVision.StageSwitchingPipeline stageSwitchingPipeline;

    public String camDeviceName = "leftCam";

    public void init(HardwareMap hwMap, String whichCam)
    {
        camDeviceName = whichCam;
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
//        webCam = new OpenCvWebcam(hwMap.get(WebcamName.class, "rightCam"), cameraMonitorViewId);
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, camDeviceName), cameraMonitorViewId);
        webCam.openCameraDevice();
        stageSwitchingPipeline = new CameraVision.StageSwitchingPipeline();
        webCam.setPipeline(stageSwitchingPipeline);
        webCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
    }

    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();

        double leftSum = 0;
        double centerSum = 0;
        double rightSum = 0;

        String SSposition = "null";

        enum Stage
        {
            YCbCr_CHAN2,
            THRESHOLD,
            RAW_IMAGE,
        }

//        private CameraVision.StageSwitchingPipeline.Stage stageToRenderToViewport = CameraVision.StageSwitchingPipeline.Stage.YCbCr_CHAN2;
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
            leftSum = 0;
            centerSum = 0;
            rightSum = 0;

            Scalar red = new Scalar(255, 0, 0);
            Scalar green = new Scalar(0,250,0);

            Scalar left = red;
            Scalar center = red;
            Scalar right = red;

            Double leftL = 0.1;
            Double leftR = 0.37;
            Double rightL = 0.63;
            Double rightR = 0.9;

            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            for(int c = (int)(thresholdMat.cols()*leftL); c < (int)(thresholdMat.cols()*leftR); c++)
            {
                for(int r = (int)(thresholdMat.rows()*.4); r <= (thresholdMat.rows()*.6); r++)
                {
                    leftSum = leftSum + (thresholdMat.get(r,c))[0];
                }
            }
            for(int c = (int)(thresholdMat.cols()*leftR); c < (int)(thresholdMat.cols()*rightL); c++)
            {
                for(int r = (int)(thresholdMat.rows()*.4); r <= (thresholdMat.rows()*.6); r++)
                {
                    centerSum = centerSum + (thresholdMat.get(r,c))[0];
                }
            }
            for(int c = (int)(thresholdMat.cols()*rightL); c < (int)(thresholdMat.cols()*rightR); c++)
            {
                for(int r = (int)(thresholdMat.rows()*.4); r <= (thresholdMat.rows()*.6); r++)
                {
                    rightSum = rightSum + (thresholdMat.get(r,c))[0];
                }
            }

            if(leftSum < rightSum && leftSum < centerSum)
            {
                SSposition = "LEFT";
                left = green;
            }
            else if(rightSum < leftSum && rightSum < centerSum)
            {
                SSposition = "RIGHT";
                right = green;
            }
            else
            {
                SSposition = "CENTER";
                center = green;
            }

            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols()*leftL,
                            input.rows()*.39),
                    new Point(
                            input.cols()*leftR,
                            input.rows()*.61),
                    left, 4);
            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols()*leftR+4,
                            input.rows()*.39),
                    new Point(
                            input.cols()*rightL - 4,
                            input.rows()*.61),
                    center, 4);
            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols()*rightL,
                            input.rows()*.39),
                    new Point(
                            input.cols()*rightR,
                            input.rows()*.61),
                    right, 4);

            double maxSum = Math.max(Math.max(leftSum,centerSum), rightSum);
            maxSum = Math.max(1, maxSum);
            Imgproc.putText( input, String.format("%.4f",1-leftSum/maxSum), new Point( 40, 325), Imgproc.FONT_HERSHEY_DUPLEX, 1, left);
            Imgproc.putText( input, String.format("%.4f",1-centerSum/maxSum), new Point( 260, 325), Imgproc.FONT_HERSHEY_DUPLEX, 1, center);
            Imgproc.putText( input, String.format("%.4f",1-rightSum/maxSum), new Point( 470, 325), Imgproc.FONT_HERSHEY_DUPLEX, 1, right);


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

                case RAW_IMAGE:
                {
                    return input;
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
        webCam.stopStreaming();
    }

    public void setCamDeviceName(String camDeviceName)
    {
        this.camDeviceName = camDeviceName;
    }
}
