/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="OpenCVBlockBlue", group="Blue")
public class OpenCVBlockB extends OpMode
{// Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Hardware robot = new Hardware();
    int stage = 1;
    char currentBlock;
    double timeOfNewStage;
    boolean foundCube = false;


    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 1.15f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
    private static float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;
    private int adjust = 0;

    OpenCvCamera phoneCam;

    // -1 = left, 0 = center, 1 = right
    int cameraMonitorViewId;
    //public Drivetrain drive;
    int delay = 0;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    private void nextStage(){
        robot.setDriveModes(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.setDriveModes(DcMotorEx.RunMode.RUN_TO_POSITION);
        stage++;
        timeOfNewStage = runtime.time(TimeUnit.SECONDS);
        robot.setMotorPowers(0,0,0,0);
    }
// 9.5 5
    @Override
    public void init() {
        robot.init( hardwareMap, telemetry );
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam  = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new OpenCVBlockB.StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.SIDEWAYS_LEFT);//display on RC
        // create a sound parameter that holds the desired player parameters.

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        //drive = Drivetrain.init( 0, 0, 0, Drivetrain.driveType.fourWheel );

        // Tell the driver that initialization is complete.

        //gyro = hardwareMap.get( GyroSensor.class, "gyro" );
        //gyro.calibrate();


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        robot.initLoop();
        if( valMid == 0 && valLeft == 255 && valRight == 255 ){
            adjust = 0;
        }else if( valMid == 255 && valLeft == 0 && valRight == 255 ){
            adjust = -450;
        }else if( valMid == 255 && valLeft == 255 && valRight == 0 ){
            adjust = 650;
        }else{
            adjust = 0;
        }
        if( runtime.time(TimeUnit.SECONDS) % 3 == 0 ){
            if( adjust == 0 ){
                robot.playSound("ss_darth_vader");
            }else if( adjust == 450 ){
                robot.playSound("ss_bb8_up");
            }else {
                robot.playSound("ss_bb8_down");
            }
        }
        telemetry.addData("Values", valLeft+"   "+valMid+"   "+valRight);
        telemetry.addData("Adjust", adjust);
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        //robot.levelArm();
        runtime.reset();
        robot.setDriveModes(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.setDriveModes(DcMotorEx.RunMode.RUN_TO_POSITION);
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Go Left
        if( stage == 1 ) {
            if( runtime.time(TimeUnit.SECONDS) >= delay ){
                nextStage();
            }
        }else if( stage == 2 ) {
            // 26
            robot.levelArm();
            robot.leftFront.setTargetPosition(-2320);
            robot.leftBack.setTargetPosition(2320);
            robot.rightFront.setTargetPosition(2320);
            robot.rightBack.setTargetPosition(-2320);
            robot.setMotorPowers(1, 1, 1, 1);
            if (Math.abs(robot.leftFront.getCurrentPosition() + 2320) < 50) {
                nextStage();
            }
        }else if( stage == 3 ) {
            // drive by adjust
            robot.leftFront.setTargetPosition(adjust);
            robot.leftBack.setTargetPosition(adjust);
            robot.rightFront.setTargetPosition(adjust);
            robot.rightBack.setTargetPosition(adjust);
            robot.setMotorPowers(1, 1, 1, 1);
            if (Math.abs(robot.leftFront.getCurrentPosition() - adjust) < 50) {
                nextStage();
            }
        }else if( stage == 4 ){
            //6
            robot.leftFront.setTargetPosition(-1750);
            robot.leftBack.setTargetPosition(1750);
            robot.rightFront.setTargetPosition(1750);
            robot.rightBack.setTargetPosition(-1750);
            robot.setMotorPowers(1, 1, 1, 1);
            if (Math.abs(robot.leftFront.getCurrentPosition() + 1750) < 50) {
                nextStage();
            }
        }else if( stage == 5 ){
            // 6 back, grab
            robot.autoGrab.setTargetPosition(-175);
            robot.autoGrab.setPower(1);
            if(!robot.autoGrab.isBusy() || (runtime.time(TimeUnit.SECONDS) > timeOfNewStage + 2) ) {
                robot.leftFront.setTargetPosition(1750);
                robot.leftBack.setTargetPosition(-1750);
                robot.rightFront.setTargetPosition(-1750);
                robot.rightBack.setTargetPosition(1750);
                robot.setMotorPowers(1, 1, 1, 1);
                if (Math.abs(robot.leftFront.getCurrentPosition() - 1750) < 50) {
                    nextStage();
                }
            }
        }else if( stage == 6 ){
            // 48 + adjust
            robot.leftFront.setTargetPosition( ( -5750 ) );
            robot.leftBack.setTargetPosition( ( -5750 ) );
            robot.rightFront.setTargetPosition( ( -5750 ) );
            robot.rightBack.setTargetPosition( ( -5750 ) );
            robot.setMotorPowers(1, 1, 1, 1 );
            if ( Math.abs( robot.leftFront.getCurrentPosition() + ( 5750 ) ) < 50 ) {
                nextStage();
            }
            
        }else if( stage == 7 ){
            // Letgo
            robot.autoGrab.setTargetPosition(0);
            robot.autoGrab.setPower(1);
            robot.operateClaw(true);
            if ( Math.abs( robot.autoGrab.getCurrentPosition() ) < 20 || (runtime.time(TimeUnit.SECONDS) > timeOfNewStage + 2)) {
                nextStage();
            }
        }else if( stage == 8 ){
            // Move back 24
            robot.leftFront.setTargetPosition( 1600 );
            robot.leftBack.setTargetPosition( 1600 );
            robot.rightFront.setTargetPosition( 1600 );
            robot.rightBack.setTargetPosition( 1600 );
            robot.setMotorPowers(1, 1, 1, 1 );
            if ( Math.abs( robot.leftFront.getCurrentPosition() + 1600 ) < 50 ) {
                nextStage();
            }
        }else{
            robot.mecanumDrive(0,0,0);
        }
        telemetry.addData("Distance",robot.distance.getDistance(DistanceUnit.CM));
        telemetry.addData("StageTime",runtime.time(TimeUnit.SECONDS));
        telemetry.addData("Stage",stage);
        telemetry.update();
    }


    /*
     * Code to run ONCE after the driver hits STOP z
     */
    @Override
    public void stop() {

        //  drive.stop();
    }
    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

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
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
                {
                    return all;
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
}

