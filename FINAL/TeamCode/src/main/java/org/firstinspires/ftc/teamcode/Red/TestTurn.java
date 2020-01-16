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

package org.firstinspires.ftc.teamcode.Red;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.VuforiaSkyStoneTrack;

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

@Autonomous(name="Test 18", group="Red")

public class TestTurn extends OpMode
{// Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Hardware robot = new Hardware();
    int stage = 1;
    char currentBlock;
    double timeOfNewStage;
    boolean foundCube = false;
    boolean reachedTarget = false;
    int failedAttempts = 0;
    VuforiaSkyStoneTrack nav = new VuforiaSkyStoneTrack();
    //private DcMotor leftFront, leftBack, rightFront, rightBack, slide, claw, arm;
    //SLIDE MOTOR
    // 1120 Ticks/rev
    // d = 3cm, r = 1.5cm, C = 3pi cm
    // Dist = ticks/1120 * 3pi
    // 32cm length
    // MAX ENCODER = (32/3pi * 1120) = 3802.7, 3802 ticks+
    //private GyroSensor gyro;
    //DcMotor[] drivetrain;
    //private CRServo found;




    //public Drivetrain drive;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    private void nextStage(){
        stage++;
        timeOfNewStage = runtime.time(TimeUnit.SECONDS);
        robot.setDriveModes(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.setDriveModes(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.setMotorPowers(0,0,0,0);
    }
    private void setStage( int s ){
        stage = s;
        timeOfNewStage = runtime.time(TimeUnit.SECONDS);
        robot.setDriveModes(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.setDriveModes(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.setMotorPowers(0,0,0,0);
    }

    @Override
    public void init() {
        nav.init(hardwareMap);
        robot.init( hardwareMap, telemetry );
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

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        //robot.levelArm();
        nav.activate();
        CameraDevice.getInstance().setFlashTorchMode(true);
        runtime.reset();
        for(DcMotorEx m : robot.drivetrain){
            m.setTargetPosition(0);
            m.setPower(0);
            m.setPositionPIDFCoefficients(1.25);
        }
        robot.setDriveModes(DcMotor.RunMode.RUN_TO_POSITION);

    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // First, rotate the  robot to be parallel to the face of the block
        if(stage == -1){
            //Strafe left
            robot.leftFront.setTargetPosition(725);
            robot.rightBack.setTargetPosition(725);
            robot.leftBack.setTargetPosition(-725);
            robot.rightFront.setTargetPosition(-725);
            robot.setMotorPowers(.4, .4, .4, .4);
            if(nav.getPosition().isPositionValid){
                setStage( 3 );
            }
            if (Math.abs(robot.leftFront.getCurrentPosition() - 725) < 50) {
                setStage( 2 );

            }
        }else if(stage == 1) {
            // Forward 20
            for (DcMotorEx m : robot.drivetrain) {
                m.setTargetPosition(-1785);
                m.setPower(1);
            }
            robot.operateClaw(true);
            robot.levelArm();

                if (Math.abs(robot.leftFront.getCurrentPosition() + 1785) < 50) {
                    nextStage();
            }
        }else if(stage == 2){
            // Try and find target for 3 seconds
            if(failedAttempts <= 1) {
                if (nav.getPosition().isPositionValid) {
                    reachedTarget = true;
                    reachedTarget = false;
                    nextStage();
                } else {
                    if (runtime.time(TimeUnit.SECONDS) > timeOfNewStage + 3) {
                        setStage(-1);
                        failedAttempts++;
                    }

                }
            }else{
                nextStage();
            }
            // Pull the foudnation grabber
            if(runtime.time(TimeUnit.SECONDS) > timeOfNewStage+2 && failedAttempts == 0){
                robot.foundationControls(true,false);
            }else{
                robot.foundationControls(false,false);
                robot.arm.setTargetPosition(260);
                robot.arm.setPower(0.75);
            }
        }else if(stage == 3) {

                // Strafe left
                robot.leftFront.setTargetPosition(725);
                robot.rightBack.setTargetPosition(725);
                robot.leftBack.setTargetPosition(-725);
                robot.rightFront.setTargetPosition(-725);
                robot.setMotorPowers(1, 1, 1, 1);
                if (Math.abs(robot.leftFront.getCurrentPosition() - 725) < 50) {
                    nextStage();
                }

        }else if(stage == 4){
            // Extend arm, move forward
            robot.slide.setTargetPosition(800);
            robot.slide.setPower(0.5);
            for (DcMotorEx m : robot.drivetrain) {
                m.setTargetPosition(-1250);
                m.setPower(1);
            }
            if (Math.abs(robot.leftFront.getCurrentPosition() + 1250) < 50) {
                nextStage();
            }
        }else if(stage == 5){
            robot.operateClaw(false);
            for (DcMotorEx m : robot.drivetrain) {
                m.setTargetPosition(1250);
                m.setPower(1);
            }
            if (Math.abs(robot.leftFront.getCurrentPosition() - 1250) < 50) {
                robot.levelArm();
                nextStage();
            }
        }else if(stage == 6){
            double gyroP = 0.2;
            double error = Math.PI - robot.yaw();
            if(Math.abs(Math.toDegrees(error)) < 5){
                nextStage();
            }else{
                robot.mecanumDrive(0,0,gyroP*error);
            }
        }else if(stage == 7){
            if(runtime.time(TimeUnit.SECONDS)>timeOfNewStage + 5){
                robot.mecanumDrive(0,-0.75,0);
            }else{
                robot.mecanumDrive(0,0,0);
                robot.operateClaw(true);
                nextStage();
            }
            if(runtime.time(TimeUnit.SECONDS)>timeOfNewStage + 3){
                robot.levelArm();
            }else{
                robot.arm.setTargetPosition(200);
                robot.arm.setPower(0.75);
            }
        }else if(stage == 8){
            robot.levelArm();
            for (DcMotorEx m : robot.drivetrain) {
                m.setTargetPosition(2540);
                m.setPower(1);
            }
            if (Math.abs(robot.leftFront.getCurrentPosition() - 2540) < 50) {
                robot.levelArm();
                nextStage();
            }
        }else{
            robot.mecanumDrive(0,0,0);
            robot.foundationControls(false,false);
            robot.armMechanismControls(false,false,false,false,0);
        }
        telemetry.addData("Target",reachedTarget);
        telemetry.addData("Distance",robot.distance.getDistance(DistanceUnit.CM));
        telemetry.addData("Runtime",runtime.time(TimeUnit.SECONDS));
        telemetry.addData("Stage",stage);
        telemetry.addData("X Y Z",nav.getPosition().x + " " + nav.getPosition().y + nav.getPosition().z);
        telemetry.update();


    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

        //  drive.stop();
    }

}
