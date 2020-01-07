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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.VuforiaSkyStoneTrack;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

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

@Autonomous(name="1 Block R Vision", group="Auto Blue")

public class JustBlockRedVision extends OpMode
{// Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Hardware robot = new Hardware();
    int stage = 1;
    double timeOfNewStage;
    double error;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */





    double accelBlock = 0.1;
    VuforiaSkyStoneTrack nav = new VuforiaSkyStoneTrack();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    private void nextStage(){
        stage++;
        timeOfNewStage = runtime.time(TimeUnit.SECONDS);
        robot.setMotorPowers(0,0,0,0);
    }

    @Override
    public void init() {
        robot.init( hardwareMap, telemetry );
        nav.init( hardwareMap );

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
        nav.activate();
        robot.setDriveModes(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        for( DcMotorEx m : robot.drivetrain ){
            m.setTargetPosition(0);
            m.setPower(0);
            m.setPositionPIDFCoefficients(4);
        }
        robot.setDriveModes(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.levelArm();
        robot.operateClaw(true);
        runtime.reset();

    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // First, rotate the  robot to be parallel to the face of the block
        if( stage == 1 ){
            for( DcMotorEx m : robot.drivetrain ){
                // (24/(4*Math.PI))*1120 == 2140
                m.setTargetPosition(2140);
                m.setPower(accelBlock);
            }
            if(accelBlock < 1){
                accelBlock += 0.1;
            }
            if( !(robot.leftFront.isBusy() || robot.leftBack.isBusy() || robot.rightFront.isBusy() || robot.rightBack.isBusy()) ){
                robot.setDriveModes(DcMotorEx.RunMode.RUN_USING_ENCODER);
                robot.mecanumDrive(0,0,0);
                nextStage();
            }
        }else if( stage == 2 ){
            VuforiaSkyStoneTrack.Position pos = nav.getPosition();
            if(pos.isPositionValid){
                accelBlock = 0.1;
                error = pos.y;
                double dir = Math.signum(error);
                robot.mecanumDrive(dir*0.5,0,0);
                if(Math.abs(error) < 0.5){
                    robot.setArmPosition(0,0);
                    nextStage();
                }
            }
        }else if( stage == 3 ){
            if(!(robot.arm.isBusy() || robot.slide.isBusy())){
                robot.arm.setPower(0);
                robot.slide.setPower(0);
                robot.setDriveModes(DcMotorEx.RunMode.RUN_TO_POSITION);
                for( DcMotorEx m : robot.drivetrain ) {
                    // (24/(4*Math.PI))*1120 == 2140
                    m.setTargetPosition(1340);
                    accelBlock = 0.1;
                    m.setPower(accelBlock);
                }
                if(accelBlock < 1){
                    accelBlock += 0.1;
                }
                if( !(robot.leftFront.isBusy() || robot.leftBack.isBusy() || robot.rightFront.isBusy() || robot.rightBack.isBusy()) ){
                    robot.setDriveModes(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    robot.mecanumDrive(0,0,0);
                    robot.operateClaw(false);
                    if(!robot.claw.isBusy()){
                        robot.levelArm();
                        nextStage();
                    }
                }
            }
        }else if( stage == 4 ){
            robot.setDriveModes(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            robot.setDriveModes(DcMotorEx.RunMode.RUN_TO_POSITION);
            for( DcMotorEx m : robot.drivetrain ) {
                m.setTargetPosition(-1340);
                accelBlock = 0.1;
                m.setPower(accelBlock);
            }
            if(accelBlock < 1){
                accelBlock += 0.1;
            }
            if( !(robot.leftFront.isBusy() || robot.leftBack.isBusy() || robot.rightFront.isBusy() || robot.rightBack.isBusy()) ){
                robot.setDriveModes(DcMotorEx.RunMode.RUN_USING_ENCODER);
                robot.mecanumDrive(0,0,0);
                nextStage();
            }
        }else if( stage == 5 ){
            if( runtime.time(TimeUnit.SECONDS) > timeOfNewStage + 3 ){
                nextStage();
            }else{
                robot.mecanumDrive(0.75,0,0);
            }
        }else if( stage == 6 ){
            robot.setDriveModes(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            robot.setDriveModes(DcMotorEx.RunMode.RUN_TO_POSITION);
            for( DcMotorEx m : robot.drivetrain ) {
                m.setTargetPosition(1340);
                accelBlock = 0.1;
                m.setPower(accelBlock);
            }
            if(accelBlock < 1){
                accelBlock += 0.1;
            }
            if( !(robot.leftFront.isBusy() || robot.leftBack.isBusy() || robot.rightFront.isBusy() || robot.rightBack.isBusy()) ){
                robot.setDriveModes(DcMotorEx.RunMode.RUN_USING_ENCODER);
                robot.mecanumDrive(0,0,0);
                robot.operateClaw(true);
                nextStage();
            }
        }else if( stage == 7 ){
            if( runtime.time(TimeUnit.SECONDS) > timeOfNewStage + 2 ){
                nextStage();
            }else{
                robot.mecanumDrive(-1,-0.75,0);
            }
        }else{
            robot.mecanumDrive(0,0,0);
        }
        telemetry.addData("Runtime",runtime.time(TimeUnit.SECONDS));
        telemetry.addData("Stage",stage);
        telemetry.update();


    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        nav.deactivate();
        //  drive.stop();
    }

}
