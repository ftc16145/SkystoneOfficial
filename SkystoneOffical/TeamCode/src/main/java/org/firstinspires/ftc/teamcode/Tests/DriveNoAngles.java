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

package org.firstinspires.ftc.teamcode.Tests;

import android.content.Context;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.abs;

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

@TeleOp(name="NoAngles", group="Mecanum")
@Disabled
public class DriveNoAngles extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    //private GyroSensor gyro;
    private DcMotor[] drivetrain;
    private CRServo found;
    private Servo test;
    private double num = 0;
    private boolean soundPlaying = false;
    private int soundID;
    private Context myApp;
    private SoundPlayer.PlaySoundParams params;

    private void playSound(String sound) {
        if(!soundPlaying) {
            if ((soundID = myApp.getResources().getIdentifier(sound, "raw", myApp.getPackageName())) != 0) {

                // Signal that the sound is now playing.
                soundPlaying = true;

                // Start playing, and also Create a callback that will clear the playing flag when the sound is complete.
                SoundPlayer.getInstance().startPlaying(myApp, soundID, params, null,
                        new Runnable() {
                            public void run() {
                                soundPlaying = false;
                            }
                        });
            }
        }
    }
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        soundID = -1;
        myApp = hardwareMap.appContext;

        // create a sound parameter that holds the desired player parameters.
        params = new SoundPlayer.PlaySoundParams();
        params.loopControl = 0;
        params.waitForNonLoopingSoundsToFinish = true;
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        //drive = Drivetrain.init( 0, 0, 0, Drivetrain.driveType.fourWheel );

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        leftFront = hardwareMap.get( DcMotor.class, "leftFront" );
        rightFront = hardwareMap.get( DcMotor.class, "rightFront" );
        leftBack = hardwareMap.get( DcMotor.class, "leftBack" );
        rightBack = hardwareMap.get( DcMotor.class, "rightBack" );
        drivetrain = new DcMotor[]{leftFront,leftBack,rightFront,rightBack};
        found = hardwareMap.get( CRServo.class, "foundation");
        test = hardwareMap.get( Servo.class, "test");
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        for( DcMotor d : drivetrain ){
            d.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            d.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
          }
        //gyro = hardwareMap.get( GyroSensor.class, "gyro" );
        //gyro.calibrate();
        playSound("ss_light_saber");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double leftX = gamepad1.left_stick_x;
        double leftY = gamepad1.left_stick_y;
//        double rightX = gamepad1.right_stick_x;

        // We only translating X or Y at one time.
        // We'll move in the direction of the larger of X or Y.
        if( abs(leftX) > abs(leftY) ) {
            // Sideways motion
//            if( leftX < 0.0 ) {
                // Strafe Left
                leftFront.setPower(leftX);
                leftBack.setPower(-leftX);
                rightFront.setPower(-leftX);
                rightBack.setPower(leftX);
//            } else {
//                // Strafe Right
//                leftFront.setPower(-leftX);
//                leftBack.setPower(leftX);
//                rightFront.setPower(leftX);
//                rightBack.setPower(-leftX);
//            }
        } else {
            // Forward motion
            leftFront.setPower(leftY);
            leftBack.setPower(leftY);
            rightFront.setPower(leftY);
            rightBack.setPower(leftY);
        }

//        double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
//        double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
//        double rightX = gamepad1.right_stick_x;
//        final double v1 = r * Math.cos(robotAngle) + rightX;
//        final double v2 = r * Math.sin(robotAngle) - rightX;
//        final double v3 = r * Math.sin(robotAngle) + rightX;
//        final double v4 = r * Math.cos(robotAngle) - rightX;
//        double[] vals = new double[]{v1,v2,v3,v4};
//        double max = 0;
//        for( double v : vals ){
//            if( abs(v) > max ){
//                max = abs(v);
//            }
//        }
//
//        for( int i = 0; i < 4; i++ ) {
//            if (max > 1){
//                vals[i] /= max;
//            }
//            drivetrain[i].setPower(vals[i]);
//        }



        if(gamepad1.dpad_down){
            found.setDirection(DcMotor.Direction.FORWARD);
            found.setPower(1);
            playSound("ss_wookie");
        }else if(gamepad1.dpad_up){
            found.setDirection(DcMotor.Direction.REVERSE);
            found.setPower(1);
            playSound("ss_roger_roger");
        }else{
            found.setPower(0);
        }
        if(gamepad1.a){
            num+=5;
        }else if(gamepad1.y){
            num-=5;
        }
        test.setPosition(num/100);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("y x", gamepad1.left_stick_y + " " + -gamepad1.left_stick_x);
//        telemetry.addData("lf rf lb rb",v1 + " " + v2 + " " + v3 + " " + v4);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        found.setPower(0);
        //  drive.stop();
    }

}
