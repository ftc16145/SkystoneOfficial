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

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;

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

@TeleOp(name="No Auto Field Op Assist", group="Mecanum")

public class FieldOperatorAssistNoAuto extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Hardware robot = new Hardware();
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
    double num = 0;
    boolean clawLock = false;
    boolean ypressed = false;
    boolean apressed = false;
    boolean lbump = false;
    boolean ltrig = false;
    boolean gy = false;
    boolean ga = false;
    boolean gb = false;
    boolean xtap = false;
    boolean scoreMode = false;
    int armx, army;

    //public Drivetrain drive;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init( hardwareMap, telemetry, 0,0,true, false );
        telemetry.addData("Status", "Initialized");


        // create a sound parameter that holds the desired player parameters.

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        //drive = Drivetrain.init( 0, 0, 0, Drivetrain.driveType.fourWheel );

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized" );

        //gyro = hardwareMap.get( GyroSensor.class, "gyro" );
        //gyro.calibrate();


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        //robot.visionTeleop();
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
    private void checkArmRestraint(){
        if(armx > 2){
            armx = 2;
        }else if(armx < 1){
            armx = 1;
        }
        if(army > 3){
            army = 3;
        }else if(army < 1){
            army = 1;
        }
        if(armx == 2){
            if(army > 2){
                army = 2;
            }
        }

    }
    @Override
    public void loop() {
        robot.mecanumDriveFieldOrient( gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x );
        if( gamepad2.x && !xtap ){
            xtap = true;
            // ACTION
            scoreMode = !scoreMode;
        }
        if( !gamepad2.x && xtap ){
            xtap = false;
        }





        if( scoreMode ){
            if( gamepad2.left_bumper && !lbump ){
                lbump = true;
                // ACTION
                army++;
            }else if( gamepad2.left_trigger >= 0.5 && !ltrig ){
                ltrig = true;
                // ACTION
                army--;
            }
            if( !gamepad2.left_bumper && lbump ){
                lbump = false;
            }
            if( !( gamepad2.left_trigger >= 0.5 ) && ltrig ){
                ltrig = false;
            }

            if( gamepad2.y && !gy ){
                gy = true;
                // ACTION
                army++;
            }else if( gamepad2.a && !ga ){
                ga = true;
                // ACTION
                army--;
            }
            if( !gamepad2.y && gy ){
                gy = false;
            }
            if( !gamepad2.a && ga ){
                ga = false;
            }

            if( gamepad2.b && !gb ){
                gb = true;
                robot.levelArm();
            }
            if( !gamepad2.b && gb ){
                gb = false;
            }
        }else{
            //double slider = 0;
           // if( gamepad2.y ){
            //    slider=0.5;
            //}else if( gamepad2.a ){
            //    slider=-0.5;
            //}else{
            //    slider=0;
            //}
            robot.armMechanismControls( gamepad2.right_bumper, gamepad2.right_trigger >= 0.5, gamepad2.left_bumper, gamepad2.left_trigger >= 0.5, gamepad2.y ? 0.5 : gamepad1.a ? -0.5 : 0 );
            robot.foundationControls( gamepad2.dpad_down, gamepad2.dpad_up );
        }
        telemetry.addData("RGB",robot.color.red() + " " + robot.color.green() + " " + robot.color.blue() );
        telemetry.addData("Gyro",robot.yaw() );
        telemetry.addData("ScoreMode X/Y",scoreMode + " " + armx + " " + army );
        telemetry.addData("Slide/Claw/Arm Enc",robot.slide.getCurrentPosition() + " " + robot.claw.getCurrentPosition() + " " + robot.arm.getCurrentPosition() );
        telemetry.addData("Status", "Run Time: " + runtime.toString() );

        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

        //playSound("ss_alarm");
        //  drive.stop();
    }

}
