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

package org.firstinspires.ftc.teamcode.Auton.Blue;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auton.AutonHardware;

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

@Autonomous(name="1SFBlue", group="Auto Blue")

public class MiniMegaAutoBlue extends OpMode
{// Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private AutonHardware robot = new AutonHardware();
    int stage = 1;
    Trajectory[] currentTraj;
    char currentBlock;
    double timeOfNewStage;
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
    }
    @Override
    public void init() {
        robot.init( hardwareMap, telemetry,-36,63,false );
        telemetry.addData("Status", "Initialized" );


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
        runtime.reset();
        currentTraj = robot.oneSkystoneFoundAuton();
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if(stage == 1){
            if(robot.actionRequest){
                if(!robot.claw.isBusy()){
                    robot.levelArm();
                    robot.actionRequest = false;
                    nextStage();
                }
            }else{
                robot.drive.followTrajectorySync(currentTraj[0]);
            }
        }else if(stage == 2){
            robot.drive.followTrajectorySync(currentTraj[1]);
            if(robot.actionRequest){
                robot.actionRequest = false;
                nextStage();
            }
        } else if (stage == 3) {
            if(runtime.time(TimeUnit.SECONDS)>timeOfNewStage+2){
                robot.found.setDirection(CRServo.Direction.REVERSE);
                robot.found.setPower(1);
            }else{
                robot.found.setPower(0);
                nextStage();
            }
        }else if(stage == 4){
            if(robot.actionRequest) {
                boolean foundDone = false, clawDone = false;
                if(runtime.time(TimeUnit.SECONDS)>timeOfNewStage+2){
                    robot.found.setDirection(CRServo.Direction.REVERSE);
                    robot.found.setPower(1);
                }else{
                    robot.found.setPower(0);
                    foundDone = true;
                }
                if(!robot.claw.isBusy()){
                    clawDone = true;
                }
                if(foundDone && clawDone){
                    robot.actionRequest = false;
                    robot.setArmPosition(0,0);
                    robot.levelArm();
                    nextStage();
                }
            }else{
                robot.drive.followTrajectorySync(currentTraj[2]);
            }
        }else if(stage == 5){
            if(robot.actionRequest){
                robot.drive.setMotorPowers(0,0,0,0);
                nextStage();
            }else {
                robot.drive.followTrajectorySync(currentTraj[3]);
            }
        }else{
            robot.drive.setMotorPowers(0,0,0,0);
        }


        robot.autoScore();

    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

}
