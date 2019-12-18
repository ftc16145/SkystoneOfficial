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

package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TeleOp.TeleOpHardware;

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

@Autonomous(name="Skystone Red", group="Auto Blue")

public class AutoAlignSkystone extends OpMode
{// Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private TeleOpHardware robot = new TeleOpHardware();
    boolean stage1 = false;
    boolean stage2= false;
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
    @Override
    public void init() {
        robot.init( hardwareMap, telemetry,0,0,true,true );
        robot.setSearchMode( TeleOpHardware.searchMode.block );
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
        robot.visionTeleop();
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
        // First, rotate the  robot to be parallel to the face of the block
        if (robot.blockxyh() != null) {
            if (stage1) {
                double[] block = robot.blockxyh();
                double degree = Math.toDegrees(block[2]);
                if (Math.abs(degree) > 5) {
                    if ( degree > 0 ) {
                        robot.mecanumDrive(0, 0, -0.3);
                    } else {
                        robot.mecanumDrive(0, 0, 0.3);
                    }

                } else {
                    // End up 12 inches in front of the block
                    robot.mecanumDrive(-0.06 * (robot.blockxyh()[0] + 12), -0.06 * robot.blockxyh()[1], 0);
                    if ( Math.abs( robot.blockxyh()[0] + 12 ) <= 1 && Math.abs(robot.blockxyh()[1] ) <= 0.75 ) {
                        stage1 = false;
                        stage2 = true;
                    }
                }
            } else if ( stage2 ) {
                robot.mecanumDrive(0, 0, 0);
                // RUN CLAW TO POSITION OPEN
                // RUN SLIDE TO POSITION OUT
                // RUN ARM TO POSITION DOWN

            }
            // STAGE 3
            // MOVE FORWARD A FEW INCHES
            // CLOSE CLAW

            // PLAN A
            // STAGE 4
            // LEVEL ARM
            // MOVE BACK, TOWARDS FOUNDATION LONG SIDE

            // STAGE 5
            // DROP SKYSTONE
            // PERFORM FOUNDATION MOVING PROCEDURES FROM FOUNDRED/BLUE

            // PLAN B
            // STAGE 4
            // LEVEL ARM
            // MOVE BACK, TOWARDS FOUNDATION SHORT SIDE

            // STAGE 5
            // DROP SKYSTONE
            // REPEAT ON NEXT SKYSTONE

        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.stop();
        //  drive.stop();
    }

}
