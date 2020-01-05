package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.VuforiaSkyStoneTrack;

@Autonomous(name="Vuforia Skystone Track", group ="Concept")

public class Auto2WithVision extends LinearOpMode
{
    @Override
    public void runOpMode() {
        VuforiaSkyStoneTrack nav = new VuforiaSkyStoneTrack();

        nav.init(hardwareMap);
        nav.activate();

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

        // waitForStart();

        while (!isStopRequested()) {
            VuforiaSkyStoneTrack.Position pos = nav.getPosition();

            // Provide feedback as to where the robot is located (if we know).
            telemetry.addData("isTargetVisible", pos.isTargetVisible);
            telemetry.addData("isPositionValid", pos.isPositionValid);

            if (pos.isTargetVisible && pos.isPositionValid) {
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        pos.x, pos.y, pos.z );

                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f",
                        pos.roll, pos.pitch, pos.heading );
            }
            else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
        }

        // Disable Tracking when we are done;
        nav.deactivate();
    }
}
