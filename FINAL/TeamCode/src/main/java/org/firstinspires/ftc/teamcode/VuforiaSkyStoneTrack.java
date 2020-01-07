package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class VuforiaSkyStoneTrack
{
    public class Position {
        public boolean     isTargetVisible;
        public String      visibleTargetName;
        public boolean     isPositionValid;
        public double      x, y, z;
        public double      roll, pitch, heading;
    }

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;  // Not Selfie Camera
    private static final boolean PHONE_IS_PORTRAIT = true;

    private static final String VUFORIA_KEY =
            "AU4HXeP/////AAABmVJly8bxo0HGllLzw8tuRc6CrpFD2db1ztBgf+59e8csd4hdmwwlhFHRBy2eue1fUGU2+Vab/tlGrbZyW6L1lUa8lrhvHT4btcGio9P0MZwprrTRCWdeHYjTzuM+gQZMrpbJO5YlaRHNb0EZmDUqw/8Wjx6B7nv90yo/jmcU2c+Z0KI0D0zqIkI7f0AxrrlrMz6kanChap54VsRMZcwhcS1oMuNN0r46XDgzEmNtxuAowf+Q/Bpsn+a1j5VVKK3ydv2L/bUBXoS7eKpXr2N3FpXEnV0CJ6gKthaoPuTSQxFyJlduBTdRi8lhU7lSSERUcf3bctSq+jhe3E3F7yySSVrvFtyLucdi7asvXys17O2v";

    // Since ImageTarget trackables use mm to specify their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    private VuforiaTrackables targetsSkyStone;  // Targets loaded from assets
    private VuforiaTrackable stoneTarget;       // The only target we're going to look for
    private OpenGLMatrix lastLocation;          // Last known location


    public void init( HardwareMap hardwareMap )
    {
        /*
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsSkyStone = vuforia.loadTrackablesFromAsset("Skystone");

        // Give targets recognizable names, only targets we care about.
        stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        // TODO: Adjust these values if needed
        float phoneXRotate    = 0;
        float phoneYRotate    = 0;
        float phoneZRotate    = 0;

        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        // TODO: Get these values correct ASAP
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /* Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : targetsSkyStone) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
    }

    public void activate() {
        // Activate the targets
        // All targets are activate even if we don't track them.
        // TODO: Figure out how to activate only the targets we want to track.
        targetsSkyStone.activate();
    }


    public VuforiaSkyStoneTrack.Position getPosition()
    {
        Position pos = new Position();

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.

        // check the targets, we wish to track, to see which one (if any) is visible.
        // Note: using "targetsToTrack" and not "targetsSkyStone"
        pos.isTargetVisible = false;
        pos.isPositionValid = false;
        if (((VuforiaTrackableDefaultListener)stoneTarget.getListener()).isVisible()) {
            pos.isTargetVisible = true;
            pos.visibleTargetName = stoneTarget.getName();

            // getUpdatedRobotLocation() will return null if no new information is available since
            // the last time that call was made, or if the trackable is not currently visible.
            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)stoneTarget.getListener()).getUpdatedRobotLocation();
            if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;
            }
        }

        if (pos.isTargetVisible && lastLocation != null) {
            pos.isPositionValid = true;

            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            pos.x = translation.get(0) / mmPerInch;
            pos.y = translation.get(1) / mmPerInch;
            pos.z = translation.get(2) / mmPerInch;

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            pos.roll = rotation.firstAngle;
            pos.pitch = rotation.secondAngle;
            pos.heading = rotation.thirdAngle;
        }

        return pos;
    }

    public void deactivate() {
        targetsSkyStone.deactivate();
    }

}
