package org.firstinspires.ftc.teamcode.Auton;

import android.content.Context;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpHardware;
import org.firstinspires.ftc.teamcode.roadrunner.drive.mecanum.SampleMecanumDriveREVOptimized;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class AutonHardware {
    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;
    public DcMotor[] drivetrain;

    public DcMotor slide = null;
    public DcMotor claw = null;
    public DcMotor arm = null;

    public CRServo found = null;

    public ColorSensor color = null;

    HardwareMap hwMap = null;
    Telemetry tel = null;
    boolean onRed;

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = true;

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
    private static final String VUFORIA_KEY =
            "AU4HXeP/////AAABmVJly8bxo0HGllLzw8tuRc6CrpFD2db1ztBgf+59e8csd4hdmwwlhFHRBy2eue1fUGU2+Vab/tlGrbZyW6L1lUa8lrhvHT4btcGio9P0MZwprrTRCWdeHYjTzuM+gQZMrpbJO5YlaRHNb0EZmDUqw/8Wjx6B7nv90yo/jmcU2c+Z0KI0D0zqIkI7f0AxrrlrMz6kanChap54VsRMZcwhcS1oMuNN0r46XDgzEmNtxuAowf+Q/Bpsn+a1j5VVKK3ydv2L/bUBXoS7eKpXr2N3FpXEnV0CJ6gKthaoPuTSQxFyJlduBTdRi8lhU7lSSERUcf3bctSq+jhe3E3F7yySSVrvFtyLucdi7asvXys17O2v";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private OpenGLMatrix blockLocator = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;
    private char option;
    double positionModifier;
    private Trajectory optionA,optionB,optionC, selectedOption;

    private ElapsedTime period = new ElapsedTime();


    String sounds[] = {"ss_alarm", "ss_bb8_down", "ss_bb8_up", "ss_darth_vader", "ss_fly_by",
            "ss_mf_fail", "ss_laser", "ss_laser_burst", "ss_light_saber", "ss_light_saber_long", "ss_light_saber_short",
            "ss_light_speed", "ss_mine", "ss_power_up", "ss_r2d2_up", "ss_roger_roger", "ss_siren", "ss_wookie"};
    boolean soundPlaying = false;
    int soundIndex, soundID;
    SoundPlayer.PlaySoundParams params;
    Context myApp;
    int cameraMonitorViewId;
    VuforiaTrackables targetsSkyStone;
    List<VuforiaTrackable> allTrackables;
    SampleMecanumDriveREVOptimized drive;
    public void playSound(String sound) {
        if (!soundPlaying) {
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
    public void init( HardwareMap hardware, Telemetry atel, double initX, double initY, boolean onRed, boolean useVision ){
        hwMap = hardware;
        tel = atel;
        this.onRed = onRed;
        drive = new SampleMecanumDriveREVOptimized(hwMap);
        drive.setPoseEstimate( new Pose2d(initX, initY, onRed ? Math.toRadians(90) : Math.toRadians(-90) ) );
        claw = hwMap.get(DcMotor.class, "claw");
        slide = hwMap.get(DcMotor.class, "slide");
        arm = hwMap.get(DcMotor.class, "arm");
        claw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        found = hwMap.get(CRServo.class, "foundation");

        color = hwMap.get(ColorSensor.class, "color");
        soundIndex = 0;
        soundID = -1;
        myApp = hwMap.appContext;
        params = new SoundPlayer.PlaySoundParams();
        params.loopControl = 0;
        params.waitForNonLoopingSoundsToFinish = true;
        cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        visionInit();
        playSound("ss_light_saber");
    }
    public void initLoop(){
        visionLoop();
        if(option == 'A'){
            positionModifier = 8;
        }else if(option == 'B'){
            positionModifier = 0;
        }else if(option == 'C'){
            positionModifier = -8;
        }else{
            positionModifier = 0;
        }
    }
    public ArrayList<Trajectory> skystoneFoundAuton(){
        ArrayList<Trajectory> trajectories = new ArrayList<Trajectory>();
        Trajectory stageA, stageB, stageC, stageD, stageE, stageF, stageG;
        int side = onRed ? -1 : 1;
        /*
        * stageA - Go to block
        * Grab block
        * stageB - Go to foundation
        * Drop, Grab, turn 45 towards
        * stageC - Forward 12 into corner
        * Release foundation
        * stageD - Orgin stone grab
        * stageE - get 2nd skystone
        * Grab skystone
        * stageF - return to drop zone
        * Rotate to -45
        * stageC - forward 12
        * Drop block
        * stageG - return to line
        * */

        // Very iffy, but works in the test file (SkystoneOffical\Concepts\skystoneFoundAuton.yaml
        stageA = drive.trajectoryBuilder()
                .splineTo((new Pose2d(positionModifier,30,90)).plus(drive.getPoseEstimate()))
                .build();
        stageB = drive.trajectoryBuilder()
                .splineTo(new Pose2d(12,side*48,0))
                .splineTo(new Pose2d(30,side*12,0))
                .build();
        stageC = drive.trajectoryBuilder()
                .forward(12)
                .build();
        stageD = drive.trajectoryBuilder()
                .splineTo(new Pose2d(0,side*48,45))
                .splineTo(new Pose2d(-36,side*33,90))
                .build();
        stageF = drive.trajectoryBuilder()
                .splineTo(new Pose2d(0,side*48,45))
                .splineTo(new Pose2d(30,side*12,90))
                .build();
        stageG = drive.trajectoryBuilder()
                .splineTo(new Pose2d(0,side*48,0))
                .build();
        if(onRed){
            stageE = drive.trajectoryBuilder()
                    .strafeLeft(24-positionModifier)
                    .build();
        }else{
            stageE = drive.trajectoryBuilder()
                    .strafeRight(24-positionModifier)
                    .build();
        }




        trajectories.add(stageA);
        trajectories.add(stageB);
        trajectories.add(stageC);
        trajectories.add(stageD);
        trajectories.add(stageE);
        trajectories.add(stageF);
        trajectories.add(stageG);
        return trajectories;
    }
    public void visionInit() {

            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

            // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraDirection = CAMERA_CHOICE;

            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(parameters);

            // Load the data sets for the trackable objects. These particular data
            // sets are stored in the 'assets' part of our application.
            targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

            VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
            stoneTarget.setName("Stone Target");
            VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
            blueRearBridge.setName("Blue Rear Bridge");
            VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
            redRearBridge.setName("Red Rear Bridge");
            VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
            redFrontBridge.setName("Red Front Bridge");
            VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
            blueFrontBridge.setName("Blue Front Bridge");
            VuforiaTrackable red1 = targetsSkyStone.get(5);
            red1.setName("Red Perimeter 1");
            VuforiaTrackable red2 = targetsSkyStone.get(6);
            red2.setName("Red Perimeter 2");
            VuforiaTrackable front1 = targetsSkyStone.get(7);
            front1.setName("Front Perimeter 1");
            VuforiaTrackable front2 = targetsSkyStone.get(8);
            front2.setName("Front Perimeter 2");
            VuforiaTrackable blue1 = targetsSkyStone.get(9);
            blue1.setName("Blue Perimeter 1");
            VuforiaTrackable blue2 = targetsSkyStone.get(10);
            blue2.setName("Blue Perimeter 2");
            VuforiaTrackable rear1 = targetsSkyStone.get(11);
            rear1.setName("Rear Perimeter 1");
            VuforiaTrackable rear2 = targetsSkyStone.get(12);
            rear2.setName("Rear Perimeter 2");

            // For convenience, gather together all the trackable objects in one easily-iterable collection */
            allTrackables = new ArrayList<VuforiaTrackable>();
            allTrackables.addAll(targetsSkyStone);


            /**
             * In order for localization to work, we need to tell the system where each target is on the field, and
             * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
             * Transformation matrices are a central, important concept in the math here involved in localization.
             * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
             * for detailed information. Commonly, you'll encounter transformation matrices as instances
             * of the {@link OpenGLMatrix} class.
             *
             * If you are standing in the Red Alliance Station looking towards the center of the field,
             *     - The X axis runs from your left to the right. (positive from the center to the right)
             *     - The Y axis runs from the Red Alliance Station towards the other side of the field
             *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
             *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
             *
             * Before being transformed, each target image is conceptually located at the origin of the field's
             *  coordinate system (the center of the field), facing up.
             */

            // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
            // Rotated it to to face forward, and raised it to sit on the ground correctly.
            // This can be used for generic target-centric approach algorithms
            stoneTarget.setLocation(OpenGLMatrix
                    .translation(0, 0, stoneZ)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

            //Set the position of the bridge support targets with relation to origin (center of field)
            blueFrontBridge.setLocation(OpenGLMatrix
                    .translation(-bridgeX, bridgeY, bridgeZ)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

            blueRearBridge.setLocation(OpenGLMatrix
                    .translation(-bridgeX, bridgeY, bridgeZ)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

            redFrontBridge.setLocation(OpenGLMatrix
                    .translation(-bridgeX, -bridgeY, bridgeZ)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

            redRearBridge.setLocation(OpenGLMatrix
                    .translation(bridgeX, -bridgeY, bridgeZ)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

            //Set the position of the perimeter targets with relation to origin (center of field)
            red1.setLocation(OpenGLMatrix
                    .translation(quadField, -halfField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

            red2.setLocation(OpenGLMatrix
                    .translation(-quadField, -halfField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

            front1.setLocation(OpenGLMatrix
                    .translation(-halfField, -quadField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

            front2.setLocation(OpenGLMatrix
                    .translation(-halfField, quadField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

            blue1.setLocation(OpenGLMatrix
                    .translation(-quadField, halfField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

            blue2.setLocation(OpenGLMatrix
                    .translation(quadField, halfField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

            rear1.setLocation(OpenGLMatrix
                    .translation(halfField, quadField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

            rear2.setLocation(OpenGLMatrix
                    .translation(halfField, -quadField, mmTargetHeight)
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
            if (CAMERA_CHOICE == BACK) {
                phoneYRotate = -90;
            } else {
                phoneYRotate = 90;
            }

            // Rotate the phone vertical about the X axis if it's in portrait mode
            if (PHONE_IS_PORTRAIT) {
                phoneXRotate = 90;
            }

            // Next, translate the camera lens to where it is on the robot.
            // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
            final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
            final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
            final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

            OpenGLMatrix robotFromCamera = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

            /**  Let all the trackable listeners know where the phone is.  */
            for (VuforiaTrackable trackable : allTrackables) {
                ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
            }

            // WARNING:
            // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
            // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
            // CONSEQUENTLY do not put any driving commands in this loop.
            // To restore the normal opmode structure, just un-comment the following line:

            // waitForStart();

            // Note: To use the remote camera preview:
            // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
            // Tap the preview window to receive a fresh image.

            targetsSkyStone.activate();

    }

    public void visionLoop() {

            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;

            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    tel.addData("Visible Target", trackable.getName());
                    targetVisible = true;


                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }

                    break;
                }
            }
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                tel.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                tel.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

            } else {
                tel.addData("Locator Target", "none");
            }

            // Provide feedback as to where the robot is located (if we know).


        }
        //tel.update();

}
