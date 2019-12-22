package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.roadrunner.drive.mecanum.SampleMecanumDriveREVOptimized;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;


import kotlin.Unit;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class Hardware {
    // Constants
        // Height of pivot above ground
            double hP = 0;
        // Min size of arm
            double dP = 0;
        // Radius of back part of arm
            double rB = 0;
        // Radius of spool wheel
            double rS = 0;
        // Radius of gear
            double rG = 0;


    // Timer
        private ElapsedTime runtime = new ElapsedTime();
    // Is this updating?
    // Scoring Mechanisms
        public SampleMecanumDriveREVOptimized drive;
        public DcMotorEx slide = null;
        public DcMotorEx claw = null;
        public DcMotorEx arm = null;
        double prevXPower, prevYPower;
        ArrayList<DcMotorEx> scoring = new ArrayList<DcMotorEx>();

        public CRServo found = null;
        boolean foundInMovement = false;
        double startTimeFound = 0;

        public ColorSensor color = null;

    // Info for the class
        HardwareMap hwMap = null;
        Telemetry tel = null;
        boolean onRed,auto;

    // Vision
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
            int cameraMonitorViewId;
            VuforiaTrackables targetsSkyStone;
            List<VuforiaTrackable> allTrackables;
            Context myApp;
    // Trajectory Variables
        private char option;
        double positionModifier;
        Pose2d estCurrent;
        private Trajectory optionA,optionB,optionC, selectedOption;
        public boolean actionRequest;



    // Sounds
        String sounds[] = {"ss_alarm", "ss_bb8_down", "ss_bb8_up", "ss_darth_vader", "ss_fly_by",
            "ss_mf_fail", "ss_laser", "ss_laser_burst", "ss_light_saber", "ss_light_saber_long", "ss_light_saber_short",
            "ss_light_speed", "ss_mine", "ss_power_up", "ss_r2d2_up", "ss_roger_roger", "ss_siren", "ss_wookie"};
            boolean soundPlaying = false;
            int soundIndex, soundID;
            SoundPlayer.PlaySoundParams params;





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

    public void init( HardwareMap hardware, Telemetry atel, double initX, double initY, boolean onRed, boolean auto ){
        hwMap = hardware;
        tel = atel;
        this.onRed = onRed;
        this.auto = auto;

        drive = new SampleMecanumDriveREVOptimized(hwMap);
        drive.setPoseEstimate( new Pose2d(initX, initY, onRed ? Math.toRadians(90) : Math.toRadians(-90) ) );
        claw = hwMap.get(DcMotorEx.class, "claw");
        slide = hwMap.get(DcMotorEx.class, "slide");
        arm = hwMap.get(DcMotorEx.class, "arm");

        scoring.add(claw);
        scoring.add(slide);
        scoring.add(arm);

        for( DcMotorEx m : scoring ){
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPositionPIDFCoefficients(1/288);
            m.setTargetPositionTolerance(5);
        }

        found = hwMap.get(CRServo.class, "foundation");

        color = hwMap.get(ColorSensor.class, "color");
        // Random Sound/Vision Things
            soundIndex = 0;
            soundID = -1;
            myApp = hwMap.appContext;
            params = new SoundPlayer.PlaySoundParams();
            params.loopControl = 0;
            params.waitForNonLoopingSoundsToFinish = true;
            cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
            if(auto) {
                visionInit();
            }
        playSound("ss_light_saber");
    }
    public void initLoop(){
        if(auto) {
            visionLoop();
            if (option == 'A') {
                positionModifier = 8;
            } else if (option == 'B') {
                positionModifier = 0;
            } else if (option == 'C') {
                positionModifier = -8;
            } else {
                positionModifier = 0;
            }
        }
    }

    public void hardBrake(){
        drive.setMotorPowers(0,0,0,0);
    }
    public void mecanumDrive(double x, double y, double rot) {
        double nX, nY;
        nX = (Math.abs(x - prevXPower) > 0.1) ? prevXPower + Math.signum(x - prevXPower) * 0.1 : x;
        nY = (Math.abs(y - prevYPower) > 0.1) ? prevYPower + Math.signum(y - prevYPower) * 0.1 : y;
        double r = Math.hypot(-nX, nY);
        double robotAngle = Math.atan2(nY, -nX) - Math.PI / 4;
        double rightX = rot;
        final double v1 = r * Math.cos(robotAngle) - rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) + rightX;
        drive.setMotorPowers(v1,v2,v4,v3);
        // NOW: leftFront, leftBack, rightFront, rightBack
        // NEED: leftFront leftRear rightRear rightFront
        prevXPower = nX;
        prevYPower = nY;
    }

    public void mecanumDriveFieldOrient(double x, double y, double rot) {
        double adjustAngle = -drive.getRawExternalHeading();
        double newX = Math.cos(adjustAngle) * x - Math.sin(adjustAngle) * y;
        double newY = Math.sin(adjustAngle) * x + Math.cos(adjustAngle) * y;
        mecanumDrive(newX, newY, rot);
    }

    public void foundationControls(boolean forward, boolean backward) {
        if (forward) {
            found.setDirection(DcMotor.Direction.FORWARD);
            found.setPower(1);
            playSound("ss_wookie");
        } else if (backward) {
            found.setDirection(DcMotor.Direction.REVERSE);
            found.setPower(1);
            playSound("ss_roger_roger");
        } else {
            found.setPower(0);
        }
    }

    public void armMechanismControls(boolean clawOpen, boolean clawClose, boolean armUp, boolean armDown, double slideControl) {
        if (clawOpen) {
            claw.setPower(0.8);
        } else if (clawClose) {
            claw.setPower(-0.8);
        } else {
            claw.setPower(0);
        }
        if (armUp) {
            arm.setPower(-0.25);
        } else if (armDown) {
            arm.setPower(0.25);
        } else {
            arm.setPower(0);
        }
        slide.setPower(slideControl);
    }


    // NOT DONE
    public void setArmPosition( double x, double y ){
        double angle, rExpand;
        if( y == 0 && x == 0){
            angle = Math.atan( ( 5-hP ) / 13 );
            rExpand = (13 / Math.cos( angle ))-dP;
        }else{
            angle = Math.atan( ( 5*y + 2.25 - hP ) / ( 14.8 + 4*x ) );
            rExpand = ((14.8 + 4*x) / Math.cos( angle )) - dP;
        }
        double armWind = ((rB*Math.sin(-angle))/(2*Math.PI*rS))*288;
        double armExt = rExpand / (2*rG*Math.PI) * 288;
        //if( (armWind > max || armWind < min) && (armExt > max || armExt < min)){
            // OUT OF BOUNDS
        //}else {
            slide.setTargetPosition((int) armExt);
            slide.setPower(0.8);
            arm.setTargetPosition((int) armWind);
            arm.setPower(0.8);
        //}
    }
    public void levelArm(){
        arm.setTargetPosition(0);
        arm.setPower(1);
    }
    // NOT DONE
    public void operateClaw(boolean open){
        //claw.setTargetPosition(open ? : );
        //claw.setPower(0.8);
    }
    public void operateFoundation(boolean up){
        found.setDirection( up ? CRServo.Direction.FORWARD : CRServo.Direction.REVERSE );
        found.setPower(0.8);
        foundInMovement = true;
        startTimeFound = runtime.time(TimeUnit.SECONDS);
    }
    public void autoScore(){
        for( DcMotorEx m : scoring ){
            if(!m.isBusy()){
                m.setPower(0);
            }
        }
    }


    public void logCurrent(Pose2d current){
        estCurrent = current;
    }
    public void requestAction(){
        actionRequest = true;
    }
    public Trajectory[] oneSkystoneFoundAuton(){
        runtime.reset();
        Trajectory traj1,traj2,traj3,traj4;
        int side = onRed ? -1 : 1;
            traj1 = drive.trajectoryBuilder()
                    .addMarker(() -> {
                        // Drop Arm, Open Claw
                        setArmPosition(0,0 );
                        operateClaw(true);
                        return Unit.INSTANCE;
                    })
                    .splineTo(new Pose2d(positionModifier, 30, side * -90).plus(drive.getPoseEstimate()))
                    .addMarker(() -> {
                        // Close Claw, Level Arm
                        operateClaw(false);
                        requestAction();
                        return Unit.INSTANCE;
                    })
                    .build();
            traj2 = drive.trajectoryBuilder()
                    .splineTo(new Pose2d(-24, side * 48, 0))
                    .forward(24)
                    .splineTo(new Pose2d(24, side * 24, side * -90))
                    .splineTo(new Pose2d(30, side * 12, 0))
                    .addMarker(() -> {
                        // Grab foundation, Lower Arm to block 1

                        setArmPosition(1,1);
                        requestAction();
                        return Unit.INSTANCE;
                    })
                    .build();
            traj3 = drive.trajectoryBuilder()
                    .splineTo(new Pose2d(24, 0, side * 59))
                    .forward(30)
                    .addMarker(() -> {
                        logCurrent(drive.getPoseEstimate());
                        operateClaw(true);
                        requestAction();


                        return Unit.INSTANCE;
                    })
                    .build();
            traj4 = drive.trajectoryBuilder()
                    .splineTo(new Pose2d(0, side * 48, 0))
                    .addMarker(()->{
                        requestAction();
                        return Unit.INSTANCE;
                    })
                    .build();

        return new Trajectory[]{traj1,traj2,traj3,traj4};
    }
    public Trajectory[] twoSkystoneFoundAuton(){
        runtime.reset();
        Trajectory traj1,traj2,traj3,traj4,traj5,traj6;
        int side = onRed ? -1 : 1;
        if(option != 'C') {
            traj4 = drive.trajectoryBuilder()
                    .splineTo(new Pose2d(0, side * 48, 0))
                    .splineTo(new Pose2d(-36, side * 48, side * -90))
                    .addMarker(() -> {
                        // Lower Arm\
                        setArmPosition(0,0 );
                        return Unit.INSTANCE;
                    })
                    .splineTo(new Pose2d(-60 + positionModifier, side * 33, side * -90))
                    .build();
        }else{
            traj4 = drive.trajectoryBuilder()
                    .splineTo(new Pose2d(0, side * 48, 0))
                    .splineTo(new Pose2d(-36, side * 48, side * -90))
                    .addMarker(() -> {
                        // Lower Arm
                        setArmPosition(0,0);
                        return Unit.INSTANCE;
                    })
                    .splineTo(new Pose2d(-57.6, side * 30.4, side * -135))
                    .lineTo(new Vector2d(0,4).plus(new Vector2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY())))
                    .addMarker(() -> {
                        // Close Claw, Level Arm
                        operateClaw(false);
                        levelArm();
                        return Unit.INSTANCE;
                    })
                    .build();

        }
        traj1 = drive.trajectoryBuilder()
                .addMarker(() -> {
                    // Drop Arm, Open Claw
                    setArmPosition(0,0);
                    operateClaw(true);
                    return Unit.INSTANCE;
                })
                .splineTo(new Pose2d(positionModifier, 30, side * -90).plus(drive.getPoseEstimate()))
                .addMarker(() -> {
                    // Close Claw, Level Arm
                    operateClaw(false);
                    levelArm();
                    return Unit.INSTANCE;
                })
                .build();
        traj2 = drive.trajectoryBuilder()
                .splineTo(new Pose2d(-24, side * 48, 0))
                .forward(24)
                .splineTo(new Pose2d(24, side * 24, side * -90))
                .splineTo(new Pose2d(30, side * 12, 0))
                .addMarker(() -> {
                    // Grab foundation, Lower Arm to block 1
                    operateFoundation(false);
                    setArmPosition(1,1);
                    return Unit.INSTANCE;
                })
                .build();
        traj3 = drive.trajectoryBuilder()
                .splineTo(new Pose2d(24, 0, side * 59))
                .forward(30)
                .addMarker(() -> {
                    // Release foundation, drop block
                    logCurrent(drive.getPoseEstimate());
                    operateFoundation(true);
                    operateClaw(true);
                    return Unit.INSTANCE;
                })
                .build();
        traj5 = drive.trajectoryBuilder()
                .splineTo(new Pose2d(-36, side * 48, side * -90))
                .splineTo(new Pose2d(0, side * 48, 0))
                .splineTo(estCurrent)
                .addMarker(() -> {
                    // Lower Arm to block 2, Release Block
                    setArmPosition(1,2);
                    operateClaw(true);
                    levelArm();
                    return Unit.INSTANCE;
                })
                .build();
        traj6 = drive.trajectoryBuilder()
                .splineTo(new Pose2d(0, side * 48, 0))
                .build();
        return new Trajectory[]{traj1,traj2,traj3,traj4,traj5,traj6};
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
                    if(trackable == targetsSkyStone.get(0)){
                        double strafe = lastLocation.getTranslation().get(1) / mmPerInch;

                        if(Math.abs(strafe)<=2){
                           option = 'B';
                        } else if(strafe > 5) {
                            option = 'A';
                        }else if(strafe < 5) {
                            option = 'C';
                        }else{
                            option = 'X';
                        }
                    }else{
                        option = 'X';
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
