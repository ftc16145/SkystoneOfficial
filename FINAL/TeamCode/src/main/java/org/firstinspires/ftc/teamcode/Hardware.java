package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


import java.util.ArrayList;
/*
Fortnite!
* ⠀⠀⠀⠀⠀⠀⣤⣿⣿⠶⠀⠀⣀⣀

⠀⠀⠀⠀⠀⠀⣀⣀⣤⣤⣶⣿⣿⣿⣿⣿⣿

⠀⠀⣀⣶⣤⣤⠿⠶⠿⠿⠿⣿⣿⣿⣉⣿⣿

⠿⣉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠛⣤⣿⣿⣿⣀

⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⣿⣿⣿⣿⣶⣤

⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣤⣿⣿⣿⣿⠿⣛⣿

⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⣿⠛⣿⣿⣿⣿

⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣶⣿⣿⠿⠀⣿⣿⣿⠛

⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⣿⠀⠀⣿⣿⣿

⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠿⠿⣿⠀⠀⣿⣶

⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⠛⠀⠀⣿⣿⣶

⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⣿⣿⠤

⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠿⣿

⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿

⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣀

⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣶⣿
* */
public class Hardware {
    // Constants
        // Height of pivot above ground
            double hP = 12;
        // Min size of arm
            double dP = 7;
        // Radius of back part of arm
            double rB = 10;
        // Radius of spool wheel
            double rS = 7/16;
        // Radius of gear
            double rG = 3/4;


    // Timer
        private ElapsedTime runtime = new ElapsedTime();
    // Is this updating?
    // Scoring Mechanisms
        public DcMotorEx leftFront, leftBack, rightFront, rightBack;
        public ArrayList<DcMotorEx> drivetrain = new ArrayList<DcMotorEx>();
        public DcMotorEx slide = null;
        public DcMotorEx claw = null;
        public DcMotorEx arm = null;
        public DcMotorEx autoGrab = null;
        double prevXPower, prevYPower;
        public ArrayList<DcMotorEx> scoring = new ArrayList<DcMotorEx>();

        public CRServo found = null;
        public ColorSensor color = null;
        public DistanceSensor distance = null;
        public BNO055IMU imu = null;

        double targetAng = 0;

    // Info for the class
        HardwareMap hwMap = null;
        Telemetry tel = null;



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





            Context myApp;



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

    public void setDriveModes( DcMotorEx.RunMode r ){
        for( DcMotorEx m : drivetrain ){
            m.setMode(r);
        }
    }

    public void resetDriveMode( DcMotorEx.RunMode r ){
        setDriveModes( DcMotorEx.RunMode.STOP_AND_RESET_ENCODER );
        setDriveModes( r );
    }

    public void setScoreModes( DcMotorEx.RunMode r ){
        for( DcMotorEx m : scoring ){
            m.setMode( r );
        }
    }

    public void init( HardwareMap hardware, Telemetry atel ){
        hwMap = hardware;
        tel = atel;

        leftFront = hwMap.get(DcMotorEx.class, "leftFront");
        leftBack = hwMap.get(DcMotorEx.class, "leftBack");
        rightFront = hwMap.get(DcMotorEx.class, "rightFront");
        rightBack = hwMap.get(DcMotorEx.class, "rightBack");
        drivetrain.add(leftFront);
        drivetrain.add(leftBack);
        drivetrain.add(rightFront);
        drivetrain.add(rightBack);
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        for( DcMotorEx m : drivetrain ){
            m.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
        claw = hwMap.get(DcMotorEx.class, "claw");
        slide = hwMap.get(DcMotorEx.class, "slide");
        arm = hwMap.get(DcMotorEx.class, "arm");
        autoGrab = hwMap.get(DcMotorEx.class,"grab");
        scoring.add(claw);
        scoring.add(slide);
        scoring.add(arm);
        scoring.add(autoGrab);
        for( DcMotorEx m : scoring ){
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setTargetPosition(0);
            m.setPower(0);
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPositionPIDFCoefficients(10);
            m.setTargetPositionTolerance(5);
        }

        found = hwMap.get(CRServo.class, "foundation");

        color = hwMap.get(ColorSensor.class, "color");
        distance = hwMap.get(DistanceSensor.class, "color");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hwMap.get(BNO055IMU.class,"imu");
        imu.initialize(parameters);
        // Random Sound/Vision Things
        soundIndex = 0;
        soundID = -1;
        myApp = hwMap.appContext;
        params = new SoundPlayer.PlaySoundParams();
        params.loopControl = 0;
        params.waitForNonLoopingSoundsToFinish = true;
        playSound("ss_light_saber");
    }

    public void initLoop(){

    }

    public void setMotorPowers( double lf, double lb, double rf, double rb ){
        leftFront.setPower( lf );
        rightFront.setPower( rf );
        leftBack.setPower( lb );
        rightBack.setPower( rb );
    }

    public double yaw(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }

    public void hardBrake(){
        setMotorPowers(0,0,0,0);
    }

    public void mecanumDrive(double x, double y, double rot) {
        double nX, nY;
        // Updated from 0.1 to 0.2, testing to amp up accel
        nX = (Math.abs(x - prevXPower) > 0.2) ? prevXPower + Math.signum(x - prevXPower) * 0.2 : x;
        nY = (Math.abs(y - prevYPower) > 0.2) ? prevYPower + Math.signum(y - prevYPower) * 0.2 : y;

        double r = Math.hypot(-nX, nY);
        double robotAngle = Math.atan2(nY, -nX) - Math.PI / 4;
        double rightX = rot;
        double v1 = r * Math.cos(robotAngle) - rightX;
        double v2 = r * Math.sin(robotAngle) - rightX;
        double v3 = r * Math.sin(robotAngle) + rightX;
        double v4 = r * Math.cos(robotAngle) + rightX;
        double[] vals = {v1,v2,v3,v4};

        // Calculate maximum out of the 4 values
        double max = 0;
        for( double v : vals ){
            if( Math.abs( v ) > max ){
                max = Math.abs( v );
            }
        }

        // As long as the robot is in fact moving and is not just turning
        // Set all values to at most 1, mult by r to account for the magnitude input

        // Might need to make these deadbands instead of a hard zero
        if( max != 0 && r != 0 ){
            v1 = ( v1 / max ) * r;
            v2 = ( v2 / max ) * r;
            v3 = ( v3 / max ) * r;
            v4 = ( v4 / max ) * r;
        }
        setMotorPowers(v1,v2,v3,v4);
        // NOW: leftFront, leftBack, rightFront, rightBack
        prevXPower = nX;
        prevYPower = nY;

    }

    public enum Direction{
        kForward,kLeft,kRight,kReverse
    }

    public void driveDirection( Direction d, double inches, double power ){
        double encoder = inches/(4*Math.PI)*1120;
        switch( d ){
            case kForward:
                for( DcMotorEx m : drivetrain ){
                    m.setTargetPosition( (int) -encoder );
                }
                break;
            case kReverse:
                for( DcMotorEx m : drivetrain ){
                    m.setTargetPosition( (int) encoder );
                }
                break;
            case kLeft:
                leftFront.setTargetPosition( (int) encoder );
                leftBack.setTargetPosition( (int) -encoder );
                rightFront.setTargetPosition( (int) -encoder );
                rightBack.setTargetPosition( (int) encoder );
                break;
            case kRight:
                leftFront.setTargetPosition( (int) -encoder );
                leftBack.setTargetPosition( (int) encoder );
                rightFront.setTargetPosition( (int) encoder );
                rightBack.setTargetPosition( (int) -encoder );
                break;
        }
        if( isAtDriveTarget() ){
            hardBrake();
        }else {
            mecanumDrive(0, power, 0);
        }
    }

    public boolean isAtDriveTarget(){
        return Math.abs(leftFront.getCurrentPosition() - leftFront.getTargetPosition()) < 50;
    }

    public void rotateDrive( double ang, double power ){
        if( isAtRotateTarget() ) {
            hardBrake();
        }else{
            mecanumDrive(0, 0, (Math.toDegrees(yaw()) - ang) * -0.2 * power);
        }
    }

    public boolean isAtRotateTarget(){
        return Math.abs( Math.toDegrees( yaw() ) - targetAng ) <= 5;
    }

    public void mecanumDriveFieldOrient(double x, double y, double rot) {
        double adjustAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
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

    public void spoolControl( boolean up, boolean down ){
        if (up) {
            arm.setPower(-0.75);
        } else if (down) {
            arm.setPower(0.75);
        } else {
            arm.setPower(0);
        }
    }

    public void spoolControl( double power ){
        arm.setPower( power );
    }

    public void clawControl( boolean open, boolean close ){
        if (open) {
            clawControl(0.8);
        } else if (close) {
            clawControl(-0.8);
        } else {
            clawControl(0);
        }
    }

    public void clawControl( double power ){
        claw.setPower( power );
    }

    public void slideControl( boolean out, boolean in ){
        if (out) {
            slideControl(0.8);
        } else if (in) {
            slideControl(-0.8);
        } else {
            slideControl(0);
        }
    }

    public void slideControl( double power ){
        slide.setPower( power );
    }

    public void armMechanismControls(boolean clawOpen, boolean clawClose, boolean armUp, boolean armDown, double slideControl) {
        clawControl( clawOpen, clawClose );
        spoolControl( armUp, armDown );
        slideControl( slideControl );
    }

    public void setSideGrab( boolean open ){
        autoGrab.setTargetPosition( open ? 0 : 144 );
        autoGrab.setPower(1);
    }

    // NOT DONE, NEEDS VALUES, MAX
    public void setArmPosition( double x, double y ){
        double angle, rExpand;
        if( y == 0 && x == 0){
            angle = Math.atan( ( 5-hP ) / 19 );
            rExpand = (19 / Math.cos( angle ))-dP;
        }else{
            angle = Math.atan( ( 5*y + 2.25 - hP ) / ( 14.8 + 4*x ) );
            rExpand = ((14.8 + 4*x) / Math.cos( angle )) - dP;
        }
        double armWind = ((rB*Math.sin(-angle))/(2*Math.PI*rS))*288;
        double armExt = rExpand / (2*rG*Math.PI) * 288;
        // Fix ArmWInd
            slide.setTargetPosition((int) armExt);
            slide.setPower(0.8);
            arm.setTargetPosition((int) armWind);
            arm.setPower(0.8);

    }

    public void levelArm(){
        arm.setTargetPosition(0);
        arm.setPower(1);
        if(slide.getCurrentPosition()<630){
            slide.setTargetPosition(630);
            slide.setPower(0.5);
        }
    }

    public void operateClaw(boolean open){
        claw.setTargetPosition(open ? -240 : 0);
        claw.setPower(0.8);
    }





}
