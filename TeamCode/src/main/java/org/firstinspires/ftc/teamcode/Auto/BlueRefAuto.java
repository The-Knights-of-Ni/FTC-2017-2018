package org.firstinspires.ftc.teamcode.Auto;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Robot;

/**
 * Created by AndrewC on 11/25/2017.
 */
@Autonomous(name = "Blue Referee Mk1 Auto")
public class BlueRefAuto extends LinearOpMode {
    private static final String TAG = "BlueRefMk1Auto";

    private Robot robot;
    private ElapsedTime timer;

    private VuforiaLocalizer vuforia;
    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable relicTemplate;

    //DO WITH ENCODERS
    private static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // AM Orbital 20 motor
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final double     DRIVE_SPEED             = 0.6;
    private static final double     TURN_SPEED              = 0.5;

    //Timing Constants
    private static final int PICTOGRAPH_TIMEOUT = 5000;
    private static final int JEWEL_DEPLOY_WAIT = 1500;
    private static final double GLYFT_LIFT_TIME = 0.40;

    //Encoder Constants

    @Override
    public void runOpMode() throws InterruptedException {
        timer = new ElapsedTime();
        double startTime = 0.0;
        robot = new Robot(this, timer);

        initRobot();
        waitForStart();

        log("Started Mark 1 Auto");

        //Read pictograph
        RelicRecoveryVuMark vuMark = null;
        relicTrackables.activate();
        startTime = timer.seconds();
        while (opModeIsActive() && (timer.seconds() - startTime) < PICTOGRAPH_TIMEOUT) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                break;
            }
        }
        relicTrackables.deactivate();
        log("Finished reading pictograph: " + vuMark);

        //Grab glyft and raise lift
        robot.glyft.closeSqueezers();
        sleep(500);
        startTime = timer.seconds();
        while (opModeIsActive() && (timer.seconds() - startTime) < GLYFT_LIFT_TIME) {
            robot.glyft.setPower(0.50);
        }
        robot.glyft.setPower(0);
        sleep(500);

        //Deploy jewel arm and read jewel color
        robot.jewel.deploy();
        sleep(JEWEL_DEPLOY_WAIT);
        boolean jewelIsRed = robot.jewel.detectJewels1();
        log("Jewel Detected: " + (jewelIsRed ? "RED | BLUE" : "BLUE | RED"));
        sleep(500);

        //Knock off correct jewel
        startTime = timer.seconds();
        int targetPosition = 0;
        robot.drive.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drive.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (jewelIsRed) {
            targetPosition = 80;
        } else {
            targetPosition = -900;
        }
        robot.drive.setTargetPosition(targetPosition);
        robot.drive.setPower(0.10);
        while (opModeIsActive() && robot.drive.frontLeft.isBusy() && robot.drive.frontRight.isBusy()) {

        }
        robot.drive.stop();
        robot.jewel.retract();
        sleep(500);


        //If backwards, drive to pictograph reading position
        if (jewelIsRed) {
            robot.drive.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.drive.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.drive.setTargetPosition(-980);
            robot.drive.setPower(0.10);
            while (opModeIsActive() && robot.drive.frontLeft.isBusy() && robot.drive.frontRight.isBusy()) {

            }
            robot.drive.stop();
            sleep(500);
        }

        robot.drive.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        startTime = timer.seconds();
        while (opModeIsActive() && timer.seconds() - startTime < 0.25) {
            robot.drive.setPower(-0.20);
        }
        robot.drive.stop();

        //Turn 180 degrees to face cryptobox
        robot.drive.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (opModeIsActive() && Math.abs(robot.drive.getYaw() - 180) > 2) {
            robot.drive.turn(0.10);
        }
        robot.drive.stop();
        sleep(500);

        //Drive to correct column
        switch (vuMark) {
            case LEFT:
                targetPosition = 250;
                break;
            case CENTER:
                targetPosition = 610;
                break;
            case RIGHT:
                targetPosition = 970;
                break;
            default:
                targetPosition = 610;
        }
        robot.drive.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drive.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.drive.strafe(targetPosition);
        robot.drive.setPower(0.25);
        while (opModeIsActive() && robot.drive.frontLeft.isBusy() && robot.drive.rearLeft.isBusy()) {

        }
        robot.drive.stop();
        sleep(500);

        //Drive forward to approach cryptobox
        robot.glyft.openSqueezers();
        sleep(500);
        robot.drive.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drive.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        startTime = timer.seconds();
        while (opModeIsActive() && (timer.seconds() - startTime) < 2.5) {
            robot.drive.setPower(0.10);
        }
        robot.drive.stop();
        sleep(500);

        startTime = timer.seconds();
        while (opModeIsActive() && (timer.seconds() - startTime) < 0.5) {
            robot.drive.setPower(-0.10);
        }
        robot.drive.stop();

        robot.glyft.moveGlyft(0);
        while (robot.glyft.glyftMotor1.isBusy() && robot.glyft.glyftMotor2.isBusy()) {

        }
    }

    private void initRobot() {
        robot.init();
        robot.jewel.retract();
        robot.drive.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drive.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Vuforia initialization
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AX1T/JH/////AAAAGWQh21MbJEIhpE//dkoSovcwBwhbe6+121U+fGQaCJZI0cDQka2Bqcnc1N9dRlzyr5ZwjGPLUqxXId7+l/yUFBV1v66pF5nuD5JJOr9IVM22ZUxMSQesMrpCqfzGowHAv/dTDZmuqOxfqazZ6xeJ5V/V/2HdwGFDCrTXbZd4PzSwaOQed48I7XtIvu2m3nEJAb+aAC6DT78HHLRIFStmgfS4QglTEy+M7JOtDkc5u5k5CQhk9hwNsea4nDqfVf9XJjKLJJFhTat0IdiPz8BIrsNWxP8S7EiZLaWdanHJIOdP2NhokmI0jkLgPuRLkC7BvorDDeVI+pdutDMjN9kf/b11uGyrf6fJ4AySTe1+R9m/";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);

        telemetry.addLine("Finished Initialization. Waiting for start.");
        telemetry.update();
        Log.d(TAG, "Finished Initialization. Waiting for start.");
    }

    private void log(String message) {
        telemetry.addLine(message);
        telemetry.update();
        Log.d(TAG, message);
    }
}
