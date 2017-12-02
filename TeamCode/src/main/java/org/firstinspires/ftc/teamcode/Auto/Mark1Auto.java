package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

import static com.sun.tools.doclint.HtmlTag.I;

/**
 * Created by AndrewC on 11/25/2017.
 */
@Autonomous(name = "Mark1Auto")
public class Mark1Auto extends LinearOpMode {

    //DO WITH ENCODERS
    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // AM Orbital 20 motor
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() throws InterruptedException
    {
        ElapsedTime timer = new ElapsedTime();
        Robot robot = new Robot(this, timer);

        robot.init();
        robot.jewel.retract();

        telemetry.addLine("Auto Mk1 start");
        telemetry.update();

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();


        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        robot.jewel.deploy();
        sleep(500);

        while(opModeIsActive()){
            int countForColor = 0;
            boolean isRed = robot.jewel.detectJewels1();
            sleep(500);
            telemetry.addData("Color", isRed);
            telemetry.update();

            if (isRed == false)
            {
                telemetry.addData("detected Blue", isRed);
                telemetry.update();
                driveBackward(1000, robot);
                robot.jewel.retract();
                driveForward(1000, robot);
            }

            driveForward(1000, robot);
            robot.jewel.retract();

            //stop moving
            robot.frontRight.setPower(0);
            robot.frontLeft.setPower(0);
            robot.rearRight.setPower(0);
            robot.rearLeft.setPower(0);
            break;

            //move forward - using encoders


            //then use PID control to go to pictograph and read it
        }



    }

    public void driveBackward(int timeLen, Robot robot)
    {
        robot.frontRight.setPower(-0.5);
        robot.frontLeft.setPower(-0.5);
        robot.rearRight.setPower(-0.5);
        robot.rearLeft.setPower(-0.5);
        sleep(timeLen);
    }

    public void driveForward(int timeLen, Robot robot)
    {
        robot.frontRight.setPower(0.5);
        robot.frontLeft.setPower(-0.5);
        robot.rearRight.setPower(-0.5);
        robot.rearLeft.setPower(0.5);
        sleep(timeLen);
    }

    /*public void driveForwardEncoder(double speed,
                                    double leftInches, double rightInches,
                                    double timeoutS, Robot robot)
    {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            robot.frontLeft(robot.frontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH));
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
    }*/

    private double[] calcMotorPowers(double leftStickX, double leftStickY, double rightStickX)
    {
        //probably a better way to do this
        double r = Math.hypot(leftStickX, leftStickY);
        double robotAngle = Math.atan2(leftStickY, leftStickX) - Math.PI / 4;
        double lrPower = r * Math.sin(robotAngle) + rightStickX;
        double lfPower = r * Math.cos(robotAngle) + rightStickX;
        double rrPower = r * Math.cos(robotAngle) - rightStickX;
        double rfPower = r * Math.sin(robotAngle) - rightStickX;
        return new double[]{lrPower, lfPower, rrPower, rfPower};
    }
}
