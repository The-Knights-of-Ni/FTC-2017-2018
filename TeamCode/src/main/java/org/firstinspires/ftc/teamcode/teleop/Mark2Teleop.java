package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Glyft;

import static com.sun.tools.doclint.HtmlTag.P;

     /**
 * Created by Shiva787 on 1/4/18.
 */

@TeleOp(name = "Mark 2 Teleop")
public class Mark2Teleop extends LinearOpMode{
    @Override
    public void runOpMode() {
        ElapsedTime timer = new ElapsedTime();
        Robot robot = new Robot(this, timer);
        telemetry.addLine("Initialization finished. Waiting for start...");
        telemetry.update();
        waitForStart();
        boolean lbWasPressedLastLoop = false;
        boolean rbWasPressedLastLoop = false;
        boolean upWasPressedLastLoop = false;
        boolean downWasPressedLastLoop = false;
        boolean leftWasPressedLastLoop = false;
        boolean rightWasPressedLastLoop = false;
        boolean areStacking = false;
        double leftServoPosition = 0.5;
        double rightServoPosition = 0.5;
        double clawServoPosition = 0.5;
        double compliantWheelPower = 0;

        double lastIncreaseTime;

        while (opModeIsActive()) {
            double leftStickX = gamepad1.left_stick_x;
            double leftStickY = -gamepad1.left_stick_y;
            double rightStickX = gamepad1.right_stick_x;
            double[] motorPowers = calcMotorPowers(leftStickX, leftStickY, rightStickX);
            robot.drive.rearLeft.setPower(motorPowers[0]);
            robot.drive.frontLeft.setPower(motorPowers[1]);
            robot.drive.rearRight.setPower(motorPowers[2]);
            robot.drive.frontRight.setPower(motorPowers[3]);

            //robot.glyft.setPower(-gamepad2.right_stick_y);
            double motorPower;
            if (gamepad2.a) {
                motorPower = 0.30;
            } else if (gamepad2.b) {
                motorPower = 0.40;
            } else if (gamepad2.x) {
                motorPower = 0.50;
            } else if (gamepad2.y) {
                motorPower = 0.60;
            } else if (gamepad2.right_stick_button) {
                motorPower = 1.00;
            } else {
                motorPower = 0.00;
            }

            if (gamepad2.left_trigger > 0.5) {
                motorPower = -0.40;
            }

            robot.glyft.setPower(motorPower);

            if (gamepad1.left_bumper) { //Open
                robot.glyft.squeezerLeft.setPosition(0.35);
                robot.glyft.squeezerRight.setPosition(0.40);
                //robot.glyft.squeezerLeft.setPosition(0.35);
                //robot.glyft.squeezerRight.setPosition(0.4);
            } else if (gamepad1.right_bumper) { //Closed
                robot.glyft.squeezerLeft.setPosition(0.15);
                robot.glyft.squeezerRight.setPosition(0.55);
            }

            if (gamepad2.right_bumper) {
                robot.relicRecovery.wrist.setPosition(0);
            } else if (gamepad2.left_bumper) {
                robot.relicRecovery.wrist.setPosition(0.65);
            }

            /*
            if (gamepad1.dpad_left) {
                if (!leftWasPressedLastLoop) {
                    clawServoPosition -= 0.05;
                    leftWasPressedLastLoop = true;
                }
            } else {
                leftWasPressedLastLoop = false;
            }

            if (gamepad1.dpad_right) {
                if (!rightWasPressedLastLoop) {
                    clawServoPosition += 0.05;
                    rightWasPressedLastLoop = true;
                }
            } else {
                rightWasPressedLastLoop = false;
            }
            clawServoPosition = Range.clip(clawServoPosition, 0.0, 1.0);
            */

            if (gamepad1.left_trigger > 0.5) { //Intake Open
                robot.glyft.setGlyftState(Glyft.GlyftState.LIFTING);
                compliantWheelPower = 0;
                /*robot.intakePivotLeft.setPosition(0.20);
                robot.intakePivotRight.setPosition(0.60);
                robot.compliantWheelLeft.setPower(compliantWheelPower);
                robot.compliantWheelRight.setPower(compliantWheelPower);*/
            } else if (gamepad1.right_trigger > 0.5) { // Intake Closed
                robot.glyft.setGlyftState(Glyft.GlyftState.INTAKING);
                compliantWheelPower = 0.9;
                /*robot.intakePivotLeft.setPosition(0.005);
                robot.intakePivotRight.setPosition(0.85/*0.70);
                robot.compliantWheelLeft.setPower(compliantWheelPower);
                robot.compliantWheelRight.setPower(compliantWheelPower);*/
            } else if (gamepad1.a) { // Scoring Storage
                robot.glyft.setGlyftState(Glyft.GlyftState.SCORING);
                compliantWheelPower = 0;
                /*robot.intakePivotLeft.setPosition(0.45);
                robot.intakePivotRight.setPosition(0.10/*0.15);
                robot.compliantWheelLeft.setPower(compliantWheelPower);
                robot.compliantWheelRight.setPower(compliantWheelPower);*/
            } else if (gamepad1.b) {
                robot.glyft.setGlyftState(Glyft.GlyftState.STACKING);
            }

            if (gamepad1.y)
            {
                robot.compliantWheelRight.setPower(-0.8);
            }
            else
            {
                robot.compliantWheelRight.setPower(compliantWheelPower);
            }

            if (gamepad2.right_trigger > 0.5) {
                /*robot.compliantWheelLeft.setPower(0.9);
                robot.compliantWheelRight.setPower(0.9);*/
            } else {
                /*robot.compliantWheelLeft.setPower(0);
                robot.compliantWheelRight.setPower(0);*/
            }

            if (gamepad2.dpad_up) {
                robot.intakePivotRight.setPwmDisable();
            } else {
                robot.intakePivotRight.setPwmEnable();
            }

            if (gamepad1.x)
            {
                areStacking = true;
            }

            //robot.intakePivotLeft.setPosition(clawServoPosition);
            //robot.intakePivotRight.setPosition(rightServoPosition);

            //robot.glyft.squeezerLeft.setPosition(leftServoPosition);
            //robot.glyft.squeezerRight.setPosition(rightServoPosition);
            robot.relicRecovery.claw.setPosition(clawServoPosition);
            //robot.relicRecovery.wrist.setPosition(rightServoPosition);
            robot.relicRecovery.relicMotor.setPower(Math.pow(gamepad2.left_stick_y, 3.0));

            telemetry.addData("Glyft State:", robot.glyft.getGlyftState());
            telemetry.addData("Are stacking:", areStacking);
            //Glyft State Machine
            switch(robot.glyft.getGlyftState()) {
                case STOPPED:
                    break;
                case INTAKING:
                    if (!areStacking)
                    {
                        robot.glyft.openSqueezers();
                    }
                    //move glyft to the right place
                    robot.glyft.closeCompliantWheels();
                    robot.glyft.setCWPower(0.9);
                    break;
                case LIFTING:
                    robot.glyft.closeSqueezers();
                    robot.glyft.openCompliantWheels();
                    robot.glyft.setCWPower(0);
                    robot.glyft.moveGlyft(1892);
                    robot.glyft.closeCompliantWheels();
                    robot.glyft.setCWPower(0.9);
                    robot.glyft.setGlyftState(Glyft.GlyftState.INTAKING);
                    break;
                case STACKING:
                    robot.glyft.openCompliantWheels();
                    robot.glyft.setCWPower(0);
                    robot.glyft.moveGlyft(-146);
                    robot.glyft.openSqueezers();
                    robot.glyft.moveGlyft(-1746);
                    robot.glyft.closeSqueezers();
                    robot.glyft.setGlyftState(Glyft.GlyftState.SCORING);
                    break;
                case SCORING:
                    robot.glyft.storeCompliantWheels();
                    robot.glyft.setCWPower(0);
                    areStacking = false;
                    break;
            }

            telemetry.addData("Servo position L", leftServoPosition);
            telemetry.addData("Servo position R", rightServoPosition);
            telemetry.addData("Servo position claw", clawServoPosition);
            telemetry.update();
        }
    }

    private double[] calcMotorPowers(double leftStickX, double leftStickY, double rightStickX) {
        double r = Math.hypot(leftStickX, leftStickY);
        double robotAngle = Math.atan2(leftStickY, leftStickX) - Math.PI / 4;
        double lrPower = r * Math.sin(robotAngle) + rightStickX;
        double lfPower = r * Math.cos(robotAngle) + rightStickX;
        double rrPower = r * Math.cos(robotAngle) - rightStickX;
        double rfPower = r * Math.sin(robotAngle) - rightStickX;
        lrPower *= 0.6;
        lfPower *= 0.6;
        rrPower *= 0.6;
        rfPower *= 0.6;
        return new double[]{lrPower, lfPower, rrPower, rfPower};
    }
}
