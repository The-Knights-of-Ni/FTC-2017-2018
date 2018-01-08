package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.subsystems.Glyft.GlyftState.GRABBING;
import static org.firstinspires.ftc.teamcode.subsystems.Glyft.GlyftState.INTAKING;
import static org.firstinspires.ftc.teamcode.subsystems.Glyft.GlyftState.LIFTING;
import static org.firstinspires.ftc.teamcode.subsystems.Glyft.GlyftState.MANUAL;
import static org.firstinspires.ftc.teamcode.subsystems.Glyft.GlyftState.SCORING;

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

        double stateEnterTime = 0;
        boolean isFirstLoopInState = true;
        boolean isFirstGlyph = true;

        while (opModeIsActive()) {
            double leftStickX = gamepad1.left_stick_x;
            double leftStickY = -gamepad1.left_stick_y;
            double rightStickX = gamepad1.right_stick_x;
            double[] motorPowers = calcMotorPowers(leftStickX, leftStickY, rightStickX);
            robot.drive.rearLeft.setPower(motorPowers[0]);
            robot.drive.frontLeft.setPower(motorPowers[1]);
            robot.drive.rearRight.setPower(motorPowers[2]);
            robot.drive.frontRight.setPower(motorPowers[3]);

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

            /*
            if (gamepad1.left_trigger > 0.5) { //Intake Open
                robot.glyft.setGlyftState(Glyft.GlyftState.LIFTING);
                compliantWheelPower = 0;
                /*robot.intakePivotLeft.setPosition(0.20);
                robot.intakePivotRight.setPosition(0.60);
                robot.compliantWheelLeft.setPower(compliantWheelPower);
                robot.compliantWheelRight.setPower(compliantWheelPower);
            } else if (gamepad1.right_trigger > 0.5) { // Intake Closed
                robot.glyft.setGlyftState(Glyft.GlyftState.INTAKING);
                compliantWheelPower = 0.9;
                /*robot.intakePivotLeft.setPosition(0.005);
                robot.intakePivotRight.setPosition(0.85/*0.70);
                robot.compliantWheelLeft.setPower(compliantWheelPower);
                robot.compliantWheelRight.setPower(compliantWheelPower);
            } else if (gamepad1.a) { // Scoring Storage
                robot.glyft.setGlyftState(Glyft.GlyftState.SCORING);
                compliantWheelPower = 0;
                /*robot.intakePivotLeft.setPosition(0.45);
                robot.intakePivotRight.setPosition(0.10/*0.15);
                robot.compliantWheelLeft.setPower(compliantWheelPower);
                robot.compliantWheelRight.setPower(compliantWheelPower);
            } else if (gamepad1.b) {
                robot.glyft.setGlyftState(STACKING);
            }
            */

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
                case MANUAL:
                    //Glyft linear slide control
                    double glyftMotorPower;
                    if (gamepad2.a) {
                        glyftMotorPower = 0.40;
                    } else if (gamepad2.b) {
                        glyftMotorPower = 0.50;
                    } else if (gamepad2.x) {
                        glyftMotorPower = 0.60;
                    } else if (gamepad2.y) {
                        glyftMotorPower = 0.70;
                    } else if (gamepad2.right_stick_button) {
                        glyftMotorPower = 1.00;
                    } else {
                        glyftMotorPower = 0.00;
                    }

                    if (gamepad2.left_trigger > 0.5) {
                        glyftMotorPower = -1.0;
                    }
                    robot.glyft.setPower(glyftMotorPower);

                    //Vertical squeezer control
                    if (gamepad1.left_bumper) { //Open
                        robot.glyft.squeezerLeft.setPosition(0.35);
                        robot.glyft.squeezerRight.setPosition(0.40);
                    } else if (gamepad1.right_bumper) { //Closed
                        robot.glyft.squeezerLeft.setPosition(0.15);
                        robot.glyft.squeezerRight.setPosition(0.55);
                    }

                    if (gamepad1.right_trigger > 0.5) {
                        robot.glyft.setGlyftState(INTAKING);
                        isFirstLoopInState = true;
                        isFirstGlyph = true;
                    }
                    break;
                case INTAKING:
                    //move glyft to the right place
                    robot.glyft.closeCompliantWheels();
                    robot.glyft.setCWPower(0.9);

                    double power = 0.0;
                    if (gamepad2.a) {
                        power = 0.40;
                    } else if (gamepad2.b) {
                        power = 0.50;
                    } else if (gamepad2.x) {
                        power = 0.60;
                    } else if (gamepad2.y) {
                        power = 0.70;
                    } else if (gamepad2.right_stick_button) {
                        power = 1.00;
                    } else {
                        power = 0.00;
                    }

                    if (gamepad2.left_trigger > 0.5) {
                        power = -1.0;
                    }
                    robot.glyft.setPower(power);

                    if (gamepad1.right_bumper) {
                        robot.glyft.setGlyftState(GRABBING);
                        isFirstLoopInState = true;
                    }

                    if (gamepad1.y) {
                        robot.glyft.setCWLeftPower(0.9);
                        robot.glyft.setCWRightPower(-0.8);
                    }
                    if (gamepad1.a) {
                        isFirstLoopInState = true;
                        robot.glyft.setGlyftState(SCORING);
                    }
                    break;
                case LIFTING:
                    if (isFirstLoopInState) {
                        stateEnterTime = timer.seconds();
                        robot.glyft.moveGlyft(1700);
                        //robot.glyft.setPower(0.5);
                        isFirstLoopInState = false;
                    }
                    if (Math.abs(robot.glyft.glyftMotor1.getCurrentPosition()-1514) > 100/*timer.seconds() - stateEnterTime < 1.5/*robot.glyft.glyftMotor1.isBusy() && robot.glyft.glyftMotor2.isBusy()*/) {
                        //double elapsedTime = timer.seconds() - stateEnterTime;
                        //double glyftPower = (elapsedTime/0.5) * 0.75;
                        robot.glyft.setPower(0.75);
                        break;
                    }
                    robot.glyft.setPower(0);
                    robot.glyft.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    isFirstGlyph = false;
                    isFirstLoopInState = true;
                    robot.glyft.setGlyftState(INTAKING);
                    break;
                case GRABBING:
                    if (isFirstLoopInState) {
                        robot.glyft.setCWPower(0);
                        robot.glyft.openCompliantWheels();
                        sleep(250);
                        robot.glyft.openSqueezers();
                        robot.glyft.moveGlyft(0);
                        isFirstLoopInState = false;
                    }
                    if (robot.glyft.glyftMotor1.isBusy() && robot.glyft.glyftMotor2.isBusy()) {
                        break;
                    }
                    robot.glyft.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.glyft.closeSqueezers();
                    isFirstLoopInState = true;
                    if (isFirstGlyph) {
                        robot.glyft.setGlyftState(LIFTING);
                    } else {
                        robot.glyft.setGlyftState(SCORING);
                    }
                    /*
                    robot.glyft.openCompliantWheels();
                    robot.glyft.setCWPower(0);
                    robot.glyft.moveGlyft(-146);
                    robot.glyft.openSqueezers();
                    robot.glyft.moveGlyft(-1746);
                    robot.glyft.closeSqueezers();
                    robot.glyft.setGlyftState(Glyft.GlyftState.SCORING);
                    */
                    break;
                case SCORING:
                    robot.glyft.storeCompliantWheels();
                    robot.glyft.setCWPower(0);
                    isFirstLoopInState = true;
                    robot.glyft.setGlyftState(MANUAL);
                    break;
            }

            telemetry.addData("Servo position L", leftServoPosition);
            telemetry.addData("Servo position R", rightServoPosition);
            telemetry.addData("Servo position claw", clawServoPosition);
            telemetry.addData("Glyft Motor 1", robot.glyft.glyftMotor1.getCurrentPosition());
            telemetry.addData("Glyft Motor 2", robot.glyft.glyftMotor2.getCurrentPosition());
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
        lrPower *= 0.8;
        lfPower *= 0.8;
        rrPower *= 0.8;
        rfPower *= 0.8;
        return new double[]{lrPower, lfPower, rrPower, rfPower};
    }
}
