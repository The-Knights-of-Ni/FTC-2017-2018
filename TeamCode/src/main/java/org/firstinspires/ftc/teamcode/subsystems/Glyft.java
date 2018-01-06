package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

import static com.qualcomm.robotcore.hardware.DcMotor.*;
import static com.sun.tools.doclint.HtmlTag.P;
import static org.firstinspires.ftc.teamcode.subsystems.Glyft.GlyftState.STOPPED;

/**
 * Created by Cat on 11/10/17.
 */

public class Glyft {

    private static final double VS_LEFT_OPEN_POSITION = 0.35;
    private static final double VS_RIGHT_OPEN_POSITION = 0.40;
    private static final double VS_LEFT_CLOSED_POSITION = 0.15;
    private static final double VS_RIGHT_CLOSED_POSITION = 0.55;

    private static final double CW_LEFT_OPEN_POSITION = 0.20;
    private static final double CW_RIGHT_OPEN_POSITION = 0.60;
    private static final double CW_LEFT_INTAKING_POSITION = 0;
    private static final double CW_RIGHT_INTAKING_POSITION = 0.9;
    private static final double CW_LEFT_STORED_POSITION = 0.45;
    private static final double CW_RIGHT_STORED_POSITION = 0.10;

    public Servo squeezerLeft;
    public Servo squeezerRight;
    public DcMotor glyftMotor1;
    public DcMotor glyftMotor2;
    public ServoImplEx intakePivotLeft;
    public ServoImplEx intakePivotRight;
    public CRServo compliantWheelLeft;
    public CRServo compliantWheelRight;
    public ElapsedTime OpModeTime;

    public enum GlyftState {
        INTAKING, LIFTING, STACKING, SCORING, STOPPED
    }

    public GlyftState glyftState = STOPPED;

    public Glyft(Servo squeezerLeft, Servo squeezerRight, DcMotor glyftMotor1, DcMotor glyftMotor2, ServoImplEx intakePivotLeft, ServoImplEx intakePivotRight, CRServo compliantWheelLeft, CRServo compliantWheelRight, ElapsedTime OpModeTime) {
        this.squeezerLeft = squeezerLeft;
        this.squeezerRight = squeezerRight;
        this.glyftMotor1 = glyftMotor1;
        this.glyftMotor2 = glyftMotor2;
        this.intakePivotLeft = intakePivotLeft;
        this.intakePivotRight = intakePivotRight;
        this.compliantWheelLeft = compliantWheelLeft;
        this.compliantWheelRight = compliantWheelRight;
        this.OpModeTime = OpModeTime;
    }

    // TO DO: specific functions + positions + test
    public GlyftState getGlyftState() {
        return glyftState;
    }

    public void setGlyftState(GlyftState state) {
        glyftState = state;
    }

    public void rotateSqueezers() {
        squeezerLeft.setPosition(1); // self-reminder: servo goes from 0 to 1, 0.5 is 45 deg turn
        squeezerRight.setPosition(1);
    }

    public void openSqueezers() {
        squeezerLeft.setPosition(VS_LEFT_OPEN_POSITION );
        squeezerRight.setPosition(VS_RIGHT_OPEN_POSITION);
    }

    public void closeSqueezers() {
        squeezerLeft.setPosition(VS_LEFT_CLOSED_POSITION);
        squeezerRight.setPosition(VS_RIGHT_CLOSED_POSITION);
    }

    public void openCompliantWheels() {
        intakePivotLeft.setPosition(CW_LEFT_OPEN_POSITION);
        intakePivotRight.setPosition(CW_RIGHT_OPEN_POSITION);
    }

    public void closeCompliantWheels() {
        intakePivotLeft.setPosition(CW_LEFT_INTAKING_POSITION);
        intakePivotRight.setPosition(CW_RIGHT_INTAKING_POSITION);
    }

    public void storeCompliantWheels() {
        intakePivotLeft.setPosition(CW_LEFT_STORED_POSITION);
        intakePivotRight.setPosition(CW_RIGHT_STORED_POSITION);
    }

    public void setCWPower (double power)
    {
        compliantWheelLeft.setPower(power);
        compliantWheelRight.setPower(power);
    }

    public void moveGlyft (int targetPosition)
    {
        glyftMotor1.setMode(RunMode.STOP_AND_RESET_ENCODER);
        glyftMotor2.setMode(RunMode.STOP_AND_RESET_ENCODER);
        glyftMotor1.setMode(RunMode.RUN_TO_POSITION);
        glyftMotor2.setMode(RunMode.RUN_TO_POSITION);
        glyftMotor1.setTargetPosition(targetPosition);
        glyftMotor2.setTargetPosition(targetPosition);
        glyftMotor1.setPower(0.60);
        glyftMotor2.setPower(0.60);
        while (glyftMotor1.isBusy() && glyftMotor2.isBusy())
        {
            //do nothing
        }
    }

    public void setPower(double power) {
        glyftMotor1.setPower(power); // need to test
        glyftMotor2.setPower(power);
    }

}