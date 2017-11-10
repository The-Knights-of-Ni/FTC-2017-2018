package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Cat on 11/10/17.
 */

public class Glyft {

    public Servo squeezerLeft;
    public Servo squeezerRight;
    public DcMotor glyftMotor1;
    public DcMotor glyftMotor2;
    public ElapsedTime OpModeTime;

    public enum GlyftState {
        STOPPED, SQUEEZED, LIFTING, LIFTED;
    }

    public GlyftState glyftState = STOPPED;

    public Glyft(Servo squeezerLeft, Servo squeezerRight, DcMotor glyftMotor1, DcMotor glyftMotor2, ElapsedTime OpModeTime) {
        this.squeezerLeft = squeezerLeft;
        this.squeezerRight = squeezerRight;
        this.glyftMotor1 = glyftMotor1;
        this.glyftMotor2 = glyftMotor2;
        this.OpModeTime = OpModeTime;
    }

    // TO DO: specific functions + positions
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

    public void glyphLift() {
        glyftMotor1.setPower(0.5); // need to test
        glyftMotor2.setPower(0.5);
    }

}