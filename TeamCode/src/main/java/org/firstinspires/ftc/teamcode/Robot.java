package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.*;
/**
 * Created by AndrewC on 11/25/2017.
 */

public class Robot {
    public String name;
    public HardwareMap hardwareMap;
    public ElapsedTime timer;

    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx rearRight;
    public DcMotorEx rearLeft;

    public Servo jewelServo;

    public ColorSensor colorSensor;

    public Jewel jewel;

    public Robot(OpMode opMode, ElapsedTime timer){
        hardwareMap = opMode.hardwareMap;
        this.timer = timer;
    }
    public void init(){
        frontRight = (DcMotorEx) hardwareMap.dcMotor.get("FR");
        frontLeft = (DcMotorEx) hardwareMap.dcMotor.get("FL");
        rearRight = (DcMotorEx) hardwareMap.dcMotor.get("RR");
        rearLeft = (DcMotorEx) hardwareMap.dcMotor.get("RL");

        jewelServo = hardwareMap.servo.get("JS");
        colorSensor = hardwareMap.colorSensor.get("CS");
        jewel = new Jewel(jewelServo, colorSensor, timer);


    }
}

