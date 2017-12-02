package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * Created by AndrewC on 11/25/2017.
 */
@Autonomous(name = "Mark1Auto")
public class Mark1Auto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        Robot robot = new Robot(this, timer);
        robot.init();
        robot.jewel.retract();

        waitForStart();
        robot.jewel.deploy();
        sleep(500);
        while(opModeIsActive()){
            boolean isRed = robot.jewel.detectJewels1();
            telemetry.addData("Color", isRed);
            telemetry.update();
        }



    }
    public void readJewel(){

    }


}
