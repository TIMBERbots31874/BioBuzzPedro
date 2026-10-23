package org.firstinspires.ftc.teamcode.Drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TestMotion extends LinearOpMode {

    Motion drive;

    @Override
    public void runOpMode()  {
        drive = new Motion(this);

        waitForStart();

        while (opModeIsActive()){
            if(gamepad1.a) drive.setMotorPowers(0.2, 0,0,0);
            else if(gamepad1.x) drive.setMotorPowers(0, 0.2,0,0);
            else if(gamepad1.y) drive.setMotorPowers(0, 0,0.2,0);
            else if(gamepad1.b) drive.setMotorPowers(0, 0,0,0.2);
        }

    }
}
