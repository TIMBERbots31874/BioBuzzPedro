package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.biobuzz.Shooter;

@TeleOp
public class TestShooter extends LinearOpMode {

    private Shooter sh = new Shooter();

    public void runOpMode() {
        sh.init(hardwareMap);

        waitForStart();

        sh.setPower(0.5);

        while (opModeIsActive()){
            telemetry.addData("Speed", sh.shm.getVelocity());
            telemetry.update();
        }

    }
}
