package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.biobuzz.Intake;

@TeleOp
public class RunIntake extends LinearOpMode {

    private Intake  intake = new Intake();

    public void runOpMode(){
        intake.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()){
            boolean lbWasPressed = gamepad1.leftBumperWasPressed();
            boolean rbWasPressed = gamepad1.rightBumperWasPressed();

            switch (intake.getCurrentState()){
                case FORWARD:
                    if (lbWasPressed) intake.setReadyState(Intake.IntakeState.REVERSE);
                    else if (rbWasPressed) intake.setReadyState(Intake.IntakeState.STOPPED);
                    break;
                case STOPPED:
                    if (lbWasPressed) intake.setReadyState(Intake.IntakeState.REVERSE);
                    else if (rbWasPressed) intake.setReadyState(Intake.IntakeState.FORWARD);
                    break;
                case REVERSE:
                    if (lbWasPressed) intake.setReadyState(Intake.IntakeState.STOPPED);
                    else if (rbWasPressed) intake.setReadyState(Intake.IntakeState.FORWARD);
            }
        }
    }
}