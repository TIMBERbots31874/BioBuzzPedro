package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    //in = intake motor
    private DcMotorEx intake1;
    private DcMotorEx intake2;

    public enum IntakeState{
        FORWARD, REVERSE, STOPPED
    }

//    private double intakeS;

    private IntakeState currentState = IntakeState.STOPPED;

    public void init(HardwareMap hwMap) {
        intake1 = hwMap.get(DcMotorEx.class, "IN1");
        intake1.setDirection(DcMotorEx.Direction.REVERSE);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake2 = hwMap.get(DcMotorEx.class, "IN2");
        intake2.setDirection(DcMotorEx.Direction.REVERSE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setReadyState(IntakeState.STOPPED);
    }

    public void setReadyState(IntakeState newState) {
        currentState = newState;

        switch (currentState) {
            case FORWARD:
                intake2.setPower(1.0);
                intake1.setPower(1.0);
                break;
            case REVERSE:
                intake2.setPower(-1.0);
                intake1.setPower(-1.0);
                break;
            case STOPPED:
                intake2.setPower(0.0);
                intake1.setPower(0.0);

        }
    }

    public IntakeState getCurrentState() {
        return currentState;
    }


//        if (intakeS = 1.0){
//            intake.setDirection(DcMotorEx.Direction.FORWARD);
//        }else (intakeS = 0.0){
//            intake.setPower(DcMotor.ZeroPowerBehavior.BRAKE);
//        }



}
