package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class intake {

    //in = intake motor
    private DcMotorEx intake;

    private enum IntakeState{
        FORWARD, REVERSE, STOPPED
    }

//    private double intakeS;

    private IntakeState currentState = IntakeState.STOPPED;

    public void init(HardwareMap hwMap){
        intake = hwMap.get(DcMotorEx.class, "Intake");
        intake.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setReadyState(IntakeState.STOPPED);

        public void setReadyState(IntakeState newState){
            currentState = newState;

            switch (currentState){
                case FORWARD:
                    intake.setPower(1.0);
                    break;
                case REVERSE:
                    intake.setPower(-1.0);
                    break;
                case STOPPED:
                    intake.setPower(0.0);
                    break;

            }
        }

        public IntakeState getCurrentState(){
            return currentState;
        }


//        if (intakeS = 1.0){
//            intake.setDirection(DcMotorEx.Direction.FORWARD);
//        }else (intakeS = 0.0){
//            intake.setPower(DcMotor.ZeroPowerBehavior.BRAKE);
//        }

    }

}
