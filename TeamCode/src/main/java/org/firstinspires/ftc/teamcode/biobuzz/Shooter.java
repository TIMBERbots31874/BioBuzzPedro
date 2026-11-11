package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Shooter {
    public DcMotorEx shm;

    public PIDFCoefficients pidf = new PIDFCoefficients(0, 0,0, 11);

    public void init(HardwareMap hwMap){
        shm = hwMap.get(DcMotorEx.class, "Shooter");
        shm.setDirection(DcMotorSimple.Direction.REVERSE);
        shm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shm.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    public void setPower(double pow){
        shm.setPower(pow);
    }

}
