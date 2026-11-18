package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Shooter {
    public DcMotorEx shm;

    public PIDFCoefficients pidf = new PIDFCoefficients(11, 0,0, 14);

    public void init(HardwareMap hwMap){
        shm = hwMap.get(DcMotorEx.class, "Shooter");
        shm.setDirection(DcMotorSimple.Direction.REVERSE);
        shm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shm.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    public void setPower(double pow){
        shm.setPower(pow);
    }

    public void setVelocity(double vel){
        shm.setVelocity(vel);
    }

    public double getPower(){
        return shm.getPower();
    }

}
