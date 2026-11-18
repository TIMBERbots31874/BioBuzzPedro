package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.timber_util.Adjuster;

import java.util.ArrayList;


@TeleOp
public class TuneShooter extends OpMode {

    private DcMotorEx shooter;
    private MotorConfigurationType mConfig;
    private Adjuster adj;

    private double targetTicksPerSec = 0;
    private boolean running = false;
    private ElapsedTime et = new ElapsedTime();

    private ArrayList<Double> vList = new ArrayList<>();
    boolean stabilized = false;
    double stabilizationMillis = 0;




    public void init(){
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        mConfig = shooter.getMotorType();
        telemetry.addData("Max RPM", mConfig.getMaxRPM());
        telemetry.addData("Ticks per Rev", mConfig.getTicksPerRev());
        telemetry.addData("Max Fraction", mConfig.getAchieveableMaxRPMFraction());
        telemetry.addData("Achievable Ticks per Sec", mConfig.getAchieveableMaxTicksPerSecond());
        telemetry.addData("Rounded Ticks per Sec", mConfig.getAchieveableMaxTicksPerSecondRounded());
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        targetTicksPerSec = mConfig.getAchieveableMaxTicksPerSecond() / 2.0;
        targetTicksPerSec = 20.0 * Math.round(targetTicksPerSec / 20);
        telemetry.addData("Target ticks per sec", targetTicksPerSec);
    }

    public void start(){
        adj = new Adjuster(() -> gamepad1.dpadRightWasPressed(), () -> gamepad1.dpadLeftWasPressed(),
                () -> gamepad1.dpadUpWasPressed(), () -> gamepad1.dpadDownWasPressed(),
                new Pair<>("F", 13.0), new Pair<>("P", 0.00001));
        shooter.setVelocityPIDFCoefficients(adj.getValue("P"), 0, 0, adj.getValue("F"));
    }

    public void loop(){
        PIDFCoefficients currentPIDF = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        adj.update();
        telemetry.addData("Current", "P: %.5f  F: %.5f", currentPIDF.p, currentPIDF.f);
        telemetry.addData("Adjuster", adj);

        if (gamepad1.aWasPressed()){
            running = !running;
            if (!running){
                shooter.setVelocity(0);
            } else {
                et.reset();
                vList.clear();
                stabilized = false;
                shooter.setVelocity(targetTicksPerSec);
            }
        }

        if (running){
            double v = shooter.getVelocity();
            telemetry.addData("Velocity", v);
            vList.add(v);
            while (vList.size() > 10) {vList.remove(0);}
            if (vList.size() == 10){
                double sum = 0;
                double sumSqr = 0;
                for (int k=0; k<10; k++){
                    sum += vList.get(k);
                    sumSqr += vList.get(k) * vList.get(k);
                }
                double stdDev = Math.sqrt(sumSqr/10.0 - sum*sum/100.0);
                telemetry.addData("Stats", "Millis: %.0f  StdDev: %.1f", et.milliseconds(),
                        stdDev);
                if (!stabilized && stdDev < 40){
                    stabilized = true;
                    stabilizationMillis = et.milliseconds();
                }
                if (stabilized){
                    telemetry.addData("Stabilization Millis", stabilizationMillis);
                }
            }
        }

        telemetry.addData("press A to start or stop running","");
        telemetry.addData("Currently tuning", adj.getCurrentKey());
        telemetry.addData("Press y to switch between f and p", "");
        telemetry.addData("Press X to load adjuster values to motor","");
        if (gamepad1.xWasPressed()){
            shooter.setVelocityPIDFCoefficients(adj.getValue("P"), 0, 0, adj.getValue("F"));
        }

        if (gamepad1.yWasPressed()){
            if (adj.getCurrentKey().equals("F")){
                adj.setCurrentKey("P");
            } else {
                adj.setCurrentKey("F");
            }
        }

    }
}
