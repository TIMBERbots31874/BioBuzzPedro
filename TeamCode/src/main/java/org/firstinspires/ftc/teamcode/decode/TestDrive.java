package org.firstinspires.ftc.teamcode.decode;

import static org.firstinspires.ftc.teamcode.biobuzz.Intake.IntakeState.REVERSE;
import static org.firstinspires.ftc.teamcode.biobuzz.Intake.IntakeState.STOPPED;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.biobuzz.Intake;
import org.firstinspires.ftc.teamcode.biobuzz.Shooter;

@TeleOp
public class TestDrive  extends OpMode {

    private Motion bot;
    enum SpeedMode{SLOW, MED, FAST}
    private SpeedMode speedMode =  SpeedMode.SLOW;
    private GoBildaPinpointDriver pinpoint;
    private Intake intake = new Intake();

    private Shooter shooter = new Shooter();
    private double shooterSpeed = 1300;  //this is where to change the speed was 1000

    Intake.IntakeState intakeState = STOPPED;

    public void init(){
        bot = new Motion(this);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        bot.setPose(new Pose(0,0,0));

        intake.init(hardwareMap);

        shooter.init(hardwareMap);
    }



    public void loop(){
        bot.updateOdometry();
        telemetry.addData("x ticks", pinpoint.getEncoderX());
        telemetry.addData("Y ticks", pinpoint.getEncoderY());

        telemetry.addData("pose", "%.1f  %.1f  %.1f", bot.getPose().x(),
                bot.getPose().y(), Math.toDegrees(bot.getPose().heading()));

        if(gamepad1.aWasPressed()){
            if(speedMode == SpeedMode.SLOW) speedMode = SpeedMode.MED;
            else if (speedMode == SpeedMode.MED) speedMode = SpeedMode.FAST;
            else speedMode = SpeedMode.SLOW;
        }

        telemetry.addData("Speed Mode", speedMode);

        double scale = speedMode == SpeedMode.SLOW? 2.0
                : speedMode == SpeedMode.MED? 1.5 : 1.0;

        double px = -gamepad1.left_stick_y / scale;
        double py = -gamepad1.left_stick_x / scale;
        double pa = -gamepad1.right_stick_x / scale;

        bot.setDrivePower(px,py,pa);


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


        if (gamepad1.bWasPressed()){
            if (shooter.getPower() == 0.0){
                shooter.setVelocity(shooterSpeed);
            }else{
                shooter.setVelocity(0);
            }
        }


    }



}
