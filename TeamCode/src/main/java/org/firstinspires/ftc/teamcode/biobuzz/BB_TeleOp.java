package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.Intake.IntakeState.STOPPED;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp
public class BB_TeleOp extends OpMode {

    private Follower follower;
    enum SpeedMode{SLOW, MED, FAST}
    private SpeedMode speedMode =  SpeedMode.SLOW;
    private boolean fieldCentric = false;
    private boolean holdpose = false;
    private GoBildaPinpointDriver pinpoint;
    private Intake intake = new Intake();

    private Shooter shooter = new Shooter();
    private double highShooterSpeed = 1630;//this is where to change the speed, was 2000
    private double lowShooterSpeed = 1401;

    private double shooterSpeed = highShooterSpeed;

    Intake.IntakeState intakeState = STOPPED;

    public void init(){
       follower = Constants.create(hardwareMap);

       if(follower == null){
           telemetry.addData("nullfollower","");
       }

        intake.init(hardwareMap);

        shooter.init(hardwareMap);
    }

    public void start(){
        follower.setPose(new Pose(0,0,0));
    }



    public void loop(){

        // HANDLE DRIVE TRAIN

        if(gamepad1.aWasPressed()){
            if(speedMode == SpeedMode.SLOW) speedMode = SpeedMode.MED;
            else if (speedMode == SpeedMode.MED) speedMode = SpeedMode.FAST;
            else speedMode = SpeedMode.SLOW;
        }

        if (gamepad1.xWasPressed()){
            fieldCentric = !fieldCentric;
        }

        if (gamepad1.yWasPressed()){
            holdpose = !holdpose;
        }

        telemetry.addData("Speed Mode", speedMode);
        telemetry.addData("fieldCentric", fieldCentric);
        telemetry.addData("holdpose", holdpose);

        double scale = speedMode == SpeedMode.SLOW? 2.0
                : speedMode == SpeedMode.MED? 1.5 : 1.0;

        double px = -gamepad1.left_stick_y;
        double py = -gamepad1.left_stick_x ;
        double pa = gamepad1.left_trigger-gamepad1.right_trigger;

        DrivePowers drivePowers;

        if (fieldCentric){
            drivePowers = ManualDrive.fieldCentric(px, py, pa, follower.pose().heading());
        }else {
            drivePowers = new DrivePowers(px, py, pa);
        }

        if (holdpose){
            ManualDrive.driveOrHold(follower, drivePowers);
        }else {
            follower.manual(drivePowers);
        }

        follower.update();

        Pose pose = follower.pose();
        telemetry.addData("pose","x: %.1f  y: %.1f  h: %.1f",
                pose.x(),pose.y(),Math.toDegrees(pose.heading()));

        // HANDLE INTAKE

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


        // HANDLE SHOOTER

        if (gamepad1.dpadUpWasPressed()){
            shooterSpeed = highShooterSpeed;
        }

        if (gamepad1.bWasPressed()){
            if (shooter.getPower() == 0.0){
                shooter.setVelocity(highShooterSpeed);
            }else{
                shooter.setVelocity(0);
            }
        }


    }



}
