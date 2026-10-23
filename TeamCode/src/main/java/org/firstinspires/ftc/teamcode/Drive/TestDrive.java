package org.firstinspires.ftc.teamcode.Drive;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TestDrive  extends OpMode {

    private Motion bot;
    enum SpeedMode{SLOW, MED, FAST}
    private SpeedMode speedMode =  SpeedMode.SLOW;

    public void init(){
        bot = new Motion(this);
        bot.setPose(new Pose(0,0,0));

    }
    public void loop(){
        bot.updateOdometry();

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


    }

}
