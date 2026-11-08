package org.firstinspires.ftc.teamcode.pedro.pathutil;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class PedroAsync {

    private LinearOpMode opMode;
    private Follower follower;

    public static double TURN_TOLERANCE = Math.toRadians(1);
    public static double TURN_SPEED_TOLERANCE = Math.toRadians(10);

    public PedroAsync(LinearOpMode opMode, Follower follower){
        this.opMode = opMode;
        this.follower = follower;
    }

    public void followPathAsync(Path path, Runnable runnable){
        follower.follow(path);
        while (follower.isBusy() && opMode.opModeIsActive()){
            follower.update();
            if (runnable != null){
                runnable.run();
            }
        }
        follower.stop();
        follower.update();
    }

    public void driveToTangent(Pose target, boolean reverse, Runnable runnable){
        Path path = reverse? Paths.line(follower.pose(), target).reverseTangent()
                : Paths.line(follower.pose(), target).tangent();
        followPathAsync(path, runnable);
    }

    public void driveToConstant(Pose target, double heading, Runnable runnable){
        Path path = Paths.line(follower.pose(), target).constant(heading);
        followPathAsync(path, runnable);
    }

    public void driveToLinear(Pose target, double endHeading, Runnable runnable){
        Pose startPose = follower.pose();
        Path path = Paths.line(startPose, target).linear(startPose.heading(), endHeading);
        followPathAsync(path, runnable);
    }

    public void turnTo(Pose targetPose, double targetHeading, Runnable runnable){
        Pose actualTargetPose = new Pose(targetPose.x(), targetPose.y(), targetHeading);
        follower.hold(actualTargetPose);
        while (opMode.opModeIsActive()){
            follower.update();
            Pose pose = follower.pose();
            double headingOffset = Math.abs(AngleUnit.normalizeRadians(targetHeading - pose.heading()));
            Velocity velocity = follower.velocity();
            if (headingOffset < TURN_TOLERANCE && Math.abs(velocity.omega) < TURN_SPEED_TOLERANCE){
                break;
            }
            if (runnable != null){
                runnable.run();
            }
        }
        follower.stop();
        follower.update();
    }

    public void turnTo(double targetHeading, Runnable runnable){
        Pose targetPose = follower.pose();
        turnTo(targetPose, targetHeading, runnable);
    }

}
