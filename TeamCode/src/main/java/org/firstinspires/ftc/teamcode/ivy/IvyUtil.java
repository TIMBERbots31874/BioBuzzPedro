package org.firstinspires.ftc.teamcode.ivy;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class IvyUtil {

    public static Command turnTo(Follower follower, Pose targetPose){
        return Command.build()
                .setStart(()->follower.hold(targetPose))
                .setDone(()->{
                    Pose pose = follower.pose();
                    Velocity vel = follower.velocity();
                    double hdErr = Math.abs(targetPose.heading() - pose.heading());
                    double dstErr = Math.hypot(targetPose.x()-pose.x(), targetPose.y()-pose.y());
                    return hdErr < Constants.foresightConfig.headingConstraint.get()
                            && dstErr < Constants.foresightConfig.translationalConstraint.get()
                            && Math.hypot(vel.vx, vel.vy) < Constants.foresightConfig.velocityConstraint.get()
                            && Math.abs(vel.omega) < Math.toRadians(1);
                });
    }


}
