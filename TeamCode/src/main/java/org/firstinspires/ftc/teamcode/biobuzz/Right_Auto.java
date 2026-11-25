package org.firstinspires.ftc.teamcode.biobuzz;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.pathutil.Spline;

import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import com.pedropathing.ivy.pedro.PedroCommands;


@Autonomous
public class Right_Auto extends LinearOpMode {

    Follower follower;

    private final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose = p.of(133,59, 180);
    private final Pose shootPose = p.of(121, 59,180);
    private final Pose controlPose1 = p.of(88, 26, 180);
    private final Pose parkPose = p.of(47, 12, 180);

    private final Path path1 = Paths.line(startPose, shootPose).linear(Math.toRadians(180), Math.toRadians(165));
    private final Path parkCurve = Spline.curve(Math.toRadians(-90), Math.toRadians(-180),
           shootPose, controlPose1, parkPose).linear(Math.toRadians(165), Math.toRadians(180));

    Intake intake = new Intake();
    Shooter shooter = new Shooter();

    private Command setIntake(Intake.IntakeState state){
        return instant(() -> intake.setReadyState(state));
    }

    private Command setShooter(double speed){
        return instant(() -> shooter.setVelocity(speed));
    }

    private Command drivePath(Path path){
        return PedroCommands.follow(follower, path);
    };

    private Command mainCommand(){
        return sequential(
                drivePath(path1),
                setShooter(1630),
                waitMs(1000),
                setIntake(Intake.IntakeState.FORWARD),
                waitMs(5000),
                setShooter(0),
                setIntake(Intake.IntakeState.STOPPED),
                drivePath(parkCurve)
        );
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

        shooter.init(hardwareMap);
        intake.init(hardwareMap);

        Scheduler.schedule(mainCommand());

        waitForStart();

        follower.setPose(startPose);
        follower.update();

        while (opModeIsActive()){
            follower.update();
            Scheduler.execute();
        }

    }
}
