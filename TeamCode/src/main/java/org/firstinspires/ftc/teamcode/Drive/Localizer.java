package org.firstinspires.ftc.teamcode.Drive;


import com.pedropathing.math.Pose;

public interface Localizer {

    void update();

    Pose getPose();

    Pose getVelocity();

    void setPose(Pose pose);

}
