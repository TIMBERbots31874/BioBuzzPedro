package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("FL");
        c.frontRightName.set("FR");
        c.backLeftName.set("BL");
        c.backRightName.set("BR");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });


    public static PinpointConfig  localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(6.35);
        c.yPodOffset.set(-6.76);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.19694966050624946);
                Controller secondaryTranslationalForward = Controller.proportional(0.07276763696520791);
                Controller primaryTranslationalLateral = Controller.proportional(0.2467809839979599);
                Controller secondaryTranslationalLateral = Controller.proportional(0.09117897947790832);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01463778109087937));
                c.brake.set(Controller.proportionalFeedforward(0.012442113927247464));

                c.headingFeedback.set(Controller.proportional(3.6124756886119713));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04516655449663614, 0.006421534186047363));

                c.linearBrakeCoefficients.set(Matrix.diag(0.0355134022333415, 0.023428033355363947));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001723145167111934, 0.001998651297447403));

                c.maxAchievableForwardVelocity.set(70.87360342305799);
                c.maxAchievableStrafeVelocity.set(60.698626429738624);
                c.naturalForwardDeceleration.set(30.848086233826987);
                c.naturalStrafeDeceleration.set(59.47743627462667);
            }
    );

//    public static ForesightConfig foresightConfig2 = new ForesightConfig(
//            c -> {
//                Controller primaryTranslationalForward = Controller.proportional(0.19899964449260296);
//                Controller secondaryTranslationalForward = Controller.proportional(0.07352505127158457);
//                Controller primaryTranslationalLateral = Controller.proportional(0.255890541582658);
//                Controller secondaryTranslationalLateral = Controller.proportional(0.09454471759359262);
//
//                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
//                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));
//
//                c.coast.set(Controller.proportionalFeedforward(0.014771422355640836));
//                c.brake.set(Controller.proportionalFeedforward(0.01255570900229471));
//
//                c.headingFeedback.set(Controller.proportional(7.292905457000314));
//                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04251376897163957, 0.006612186963676324));
//
//                c.linearBrakeCoefficients.set(Matrix.diag(0.04940478252073021, 9.793677539395903E-4));
//                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001322610089970663, 0.0020821168835202055));
//
//                c.maxAchievableForwardVelocity.set(69.89471190970698);
//                c.maxAchievableStrafeVelocity.set(59.428873793466);
//                c.naturalForwardDeceleration.set(37.35672298099642);
//                c.naturalStrafeDeceleration.set(64.41763612346281);
//            }
//    );

    public static Follower create(HardwareMap h) {
        // return new Follower(Drivetrain, Localizer, Foresight);
        return null;
    }
}