package org.firstinspires.ftc.teamcode.Drive;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.HashMap;
import java.util.List;

public class Motion {

    DcMotorEx fL, bL, fR, bR;

    OpMode opMode;

    // GoBildaPinpointDriver odo;
    Localizer localizer;

    VoltageSensor voltageSensor;


    final double FORWARD_TICKS_PER_INCH = 42.3;
    final double STRAFE_TICKS_PER_INCH = 49.8;
    final double TICKS_PER_RADIAN = 631;
    final double MAX_TICKS_PER_SEC = 2400;


    public Motion(OpMode opMode) {
        this.opMode = opMode;

        fL = opMode.hardwareMap.get(DcMotorEx.class, "FL");
        bL = opMode.hardwareMap.get(DcMotorEx.class, "BL");
        fR = opMode.hardwareMap.get(DcMotorEx.class, "FR");
        bR = opMode.hardwareMap.get(DcMotorEx.class,"BR");

        fR.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.REVERSE);

        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        voltageSensor = opMode.hardwareMap.getAll(VoltageSensor.class).get(0);

        setCompensatedPIDFCoefficients(10, 0.5, 0, 13);

//        odo = opMode.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
//        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        odo.setOffsets(6.16, -6.65, DistanceUnit.INCH);
//        odo.setYawScalar(1.00167);
//        odo.resetPosAndIMU();
//        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        localizer = new PinpointLocalizer();

    }

//    public Pose getPose() {
//        Pose2D pose2D = odo.getPosition();
//        Pose result = new Pose(pose2D.getX(DistanceUnit.INCH), pose2D.getY(DistanceUnit.INCH),
//                pose2D.getHeading(AngleUnit.RADIANS));
//        return result;
//    }
//
//    public Pose getVelocity(){
//        return new Pose(odo.getVelX(DistanceUnit.INCH), odo.getVelY(DistanceUnit.INCH),
//                odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
//    }
//
//
//    public double getCumulativeHeading(){
//        return odo.getHeading(AngleUnit.RADIANS);
//    }
//
//    public void setPose(Pose pose) {
//        Pose2D pose2d = new Pose2D(DistanceUnit.INCH, pose.getX(), pose.getY(),
//                AngleUnit.RADIANS, pose.getHeading());
//        //called the method and passed the parameter in to set the position (where the robot is)
//        odo.setPosition(pose2d);
//    }
//
//    public void updateOdometry() {
//        odo.update();
//    }

    public Pose getPose(){
        return localizer.getPose();
    }

    public Pose getVelocity(){
        return localizer.getVelocity();
    }

    public void setPose(Pose pose){
        localizer.setPose(pose);
    }

    public void updateOdometry(){
        localizer.update();
    }

    public void setDrivePower(double px, double py, double pa) {
        double pBL = px + py - pa;
        double pFL = px - py - pa;
        double pFR = px + py + pa;
        double pBR = px - py + pa;
        //power of motor -1 <= power <=1`

        double max = 1;

        max = Math.max(max, Math.abs(pBL));
        max = Math.max(max, Math.abs(pFL));
        max = Math.max(max, Math.abs(pFR));
        max = Math.max(max, Math.abs(pBR));

        if (max >1) {
            pBL /= max; //Java short hand for pbl = pbl/max
            pFL /= max;
            pFR /= max;
            pBR /= max;
        }

        bL.setPower(pBL);
        fL.setPower(pFL);
        fR.setPower(pFR);
        bR.setPower(pBR);

    }

    public void setDriveSpeed(double vx, double vy, double va){
        double px = vx * FORWARD_TICKS_PER_INCH / MAX_TICKS_PER_SEC;
        double py = vy * STRAFE_TICKS_PER_INCH / MAX_TICKS_PER_SEC;
        double pa = va * TICKS_PER_RADIAN / MAX_TICKS_PER_SEC;
        setDrivePower(px, py, pa);
    }

    public void setMotorPowers(double pBL, double pFL, double pFR, double pBR){
        bL.setPower(pBL);
        fL.setPower(pFL);
        fR.setPower(pFR);
        bR.setPower(pBR);
    }

    public void setPIDFCoefficients(double p, double i, double d, double f){
        PIDFCoefficients pidf = new PIDFCoefficients(p, i, d, f);
        bL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        fL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        fR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        bR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    public void setCompensatedPIDFCoefficients(double p, double i, double d, double f){
        double v = voltageSensor.getVoltage();
        setPIDFCoefficients(p*13.5/v, i*13.5/v, d*13.5/v, f*13.5/v);
    }

    public PIDFCoefficients getPIDFCoefficients(){
        return bL.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public HashMap<String,PIDFCoefficients> getAllPIDFCoefficients(){
        HashMap<String,PIDFCoefficients> result = new HashMap<>();
        result.put("bL", bL.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        result.put("fL", fL.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        result.put("fR", fR.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        result.put("bR", bR.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        return result;
    }

    public double getDriveCurrent(){
        return Math.abs(bL.getCurrent(CurrentUnit.AMPS)) + Math.abs(fL.getCurrent(CurrentUnit.AMPS))
                + Math.abs(fR.getCurrent(CurrentUnit.AMPS)) + Math.abs(bR.getCurrent(CurrentUnit.AMPS));
    }

    class PinpointLocalizer implements Localizer {

        GoBildaPinpointDriver odo;

        public PinpointLocalizer (){
            odo = opMode.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            odo.setOffsets(6.16, -6.65, DistanceUnit.INCH);
            odo.setYawScalar(1.002);  // 1.002 is for the v2. scalar for the v1 was 1.00167
            odo.resetPosAndIMU();
            odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        }

        @Override
        public void update() {
            odo.update();
        }

        @Override
        public Pose getPose() {
            Pose2D pose2D = odo.getPosition();
            Pose result = new Pose(pose2D.getX(DistanceUnit.INCH), pose2D.getY(DistanceUnit.INCH),
                    pose2D.getHeading(AngleUnit.RADIANS));
            return result;
        }

        @Override
        public Pose getVelocity() {
            return new Pose(odo.getVelX(DistanceUnit.INCH), odo.getVelY(DistanceUnit.INCH),
                    odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
        }

        @Override
        public void setPose(Pose pose) {
            Pose2D pose2d = new Pose2D(DistanceUnit.INCH, pose.x(), pose.y(),
                    AngleUnit.RADIANS, pose.heading());
            //called the method and passed the parameter in to set the position (where the robot is)
            odo.setPosition(pose2d);
        }
    }

    class DrivetrainLocalizer implements Localizer {

        public int blTicks, flTicks, frTicks, brTicks;
        public IMU imu;
        Pose pose = new Pose(0, 0, 0);
        Pose velocity = new Pose(0,0,0);
        double headingOffsetRadians = 0;

        public DrivetrainLocalizer(){

            imu = opMode.hardwareMap.get(IMU.class, "imu");
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

            List<LynxModule> allHubs = opMode.hardwareMap.getAll(LynxModule.class);
            for (LynxModule module : allHubs) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }

            blTicks = bL.getCurrentPosition();
            flTicks = fL.getCurrentPosition();
            frTicks = fR.getCurrentPosition();
            brTicks = bR.getCurrentPosition();

        }

        public double internalHeadingRadians(){
            double rawHeading = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
            return AngleUnit.normalizeRadians(rawHeading + headingOffsetRadians);
        }

        @Override
        public void update() {
            int blCurr = bL.getCurrentPosition();
            int flCurr = fL.getCurrentPosition();
            int frCurr = fR.getCurrentPosition();
            int brCurr = bR.getCurrentPosition();

            int dBL = blCurr - blTicks;
            int dFL = flCurr - flTicks;
            int dFR = frCurr - frTicks;
            int dBR = brCurr - brTicks;

            double dXRobot = 0.25 * (dBL + dFL + dFR + dBR) / FORWARD_TICKS_PER_INCH;
            double dYRobot = 0.25 * (dBL - dFL + dFR - dBR) / STRAFE_TICKS_PER_INCH;

            double newHeading = internalHeadingRadians();
            double deltaHeading = AngleUnit.normalizeRadians(newHeading - pose.heading());
            double avgHeading = AngleUnit.normalizeRadians(pose.heading() + 0.5 * deltaHeading);

            double sin = Math.sin(avgHeading);
            double cos = Math.cos(avgHeading);

            double dX = dXRobot * cos - dYRobot * sin;
            double dY = dXRobot * sin + dYRobot * cos;

            pose = new Pose(pose.x() + dX, pose.y() + dY, newHeading);

            double vBL = bL.getVelocity();
            double vFL = fL.getVelocity();
            double vFR = fR.getVelocity();
            double vBR = bR.getVelocity();

            double vX = 0.25 * (vBL + vFL + vFR + vBR) / FORWARD_TICKS_PER_INCH;
            double vY = 0.25 * (vBL - vFL + vFR - vBR) / STRAFE_TICKS_PER_INCH;
            double vH = 0.25 * (-vBL - vFL + vFR + vBR) / TICKS_PER_RADIAN;

            velocity = new Pose(vX, vY, vH);

            blTicks = blCurr;
            flTicks = flCurr;
            frTicks = frCurr;
            brTicks = brCurr;

        }

        @Override
        public Pose getPose() {
            return pose;
        }

        @Override
        public Pose getVelocity() {
            return velocity;
        }

        @Override
        public void setPose(Pose newPose) {
            blTicks = bL.getCurrentPosition();
            flTicks = fL.getCurrentPosition();
            frTicks = fR.getCurrentPosition();
            brTicks = bR.getCurrentPosition();
            double rawHeading = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
            headingOffsetRadians = AngleUnit.normalizeRadians(newPose.heading() - rawHeading);
            pose = new Pose(newPose.x(), newPose.y(), newPose.heading());
        }
    }

}
