package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Drive {

    private DcMotorEx bL, fL, fR, bR;
    private GoBildaPinpointDriver pinpoint;


    public void init (HardwareMap hwMap ){
        bL = hwMap.get(DcMotorEx.class, "back_left_motor");
        fL = hwMap.get(DcMotorEx.class, "front_left_motor");
        fR = hwMap.get(DcMotorEx.class, "front_right_motor");
        bR = hwMap.get(DcMotorEx.class, "back_right_motor");

        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        bL.setDirection(DcMotorSimple.Direction.REVERSE);
        fL.setDirection(DcMotorSimple.Direction.REVERSE);

        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setOffsets(4.0, 4.0, DistanceUnit.INCH);


    }
    public void setDrivePower (double px, double py, double pa){

        double pBl = px+py-pa;
        double pFl = px-py-pa ;
        double pFR = px+py+pa;
        double pBR = px-py+pa;

        double max = Math.abs(px) + Math.abs(py) + Math.abs(pa);

        if (max > 1 ) {
            pBl /= max;
            pFl /= max;
            pFR /= max;
            pFl /= max;
        }

        bL.setPower(pBl);
        fL.setPower(pFl);
        fR.setPower(pFR);
        bR.setPower(pBR);

    }
    public void setPose(double xInches, double yInches, double hRadians){
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,xInches,yInches, AngleUnit.RADIANS,hRadians));

    }
    public Pose getPose() {
        Pose2D pose2D = pinpoint.getPosition();
        return new Pose(pose2D.getX(DistanceUnit.INCH),pose2D.getY(DistanceUnit.INCH),
                pose2D.getHeading(AngleUnit.RADIANS));
    }
    public void updateOdometry(){
        pinpoint.update();
    }
}
