package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseRaw;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class ClusterUtil {

    private VisionPortal portal;
    private AprilTagProcessor processor;

    public static OpenGLMatrix ROBOT_TO_SHOOTER =
            OpenGLMatrix.rotation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES, 0, 0, 0)
                    .translated(0, 0, 0);

    public static OpenGLMatrix CAMERA_TO_ROBOT =
            OpenGLMatrix.rotation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES, -90, -45, 0)
                    .translated(0, 0, 0);

    public static OpenGLMatrix CAMERA_TO_SHOOTER = ROBOT_TO_SHOOTER.multiplied(CAMERA_TO_ROBOT);



    /**
     * Initialize the VisionPortal and AprilTagProcessor. If the CameraState becomes "STREAMING"
     * within 1.0 second, return true (success). Otherwise, return false (failure).
     * @param hardwareMap
     * @param useWebCam
     * @return
     */
    public boolean init(HardwareMap hardwareMap, boolean useWebCam){
        processor = new AprilTagProcessor.Builder().build();
        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (useWebCam) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.addProcessor(processor);

        portal = builder.build();

        ElapsedTime et = new ElapsedTime();
        while (et.seconds() < 1){
            if (portal.getCameraState() == VisionPortal.CameraState.STREAMING){
                return true;
            }
        }
        return false;
    }

    public boolean streaming(){
        return portal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    public List<AprilTagClusterDetection> getClusters(){
        if (!streaming()) {
            return null;
        }
        List<AprilTagDetection> tagDetections = processor.getFreshDetections();
        if (tagDetections == null || tagDetections.isEmpty()){
            return null;
        }

        List<AprilTagClusterDetection> result = new ArrayList<>();

        for (AprilTagDetection d : tagDetections){
            if (d instanceof AprilTagClusterDetection) result.add((AprilTagClusterDetection) d);
        }

        if (result.isEmpty()){
            return null;
        }

        return result;
    }

    public static OpenGLMatrix getRawMatrix(AprilTagDetection d){
        AprilTagPoseRaw p = d.rawPose;

        float[] data = new float[]{
                p.R.get(0,0), p.R.get(1,0), p.R.get(2,0), 0,
                p.R.get(0,1), p.R.get(1,1), p.R.get(2,1), 0,
                p.R.get(0,2), p.R.get(1,2), p.R.get(2,2), 0,
                (float)p.x, (float)p.y, (float)p.z, 1
        };

        return new OpenGLMatrix(data);
    }

    public static OpenGLMatrix aprilTagToShooter(AprilTagDetection d){
        OpenGLMatrix tagToCamera = getRawMatrix(d);
        return CAMERA_TO_SHOOTER.multiplied(tagToCamera);
    }

    public void stopStreaming(){
        portal.stopStreaming();
    }

    public void resumeStreaming(){
        portal.resumeStreaming();
    }

    public void close(){
        portal.close();
    }


}
