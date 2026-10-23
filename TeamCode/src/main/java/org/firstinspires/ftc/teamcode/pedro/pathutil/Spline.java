package org.firstinspires.ftc.teamcode.pedro.pathutil;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.curves.Curve;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Spline implements Curve {
    private CubicSpline2D spline;

    public Spline(List<Pose> controlPoints, double startDir, double endDir, double cStart, double cEnd){
        this.spline = new CubicSpline2D(controlPoints, startDir, endDir, cStart, cEnd);
    }

    public Spline(double startDir, double endDir, double cStart, double cEnd, Pose... poses){
        this(new ArrayList<>(Arrays.asList(poses)), startDir, endDir, cStart, cEnd);
    }
    public Spline(List<Pose> controlPoints, double startDir, double endDir){
        this.spline = new CubicSpline2D(controlPoints, startDir, endDir);
    }

    public Spline(double startDir, double endDir, Pose... poses){
        this(new ArrayList<>(Arrays.asList(poses)), startDir, endDir);
    }

    @Override
    public Vector2D get(double s){
        VectorF p = spline.p(s);
        return Vector2D.cartesian(p.get(0), p.get(1));
    }

    @Override
    public double length(){
        return spline.getTotalPathLength();
    }

    @Override
    public Vector2D derivative(double t) {
        t = Range.clip(t, 0, 1);
        VectorF d1 = spline.d1(t);
        return Vector2D.cartesian(d1.get(0), d1.get(1));
    }

    @Override
    public double curvature(double t){
        return spline.getCurvature(t);
    }


    @Override
    public double closestParameter(Vector2D pos, double s0) {
        double x0 = pos.x();
        double y0 = pos.y();
        double epsilon = .0001f;
        double delta = 100;
        int maxIter = 50;
        int numIter = 0;
        while(delta > epsilon && numIter<maxIter) {
            numIter++;
            VectorF p = spline.p((float)s0);
            VectorF d1 = spline.d1((float)s0);
            VectorF d2 = spline.d2((float)s0);
            double f = (p.get(0) - x0) * d1.get(0) + (p.get(1) - y0) * d1.get(1);
            double fDeriv = (p.get(0) - x0) * d2.get(0) + d1.get(0) * d1.get(0) + (p.get(1) - y0) * d2.get(1) + d1.get(1) * d1.get(1);
            double s = s0 - f / fDeriv;
            delta = Math.abs(s0 - s);
            s0 = s;
        }
        return Range.clip(s0, 0, 1);
    }

}
