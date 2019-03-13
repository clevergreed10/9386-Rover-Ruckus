package org.firstinspires.ftc.teamcode.Util.MotionProfiling;

public class TrajectoryPoint {

    public double position;
    public double velocity;
    public double acceleration;

    public TrajectoryPoint(double x, double v, double a) {
        this.position = x;
        this.velocity = v;
        this.acceleration = a;
    }
}
