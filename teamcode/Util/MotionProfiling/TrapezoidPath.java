package org.firstinspires.ftc.teamcode.Util.MotionProfiling;

// velocity curve of this path will be trapezoidal
public class TrapezoidPath extends Path1D{

    private double initialTime = 0;
    private double accelFinishTime, velocityFinishTime, decelFinishTime;
    private double endTime;

    public TrapezoidPath(double endTime) {
        super(endTime);
    }

    public double getPointOnPath(double time) {
        return 1;
    }
}
