package org.firstinspires.ftc.teamcode.Util.MotionProfiling;

// Base class for 1-dimensional motion paths
public class Path1D{
    double start;
    double end;

    public Path1D(double endTime) {
        this(0.0, endTime);
    }

    public Path1D(double startTime, double endTime) {
        this.start = startTime;
        this.end   = endTime;
    }
}
