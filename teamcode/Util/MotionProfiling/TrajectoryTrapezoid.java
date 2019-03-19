package org.firstinspires.ftc.teamcode.Util.MotionProfiling;

public class TrajectoryTrapezoid extends TrajectoryBase {

    Path1D follow = null;
    double start, end;
    double distance;

    double v, a; //maxVelocity, maxAcceleration

    double accelT, constT, decelT;
    double totalT;

    double startT;

    public TrajectoryTrapezoid(TrapezoidPath path, double maxVel, double maxAccel) {
        this(path, maxVel, maxAccel, 0);
    }


    /**
     *      Main Constructor Class
     *  Creates a Trapezoidal (velocity curve) Trajectory for smooth 1 dimensional movement.
     *
     * @param path: the pre-created distance path. This is 1 dimension, so it should only be a straight line.
     * @param maxVel: the max speed the given motor can travel. This will always be positive
     * @param maxAccel: the max acceleration the given motor can apply. This will always be positive
     * @param startTime: this variable localizes time, so you don't have to.
    */
    public TrajectoryTrapezoid(Path1D path, double maxVel, double maxAccel, double startTime) {
        follow = path;
        start = follow.start;
        end   = follow.end;
        distance = end - start;

        startT = startTime;

        v = maxVel;
        a = maxAccel;
        accelT = v/a; // time to reach max velocity

        double distTraveled = getPosFromAccel(accelT);

        if (distTraveled > distance) { // final position will be reached before max velocity is reached
            accelT = Math.sqrt((2*distance)/a);
            constT = 0;
            decelT = 0;

            totalT = accelT;

            return;
        } else if (2*distTraveled > distance) { // final position will be reached in the time it takes to accelerate and decelerate again
            // accelerate to midway point, then decelerate
            accelT = Math.sqrt((distance)/a);
            constT = 0;
            decelT = Math.sqrt((distance)/a);
            totalT = accelT + decelT; // total time to reach position is twice the time it takes to reach the halfway point. Deceleration begins after halfway point.

            return;
        } else {
            // accelerate and decelerate normally in a trapezoidal pattern.
            accelT = accelT;
            decelT = accelT; // take the same time to decelerate as accelerate

            constT = v/(2*distTraveled); // velocity X time = distance . . . time = distance / velocity

            totalT = accelT + decelT + constT;

            return;
        }
    }


    /** Main function for getting the trajectory at a certain time
     *
     * @param time: the current time of the system
     * @return returns a TrajectoryPoint with position, velocity, and accleration values for the given time.
     */
    public TrajectoryPoint getTrajectory(double time) {
        double dT = time - startT;
        double pos, vel, accel;

        // find the range the given time is within.
        if (0 < dT && dT <= accelT) {
            pos = 0.5 * a * Math.pow(dT, 2); // getPosFromAccel could be used here, not using it for readability ATM.
            vel = a*dT; // derivative of position is a*t
            accel = a;
        } else if (accelT < dT && dT <= accelT + constT) {
            pos = v*dT + getPosFromAccel(accelT);
            vel = v; // traveling at max velocity.
            accel = 0; // no change in velocity.
        } else if (accelT + constT < dT && dT <= accelT + constT + decelT) {
            pos = v * (accelT+constT) + -0.5 * a * Math.pow(dT - totalT, 2); // this is a little iffy, might need to change.
            vel = v + -a*dT;
            accel = -a;
        } else {
            pos = (dT < 0 ? 0: Math.abs(distance)); // if not abs, pos will be mixed up if distance is negative
            vel = 0;
            accel = 0;
        }

        if (distance < 0) { // travelling in a negative direction, so pos/velocity/accel needs to be flipped to negative
            pos = -pos;
            vel = -vel;
            accel = -accel;
        }

        return new TrajectoryPoint(start + pos, vel, accel);
    }

    private double getPosFromAccel(double t) {
        return 0.5 * a * (Math.pow(t, 2));
    }
}
