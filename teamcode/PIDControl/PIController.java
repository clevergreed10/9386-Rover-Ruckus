package org.firstinspires.ftc.teamcode.PIDControl;

public class PIController extends PIDController {

    public PIController(double kP, double kI) {
        super(kP, kI, 0);
    }
}
