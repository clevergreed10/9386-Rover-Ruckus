package org.firstinspires.ftc.teamcode.PIDControl;

public class PDController extends PIDController {

    public PDController(double kP, double kD) {

        super(kP, 0, kD);
    }
}
