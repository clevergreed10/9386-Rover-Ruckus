package org.firstinspires.ftc.teamcode.Drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Wheel {

    private DcMotor motor;

    public Wheel(DcMotor wheel) {
        motor = wheel;
    }

    public int getPos() {
        return motor.getCurrentPosition();
    }

    public void setPower(double power) {
        motor.setPower(power);
    }
}
