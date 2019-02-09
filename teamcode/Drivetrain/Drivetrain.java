package org.firstinspires.ftc.teamcode.Drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * All drivetrains (mecanum, tank, etc.) inherit from this class
 */
public abstract class Drivetrain{

    public DcMotor[] wheels;

    public Drivetrain(DcMotor[] motors) {
        wheels = motors;
    }

    public void stop(boolean reset) {
        for (int i = 0; i <= wheels.length;  i++) {
            wheels[i].setPower(0);
            if (reset) {
                wheels[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
    }

    public void drive(double power) {
        for (int i = 0; i <= wheels.length - 1;  i++) {
            wheels[i].setPower(power);
        }
    }

    public void setMode(DcMotor.RunMode mode){
        for (int i = 0; i<= wheels.length - 1; i++) {
            wheels[i].setMode(mode);
        }
    }
}
