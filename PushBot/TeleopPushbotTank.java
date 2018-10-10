package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Robot: Teleop Tank", group="Robot")

public class TeleopPushbotTank extends OpMode {

    private EEPushbotHardware robot = new EEPushbotHardware();

    private double leftPower, rightPower = 0;

    @Override // @Override tells the computer we intend to override OpMode's method init()
    public void init() {
        robot.init(hardwareMap);

        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop() {
        // Robot Movement Logic
        leftPower  = gamepad1.left_stick_y * -1; // remember that up is -1, down is 1
        rightPower = gamepad1.right_stick_y * -1;

        robot.wheel1.setPower(rightPower);
        robot.wheel2.setPower(leftPower);
        robot.wheel3.setPower(leftPower);
        robot.wheel4.setPower(rightPower);


        // Winch logic
        if (gamepad1.dpad_down) {
            robot.winch.setPower(-0.25);
        } else if (gamepad1.dpad_up) {
            robot.winch.setPower(0.25);
        } else {
            robot.winch.setPower(0);
        }
    }


    private double getMotorPower(int stick) {
        // experimenting with sinusoidal curve
        double finalPower = -1 * Math.sin(stick*Math.PI/2); // sin(pi*x/2) >>> period 2, [-1,1]
        return finalPower;
    }
}
