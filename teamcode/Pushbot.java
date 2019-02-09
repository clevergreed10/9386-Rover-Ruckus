package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Util.Gamepad.Button;

@TeleOp(name="Robot: Pushbot Tank", group="Robot")
//@Disabled
public class Pushbot extends OpMode {

    private EEPushbotHardware robot = new EEPushbotHardware();

    private double leftPower, rightPower, armPower, winchPower = 0;

    private Button pad1A = new Button();
    private Button pad1B = new Button();

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
        armPower = gamepad2.right_stick_y * -1;
        //winchPower  = gamepad2.left_stick_y * -1;

        robot.armDrive.setPower(armPower);
        robot.leftDrive.setPower(leftPower);
        robot.rightDrive.setPower(rightPower);



        pad1A.update(gamepad1.a);
        pad1B.update(gamepad1.b);
        // Winch logic
//        if (gamepad1.dpad_down) {
//            robot.winch.setPower(-0.75);
//        } else if (gamepad1.dpad_up) {
//            robot.winch.setPower(0.75);
//        } else {
//            robot.winch.setPower(0);
//        }

//        if (gamepad1.dpad_left) {
//            robot.armMotor.setPower(-0.5);
//        } else if (gamepad1.dpad_right) {
//            robot.armMotor.setPower(0.5);
//        } else {
//            robot.armMotor.setPower(0);
//        }


    }


    private double getMotorPower(int stick) {
        // experimenting with sinusoidal curve
        double finalPower = -1 * Math.sin(stick*Math.PI/2); // sin(pi*x/2) >>> period 2, [-1,1]
        return finalPower;
    }
}
