package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Util.Gamepad.Button;

/**
 *  Created by Josh 10-19-2018
 *  Testing out joystick drive
 */
@TeleOp(name = "JoystickOp", group = "Testing")
@Disabled
public class TankJoystickDrive extends OpMode {
    private EEBotHardware robot = new EEBotHardware();

    private double yPower = 0; // forward power
    private double xPower = 0;
    private double finalRight = 0;
    private double finalLeft = 0;
    private double max = 0;

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
        yPower = -gamepad1.left_stick_y;
        xPower = gamepad1.left_stick_x;

        finalLeft = yPower - xPower;
        finalRight = yPower + xPower;

        telemetry.addData("Left:", finalLeft);
        telemetry.addData("Right:", finalRight);

        robot.wheel1.setPower(finalRight);
        robot.wheel2.setPower(finalLeft);
        //robot.wheel3.setPower(finalLeft);
        //robot.wheel4.setPower(finalRight);
    }
}
