package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.EEBotHardware;
import org.firstinspires.ftc.teamcode.PIDControl.PIDController;
import org.firstinspires.ftc.teamcode.Util.Gamepad.Button;

@TeleOp(name="Teleop: Arm Test", group="Robot")

public class TeleopTest extends OpMode {

    private EEBotHardware robot = new EEBotHardware();

    private final double ARM_MOTOR_TPR = robot.HD_HEX_TPR; // current arm motor is a Rev HD Hex
    private final double TICKS_PER_DEGREE = ARM_MOTOR_TPR/360;
    private final double GEAR_RATIO = (12.0/86.0) * (28.0/86.0);
    private final double TICKS_PER_ARM_DEGREE = ARM_MOTOR_TPR/(GEAR_RATIO * 360.0);

    private static double MAX_ARM_POWER = 1.0;
    private static double MAX_ARM_CHANGE = 0.2;
    private double leftPower, rightPower = 0;
    private int armTarget, armPosition;

    private DcMotor armMotor;

    private int armPreset1;
    private int armPreset2;
    private double prevArmPower;

    private double kP = 1.0/30.0; // Remember to ensure division by doubles.
    private double kI = 0;
    private double kD = 0;
    private PIDController armController = new PIDController(kP, kI, kD);

    @Override // @Override tells the computer we intend to override OpMode's method init()
    public void init() {
        robot.init(hardwareMap);

        armController.setOutputClip(1.0);

        armMotor = robot.armMotor;

        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop() {
        // spool logic
        double winchPower;

        // find spool power
        if (gamepad2.right_trigger > 0.1) {
            winchPower = -gamepad2.right_trigger;
        } else if (gamepad2.left_trigger > 0.1) {
            winchPower = gamepad2.left_trigger;
        } else {
            winchPower = 0;
        }

        // CONCEPT

        if (gamepad2.x) {
            armTarget = armPreset1;
        } else if (gamepad2.y) {
            armTarget = armPreset2;
        } else {
            armTarget = armTarget - round(gamepad2.left_stick_y * 20);
        }

        if (gamepad2.dpad_left) {
            armPreset1 = armMotor.getCurrentPosition();
        } else if (gamepad2.dpad_right) {
            armPreset2 = armMotor.getCurrentPosition();
        }

        armPosition = armMotor.getCurrentPosition();
        double armErr = armTarget - armPosition;
        double armDegreeError = armTarget - armPosition/TICKS_PER_ARM_DEGREE;
        double armPower = Range.clip(armController.calculate(armErr) - prevArmPower, -MAX_ARM_CHANGE, MAX_ARM_CHANGE);
        prevArmPower = armPower;

        armMotor.setPower(armPower);
        robot.winch.setPower(winchPower);

        telemetry.addData("ArmPosition", armPosition);
        telemetry.addData("ArmTarget:", armTarget);
        telemetry.addData("AngleError:", armDegreeError);
        telemetry.addData("ArmPower", armPower);
        telemetry.update();
    }

    private int round(double num) {
        return (int) Math.round(num);
    }

    private double getMotorPower(double stick) {
        // experimenting with sinusoidal curve
        double finalPower = -1 * Math.sin(stick*Math.PI/2); // sin(pi*x/2) >>> period 2, [-1,1]
        return finalPower;
    }
}
