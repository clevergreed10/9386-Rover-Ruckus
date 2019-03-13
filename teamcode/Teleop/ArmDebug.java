package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.EEBotHardware;
import org.firstinspires.ftc.teamcode.PIDControl.PIDController;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Util.Gamepad.Button;

@TeleOp(name="Arm Debug", group="Robot")

public class ArmDebug extends OpMode {

    private EEBotHardware robot = new EEBotHardware();

    private Arm arm = new Arm(robot);

    private Button powerUp = new Button();
    private Button powerDown = new Button();

    private final double ARM_MOTOR_TPR = robot.HD_HEX_TPR; // current arm motor is a Rev HD Hex
    private final double TICKS_PER_DEGREE = ARM_MOTOR_TPR/360;
    private final double GEAR_RATIO = (12.0/86.0) * (28.0/86.0);
    private final double TICKS_PER_ARM_DEGREE = ARM_MOTOR_TPR/(GEAR_RATIO * 360.0);

    private DcMotor armMotor;

    private int armPreset1;
    private int armPreset2;
    private double armPower;
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
        powerDown.update(gamepad2.dpad_down);
        powerUp.update(gamepad2.dpad_up);
        arm.update();

        // adjust arm power
        if (powerDown.isToggle()) {
            armPower -= 0.1;
        } else if (powerUp.isToggle()) {
            armPower += 0.1;
        }

        if (gamepad2.y) {
            arm.reset();
        }

        // set arm power
        if (gamepad2.right_stick_y > 0.1) {
            arm.setArmPower(-armPower);
        } else if (gamepad2.right_stick_y < -0.1) {
            arm.setArmPower(armPower);
        } else {
            arm.setArmPower(0);
        }

        if (gamepad2.a) {
            telemetry.addLine(String.format("MinVelocity: %f", arm.getMinVelocity()));
            telemetry.addLine(String.format("MaxVelocity: %f", arm.getMaxVelocity()));
            telemetry.addLine(String.format("MinAcceleration %f", arm.getMinAcceleration()));
            telemetry.addLine(String.format("MaxAcceleration: %f", arm.getMaxAcceleration()));
        } else {
            telemetry.addLine(String.format("ArmPower: %f", armPower));
            telemetry.addLine(String.format("ArmPosition: %d", arm.getArmPosition()));
            telemetry.addLine(String.format("Velocity: %f", arm.getVelocity()));
            telemetry.addLine(String.format("Acceleration: %f", arm.getAcceleration()));
        }

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
