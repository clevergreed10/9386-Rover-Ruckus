package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.EEBotHardware;
import org.firstinspires.ftc.teamcode.PIDControl.PIDController;
import org.firstinspires.ftc.teamcode.Util.Gamepad.Button;

@TeleOp(name="Teleop: Tank", group="Robot")
@Disabled
public class TeleopTank extends OpMode {

    private EEBotHardware robot = new EEBotHardware();


    private double leftPower, rightPower = 0;
    private double armPower, springPower, liftPower  = 0;
    private double winchPower, intakePower = 0;
    private double desiredArmPos = 0;

    private Button pad2A = new Button();
    private Button pad2B = new Button();
    private Button pad2X = new Button();
    private Button pad2Y = new Button();


    private Button leftBump2 = new Button();
    private Button rightBump2 = new Button();

    private PIDController armController = new PIDController(1/50, 1/100, 1/400);

    @Override // @Override tells the computer we intend to override OpMode's method init()
    public void init() {
        robot.init(hardwareMap);

        //robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //armController.setMaxIntegral(100); // CHECK THIS

        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop() {
        double winchRotations = 0; //Math.abs(robot.winch.getCurrentPosition() / robot.CORE_HEX_TPR);


        //Update Button Data
        pad2A.update(gamepad2.a);
        pad2B.update(gamepad2.b);
        pad2X.update(gamepad2.x);
        pad2Y.update(gamepad2.y);
        leftBump2.update(gamepad2.left_bumper);
        rightBump2.update(gamepad2.right_bumper);

        // Robot Movement Logic
        leftPower  = getMotorPower(gamepad1.left_stick_y);
        rightPower = getMotorPower(-gamepad1.right_stick_y);

        armPower = gamepad2.right_stick_y * -0.7;
        springPower = gamepad2.left_stick_y * -1;

        // find spool power
        if (gamepad2.right_trigger > 0.1) {
            winchPower = gamepad2.right_trigger;
        } else if (gamepad2.left_trigger > 0.1) {
            winchPower = -gamepad2.left_trigger;
        } else {
            winchPower = 0;
        }

        if (gamepad1.right_trigger > 0.1) {
            intakePower = gamepad1.right_trigger;
        } else if (gamepad1.left_trigger > 0.1) {
            intakePower = -gamepad1.left_trigger * 0.5;
        } else {
            intakePower = 0;
        }


        //Robot Lift
        liftPower = gamepad2.right_stick_x * -1;

        //Robot Lift Lock
        /*if (pad2X.isToggle()) {
            robot.liftLockServo.setPosition(robot.LIFT_LOCK_OPEN);
        }
        if (pad2Y.isToggle()) {
            robot.liftLockServo.setPosition(robot.LIFT_LOCK_CLOSE);
        }*/

////        if (leftBump2.isToggle()) {
//            intakePower = -robot.INTAKE_SPEED; // out
//        } else if (rightBump2.isToggle()) {
//            intakePower = robot.INTAKE_SPEED; // in
//        } else {
//            intakePower = 0;
//        }

        robot.wheel1.setPower(rightPower);
        robot.wheel2.setPower(leftPower);

        robot.winch.setPower(winchPower);
        //robot.liftMotor.setPower(liftPower);

        robot.armMotor.setPower(armPower);

        //robot.springMotor.setPower(springPower);

        robot.intakeMotor.setPower(intakePower);
    }

    private double getMotorPower(double stick) {
        // experimenting with sinusoidal curve
        double finalPower = -1 * Math.sin(stick*Math.PI/2); // sin(pi*x/2) >>> period 2, [-1,1]
        return finalPower;
    }
}
