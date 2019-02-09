package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.EEBotHardware;
import org.firstinspires.ftc.teamcode.PIDControl.PIDController;
import org.firstinspires.ftc.teamcode.Util.Gamepad.Button;

/**
 *  9386 Mecanum Drive Program
 *
 *
 *   - - - CONTROLS - - -
 *
 *   gamepad1:
 *      left stick: moves robot
 *      right stick: rotates robot
 *
 *      right trigger: controls intake
 *      left trigger: controls intake
 *
 *   gamepad2:
 *      TODO: UPDATE
 *
 */
@TeleOp(name="Teleop: Final", group="Robot")

public class TeleopMain extends OpMode {

    private EEBotHardware robot = new EEBotHardware();

    double kR = 1;

    private double forward, strafe, rotate;
    private double front_right, front_left, back_right, back_left;
    private double armPower, liftPower  = 0;
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
        //Update Button Data
        pad2A.update(gamepad2.a);
        pad2B.update(gamepad2.b);
        pad2X.update(gamepad2.x);
        pad2Y.update(gamepad2.y);
        leftBump2.update(gamepad2.left_bumper);
        rightBump2.update(gamepad2.right_bumper);

        // Robot Movement Logic
        if(gamepad1.right_trigger > 0){
            robot.wheel1.setPower(-gamepad1.right_trigger);
            robot.wheel2.setPower(gamepad1.right_trigger);
            robot.wheel3.setPower(-gamepad1.right_trigger);
            robot.wheel4.setPower(gamepad1.right_trigger);
        }
        else if(gamepad1.left_trigger > 0){
            robot.wheel1.setPower(gamepad1.left_trigger);
            robot.wheel2.setPower(-gamepad1.left_trigger);
            robot.wheel3.setPower(gamepad1.left_trigger);
            robot.wheel4.setPower(-gamepad1.left_trigger);
        }
        else{
            robot.wheel1.setPower(-gamepad1.right_stick_y);
            robot.wheel2.setPower(-gamepad1.left_stick_y);
            robot.wheel3.setPower(-gamepad1.left_stick_y);
            robot.wheel4.setPower(-gamepad1.right_stick_y);
        }

        // Scoring logic
        armPower = gamepad2.right_stick_y;

        // find spool power
        if (gamepad2.right_trigger > 0.1) {
            winchPower = -gamepad2.right_trigger;
        } else if (gamepad2.left_trigger > 0.1) {
            winchPower = gamepad2.left_trigger;
        } else {
            winchPower = 0;
        }

        if (gamepad1.x){
            robot.intakeMotor.setPower(-0.7);
        }
        if (gamepad1.a){
            robot.intakeMotor.setPower(0);
        }
        if (gamepad1.b){
            robot.intakeMotor.setPower(1);
        }
        if (gamepad2.a) {
            robot.pusher.setPosition(0);
        } else {
            robot.pusher.setPosition(1);
        }

        //Robot Lift
        liftPower = gamepad2.left_stick_y * -1;

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

        robot.winch.setPower(winchPower);
        //robot.liftMotor.setPower(liftPower);

        robot.armMotor.setPower(armPower);
        robot.liftMotor.setPower(liftPower);

        //robot.intakeMotor.setPower(intakePower);

//        if (robot.touch.isPressed()){
//            if (gamepad2.right_stick_y >= 0){
//                armPower = gamepad2.right_stick_y;
//            }
//            else if (gamepad2.right_stick_y < 0){
//                armPower = 0;
//            }
//        }

        telemetry.addData("Arm Value: ", armPower);
        telemetry.addData("Lift Value: ", liftPower);
        telemetry.addData("Intake Value: ", intakePower);
        telemetry.addData("Winch Value: ", winchPower);
        telemetry.update();
    }

    private double getMotorPower(double stick) {
        // experimenting with sinusoidal curve
        double finalPower = -1 * Math.sin(stick*Math.PI/2); // sin(pi*x/2) >>> period 2, [-1,1]
        return finalPower;
    }
}
