package org.firstinspires.ftc.teamcode.AutoPrograms;


import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.EEBotHardware;
import org.firstinspires.ftc.teamcode.PIDControl.PController;
import org.firstinspires.ftc.teamcode.PIDControl.PIDController;

@Autonomous(name="Auto: Depot")

public class AutoDepot extends LinearOpMode {
    EEBotHardware bot = new EEBotHardware();

    double WHEEL_CIRCUMFERENCE = 4 * Math.PI;
    double TICKS_PER_ROTATION  = bot.HD_HEX_TPR;
    double TICKS_PER_INCH      = TICKS_PER_ROTATION/WHEEL_CIRCUMFERENCE;

    double LIFT_EXTENSION_TIME = 2.25;
    double DRIVE_SPEED         = 0.75;
    double STRAFE_SPEED        = 0.5;
    double TURN_SPEED          = 0.5;
    double HOLD_TIME           = 0.25;

    public double DRIVE_kP = 1.0/20;
    public double DRIVE_kI = 1.0/40;
    public double DRIVE_kD = 1.0/15;

    private DcMotor wheel1;
    private DcMotor wheel2;
    private DcMotor wheel3;
    private DcMotor wheel4;

    private GoldDetector detector = new GoldDetector();
    private int goldSide = 1; //-1 = NONE, 0 = LEFT, 1 = CENTER, 2 = RIGHT


    @Override
    public void runOpMode() {
        //initialize
        //WHEEL_CIRCUMFERENCE = 4 * Math.PI;
        //TICKS_PER_ROTATION = bot.HD_HEX_TPR;
        //TICKS_PER_INCH = TICKS_PER_ROTATION/WHEEL_CIRCUMFERENCE;

        //DRIVE_kP = 1/20;
        //DRIVE_kI = 1/40;
        //DRIVE_kD = 1/15;

        bot.init(hardwareMap);

        //detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 1, false);

        wheel1 = bot.wheel1;
        wheel2 = bot.wheel2;
        wheel3 = bot.wheel3;
        wheel4 = bot.wheel4;

        telemetry.addLine("PID Debug:");
        telemetry.addData("Ticks/Inch:", TICKS_PER_INCH);
        telemetry.addData("Var kP:", DRIVE_kP);

        PIDController debugController = new PIDController(DRIVE_kP, DRIVE_kI, DRIVE_kD);
        telemetry.addLine("Ready for Start");
        telemetry.update();

        waitForStart();

        //run commands
        telemetry.addLine("OpMode is Active");
        telemetry.update();

        // - - - LOWER FROM LANDER - - -
        bot.liftMotor.setPower(1);
        gyroHold(LIFT_EXTENSION_TIME);
        bot.liftMotor.setPower(0);

        //detector.enable();

        telemetry.addData("gyroHeading: ", bot.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES));
        telemetry.update();
        //gyroHold(2.0);

        gyroStrafe(STRAFE_SPEED, 0, 4, 1.0);

        gyroHold(0.5);

        gyroDrive(0, DRIVE_SPEED, 4, 1.0);

        //gyroDrive(0, 0.5, 5, 2.0);

        //gyroHold(.5);

        //gyroStrafe(0.5, 1, 4, 2.0);


        // - - - FIND GOLD POS - - -

        bot.liftMotor.setPower(-1);
        double startT = time;
        gyroStrafe(STRAFE_SPEED, 1, 8, 3.0);
        double deltaT = time - startT;
        gyroHold(Range.clip(LIFT_EXTENSION_TIME - deltaT, 0, LIFT_EXTENSION_TIME));
        bot.liftMotor.setPower(0);

        int goldSide = getGoldSide();

        telemetry.addData("gold value: ", goldSide);
        telemetry.update();
        //gyroHold(2.0);

        //detector.disable();

        // - - - SAMPLE GOLD - - -
        gyroStrafe(STRAFE_SPEED, 0, 4, 2.0);

        if (goldSide == 0) { // LEFT
            gyroDrive(45, 0.5, 24, 2.0);
            gyroHold(0.5);

            // - - - NAVIGATE TO DEPOT - - -

            gyroDrive(-45, TURN_SPEED, 10, 2.0);

            gyroHold(HOLD_TIME);

            gyroStrafe(STRAFE_SPEED, 0, 6, 2.0);

            gyroHold(HOLD_TIME);

            gyroDrive(-45, DRIVE_SPEED, 12, 4.0);

            // - - - SCORE TEAM MARKER - - -
            bot.pusher.setPosition(0);
            gyroHold(2.0);
            bot.pusher.setPosition(1);

            // - - - PARK IN CRATER - - -

            gyroDrive(-50, -DRIVE_SPEED, 34, 3.0);

        } else if (goldSide == 1) { // CENTER
            //gyroStrafe(0.5, 0, 6, 2.0);
            gyroDrive(0, 0.5, 24, 2.0);
            gyroHold(0.5);

            // - - - NAVIGATE TO DEPOT - - -

            gyroDrive(-45, TURN_SPEED, 6, 2.0);

            gyroHold(HOLD_TIME);

            gyroStrafe(STRAFE_SPEED, 0, 16, 3.0);

            // - - - SCORE TEAM MARKER - - -
            bot.pusher.setPosition(0);
            gyroHold(2.0);
            bot.pusher.setPosition(1);

            // - - - PARK IN CRATER - - -
            gyroDrive(-45, -DRIVE_SPEED, 18, 5.0);

            gyroHold(HOLD_TIME);

            gyroStrafe(STRAFE_SPEED, 0, 6, 2.0);

            gyroDrive(-45, -DRIVE_SPEED, 20, 4.0);

        } else if (goldSide == 2) { // RIGHT
            gyroDrive(-45, 0.5, 30, 2.0);
            gyroHold(0.5);

            // - - - NAVIGATE TO DEPOT - - -

            gyroStrafe(STRAFE_SPEED, 0, 48, 7.0);

            //gyroDrive(-45, -DRIVE_SPEED, 6, 2.0);

            // - - - SCORE TEAM MARKER - - -
            bot.pusher.setPosition(0);
            gyroHold(2.0);
            bot.pusher.setPosition(1);

            gyroStrafe(STRAFE_SPEED, 0, 8, 3.0);

            // - - - PARK IN CRATER - - -

            gyroDrive(-45, -DRIVE_SPEED, 36, 5.0);
        }
    }

    /**
     * TODO: Think about clipping the correction rather than final power
     *
     * @param targetAngle the desired angle to keep
     * @param desiredPower the preferred motor power
     * @param distance the distance, in inches, to drive
     * @param timeout the amount of time to elapse before aborting
     */
    private void gyroDrive(double targetAngle, double desiredPower, double distance, double timeout) {
        PController controller = new PController(DRIVE_kP);

        distance = distance * TICKS_PER_INCH;

        wheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double rightStart = bot.wheel1.getCurrentPosition(); // top right
        double leftStart = bot.wheel2.getCurrentPosition(); // top left

        double startTime = time;

        while (opModeIsActive() && Math.abs(wheel1.getCurrentPosition() - rightStart) < distance && Math.abs(wheel2.getCurrentPosition() - leftStart) < distance) {
            if (time > startTime + timeout) { // timeout
                telemetry.addLine("Drive loop timeout.");
                break;
            }

            Orientation orientation = bot.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            double angle = orientation.thirdAngle;
            double error = targetAngle - angle;
            double correction = controller.calculate(error); //controller.calculate(error);
            double rightPower = desiredPower + correction;
            double leftPower  = desiredPower - correction;

            double max = Math.max(Math.abs(rightPower), Math.abs(leftPower));
            if (max > 1) { // clip the power between -1,1 while retaining relative speed percentage
                rightPower = rightPower / max;
                leftPower = leftPower / max;
            }

            telemetry.addData("error", error);
            telemetry.addData("correction", correction);
            telemetry.addData("rightPower", rightPower);
            telemetry.addData("leftPower", leftPower);
            //telemetry.addData("kP", controller.getkP());
            //telemetry.addData("rightEncoder", bot.wheel1.getCurrentPosition());
            //telemetry.addData("distance", distance);
            //telemetry.addData("Current angle:", orientation.thirdAngle);
            telemetry.update();
            wheel1.setPower(rightPower);            wheel4.setPower(rightPower);
            wheel2.setPower(leftPower);
            wheel3.setPower(leftPower);
        }

        wheel1.setPower(0);
        wheel4.setPower(0);
        wheel2.setPower(0);
        wheel3.setPower(0);
    }

    private void gyroDirectionalDrive(double direction, double power, double distance, boolean turn, double timeout) {
        MecanumDrivetrain drivetrain = bot.drivetrain;

        PController controller = new PController(1.0/20);

        distance = distance * TICKS_PER_INCH;

        drivetrain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double rightStart = bot.wheel1.getCurrentPosition(); // top right
        double leftStart = bot.wheel2.getCurrentPosition(); // top left

        double startTime = time;
        double startAngle = bot.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

        double forward, strafe, rotate;

        // drive loop
        while (opModeIsActive() && Math.abs(wheel1.getCurrentPosition() - rightStart) < distance && Math.abs(wheel2.getCurrentPosition() - leftStart) < distance) {
            if (time > startTime + timeout) { // timeout
                telemetry.addLine("Drive loop timeout.");
                break;
            }

            Orientation orientation = bot.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            double angle = orientation.thirdAngle;
            double error = startAngle - angle;
            double correction = 0;//Range.clip(controller.calculate(error), -1, 1); //controller.calculate(error);

            if (turn) {
                forward = 0;
                strafe  = 0;
                rotate = correction;
            } else {
                forward = Math.cos(Math.toRadians(direction));
                strafe  = Math.sin(Math.toRadians(direction));
                rotate  = correction;
            }

            drivetrain.drive(forward, strafe, rotate);

            telemetry.addData("error", error);
            telemetry.addData("correction", correction);
            //telemetry.addData("kP", controller.getkP());
            //telemetry.addData("rightEncoder", bot.wheel1.getCurrentPosition());
            //telemetry.addData("distance", distance);
            //telemetry.addData("Current angle:", orientation.thirdAngle);
            telemetry.update();
        }

        wheel1.setPower(0);
        wheel4.setPower(0);
        wheel2.setPower(0);
        wheel3.setPower(0);
    }

    public boolean isActive() { // safe cleanup
        if (!opModeIsActive()) {
            //detector.disable();
            return false;
        } else {
            return true;
        }
    }

    public void gyroHold(double holdTime) {
        double startTime = time;
        while (isActive() && time - startTime < holdTime) {
            Thread.yield();
        }
    }

    public void gyroStrafe(double speed, int direction, double distance, double timeout) {

        distance = distance * TICKS_PER_INCH;

        double rightStart = bot.wheel2.getCurrentPosition(); // top right
        double leftStart = bot.wheel1.getCurrentPosition(); // top left

        double startTime = time;

        if (direction == 0) {
            //Strafe left
            while (isActive() && Math.abs(wheel1.getCurrentPosition() - leftStart) < distance) {

                if (time > startTime + timeout) { // timeout
                    telemetry.addLine("Drive loop timeout.");
                    break;
                }

                wheel1.setPower(speed);
                wheel2.setPower(-speed);
                wheel3.setPower(speed);
                wheel4.setPower(-speed);
            }
            wheel1.setPower(0);
            wheel2.setPower(0);
            wheel3.setPower(0);
            wheel4.setPower(0);
        }

        if (direction == 1) {
            //Strafe right
            while (isActive() && Math.abs(wheel2.getCurrentPosition() - rightStart) < distance) {
                if (time > startTime + timeout) { // timeout
                    telemetry.addLine("Drive loop timeout.");
                    break;
                }

                wheel1.setPower(-speed);
                wheel2.setPower(speed);
                wheel3.setPower(-speed);
                wheel4.setPower(speed);
            }
            wheel1.setPower(0);
            wheel2.setPower(0);
            wheel3.setPower(0);
            wheel4.setPower(0);
        }
    }


    private void driveByGyro(double targetAngle, double power,  double distance, double timeout) {

        double encoderTicks = inchesToTicks(distance);
        // Set the motors to measure encoder counts.
        wheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //wheel3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //wheel4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int startRPos = wheel1.getCurrentPosition();
        int startLPos = wheel2.getCurrentPosition();
        double startTime = time;

        power = Range.clip(-power, -1, 1);
        // Start the loop. Even if we're going backwards, math.abs makes sure that we can still compare to distance.
        while (isActive() && (Math.abs(wheel2.getCurrentPosition() -startRPos) < distance) && (Math.abs(wheel1.getCurrentPosition() -startLPos) < distance)) {
            if (time > startTime + timeout) {
                telemetry.addData("Last drive stop:", "Timeout!");
                break;
            }

            Orientation orientation = bot.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            // angles                  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double error = (targetAngle - (orientation.thirdAngle > 180 ? orientation.thirdAngle - 360 : orientation.thirdAngle));
            double correction = error * DRIVE_kP;
            //error = Range.clip(error, -1, 1);
            telemetry.addData("motorleft dist:", wheel2.getCurrentPosition() - startLPos);
            telemetry.addData("motorright dist:", wheel1.getCurrentPosition() - startRPos);
            telemetry.addData("Current z orientation:", orientation.thirdAngle);
            telemetry.addData("error", error);
            telemetry.update();
            double right = power + correction;
            double left = power - correction;

            double max = Math.max(Math.abs(right), Math.abs(left));

            if (max > 1.0) {
                left /= max;
                right /= max;
            }

            wheel1.setPower(right);
            //wheel4.setPower(right);
            //wheel3.setPower(left);
            wheel2.setPower(left);
        }

        telemetry.addData("Last drive stop:", "Loop complete!");
        wheel1.setPower(0.0);
        wheel2.setPower(0.0);
        //wheel3.setPower(0.0);
        //wheel4.setPower(0.0);

        wheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //wheel3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //wheel4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //return (time < startTime + timeout);
    }

    private int getGoldSide() { // this will probably need work
        detector.enable();
        gyroHold(2.0);
        if (detector.getScreenPosition().x > 0 && detector.getScreenPosition().y > 300) {
            double pos = detector.getScreenPosition().x;
            detector.disable();
            if (pos <= 340) {
                return 1; // center
            } else {
                return 2; // right
            }
        } else {
            detector.disable();
            return 0; // LEFT/not found
        }
    }

    private double averageArray(double[] vals) {
        double average = 0;

        for (int i = 0; i <= vals.length - 1; i++ ) {
            average = average + vals[i];
        }

        return average/vals.length;
    }

    private double inchesToTicks(double inches) {
        return inches * TICKS_PER_INCH;
    }
}
