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

@Autonomous(name="Auto: Crater")

public class AutoFullCraterUpdate extends LinearOpMode {
    EEBotHardware bot = new EEBotHardware();

    double WHEEL_CIRCUMFERENCE = 4 * Math.PI;
    double TICKS_PER_ROTATION  = bot.HD_HEX_TPR;
    double TICKS_PER_INCH      = TICKS_PER_ROTATION/WHEEL_CIRCUMFERENCE;

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

        detector.enable();
        //run commands
        telemetry.addLine("OpMode is Active");
        telemetry.update();

        // - - - LOWER FROM LANDER - - -
        bot.armMotor2.setPower(1);
        gyroHold(2.5);
        bot.armMotor2.setPower(0);

        gyroHold(2.0);

        gyroStrafe(0.5, 1, 6, 2.0);

        gyroDrive(0, 0.5, 5, 2.0);

        gyroHold(.5);

        gyroStrafe(0.5, 1, 4, 2.0);

        bot.armMotor2.setPower(-1);
        gyroHold(2.5);
        bot.armMotor2.setPower(0);

        // - - - FIND GOLD POS - - -
        gyroHold(3.0);

        int goldSide = getGoldSide();

        detector.disable();
//        gyroDirectionalDrive(0, 1, 12, false, 3.0);
//        gyroDirectionalDrive(90, 1, 12, false, 4.0);
//        gyroHold(1.0);
//        gyroDirectionalDrive(-90, 1, 12, false, 4.0);
//        gyroDrive(0, -0.5, 2, 3.0);
//        gyroHold(0.5);


        // - - - SAMPLE GOLD - - -
        gyroStrafe(0.5, 0, 6, 2.0);

        if (goldSide == 0) {
            gyroDrive(45, 0.5, 20, 3.0);
            gyroHold(0.5);
            gyroDrive(30, -0.5, 12, 3.0);
        } else if (goldSide == 1) {
            //gyroStrafe(0.5, 0, 6, 2.0);
            gyroDrive(0, 0.5, 12, 3.0);
            gyroHold(0.5);
            gyroDrive(0, -0.5, 6, 3.0);
        } else if (goldSide == 2) {
            gyroDrive(-15, 0.5, 18, 3.0);
            gyroHold(0.5);
            gyroDrive(-30 , -0.5, 8 , 3.0);
        }

       // - - - NAVIGATE TO DEPOT - - -

        gyroHold(0.5);

        gyroDrive(90, 0.5, 36, 4.0);

        gyroHold(0.5);

        gyroDrive(135, 0.5, 12, 3.0);

        gyroHold(.5);

        gyroStrafe(0.5, 1, 6, 2.0);

    }

    private void measureGyroAccuracy(double timeout) {
        double targetAngle = 0;

        double startTime = time;
        double[] iterationTimes = new double[100];

        double startLPos = wheel1.getCurrentPosition();
        double startRPos = wheel2.getCurrentPosition();
        double power = 0;
        int i = 0;
        for (i = 0; i <= 99; i++) {
            if (time - startTime > timeout) {
                telemetry.addData("last drive stop", time);
                break;
            }
            double loopStart = time;
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
            double right = power - correction;
            double left = power + correction;

            double max = Math.max(Math.abs(right), Math.abs(left));

            if (max > 1.0) {
                left /= max;
                right /= max;
            }

            iterationTimes[i] = time - loopStart;
        }

        double averageIterationTime = averageArray(iterationTimes);

        telemetry.addData("AVG TIME:", averageIterationTime);
        telemetry.update();
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
            wheel1.setPower(rightPower);
            wheel4.setPower(rightPower);
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

    public void gyroHold(double holdTime) {
        double startTime = time;
        while (opModeIsActive() && time - startTime < holdTime) {
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
            while (opModeIsActive() && Math.abs(wheel1.getCurrentPosition() - leftStart) < distance) {
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
            while (opModeIsActive() && Math.abs(wheel2.getCurrentPosition() - rightStart) < distance) {
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
        while (opModeIsActive() && (Math.abs(wheel2.getCurrentPosition() -startRPos) < distance) && (Math.abs(wheel1.getCurrentPosition() -startLPos) < distance)) {
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
        if (detector.getScreenPosition().x > 0 && detector.getScreenPosition().y > 300) {
            double pos = detector.getXPosition();
            if (pos <= 340) {
                return 1; // center
            } else {
                return 2; // right
            }
        } else {
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
