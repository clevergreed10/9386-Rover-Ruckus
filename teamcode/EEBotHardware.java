/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
    FTC Team 9386 Elmer and Elsie Robotics Rover Ruckus
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrivetrain;

/**
 * Main Hardware Class
 *
 * Made by Josh. Rover Ruckus 2018-19 Season
 * // Modifications mchilek 20181117 -- Added servo lift and lift lock parameters and functions
 *
 * Motors, Servos, and other hardware devices
 *
 *
 * Rev IMU:                                 "imu"
 *
 * Motor channel:  Front right drive motor: "wheel1"
 * Motor channel:  Front left drive motor:  "wheel2"
 * Motor channel:  Back left drive motor:   "wheel3"
 * Motor channel:  Back right drive motor:  "wheel4"
 * Motor channel:  Arm motor:               "armMotor"
 * Motor channel:  Spring spool motor:      "springMotor"
 * Motor channel:  Intake spinner:          "intakeMotor"
 * Motor channel;  Arm extension motor:     "winch"
 * Motor channel:  Lift motor:              "liftMotor"
 * Servo channel:  Servo to lock lift :  "liftLockServo"
 * Servo channel:  Servo to help score:  "scoreAssist"
 */
public class EEBotHardware
{
    /* Public OpMode members. */
    public BNO055IMU imu = null;

    // WHEEL CONFIG: WHEEL1 = FrontRight, WHEEL2 = FrontLeft
    public DcMotor wheel1      = null;
    public DcMotor wheel2      = null;
    public DcMotor wheel3      = null;
    public DcMotor wheel4      = null;
    public DcMotor winch       = null;
    public DcMotor armMotor    = null;
    public DcMotor liftMotor = null;
    public DcMotor intakeMotor = null;

    public MecanumDrivetrain drivetrain;

    //public DcMotor liftMotor   = null;
    //public Servo liftLockServo   = null;

    ////public TouchSensor touch = null;

    public Servo pusher = null;
    public Servo stopper = null;

    //public TouchSensor touch  = null;

    public static final double CORE_HEX_TPR   = 288;  // Core Hex has 4 ppr at base, 72:1 gearbox ratio means 288 ppr at shaft.
    public static final double HD_HEX_TPR     = 2240; // HD Hex Motor has 56  ppr at base, geared at 40:1 creates 2,240 ticks ppr at shaft.
    public static final double NEVEREST20_TPR = 28 * 20;
    public static final double NEVEREST40_TPR = 28 * 40;  // Neverest 40 has 7 ppr at base, 40:1 gearbox ratio means 280 ppr at shaft.
    public static final double NEVEREST60_TPR = 28 * 60;  // Neverest 60 has 7 ppr at base, 60:1 gearbox ratio means 280 ppr at shaft.

    public static final double INTAKE_SPEED = 1;

    //servo constants
    public static final double ASSIST_OPEN  = 0;
    public static final double ASSIST_CLOSE = 1;
    public static final double LIFT_LOCK_OPEN  = 0;
    public static final double LIFT_LOCK_CLOSE = 1;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public EEBotHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        // Define and Initialize Motors
        wheel1 = hwMap.get(DcMotor.class, "wheel1");
        wheel2 = hwMap.get(DcMotor.class, "wheel2");
        wheel3 = hwMap.get(DcMotor.class, "wheel3");
        wheel4 = hwMap.get(DcMotor.class, "wheel4");

        wheel2.setDirection(DcMotor.Direction.REVERSE);
        wheel3.setDirection(DcMotor.Direction.REVERSE);

        //liftMotor = hwMap.get(DcMotor.class, "liftMotor");
        //liftLockServo = hwMap.get(Servo.class, "liftLockServo");

        armMotor = hwMap.get(DcMotor.class, "armMotor");
        liftMotor = hwMap.get(DcMotor.class, "liftMotor");

        liftMotor.setDirection(DcMotor.Direction.REVERSE);

        //springMotor = hwMap.get(DcMotor.class, "springMotor");

        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        winch = hwMap.get(DcMotor.class, "winch");

        // Set all motors to zero power
        wheel1.setPower(0);
        wheel2.setPower(0);
        wheel3.setPower(0);
        wheel4.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        wheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Instantiate drivetrain
        drivetrain = new MecanumDrivetrain(new DcMotor[]{wheel1, wheel2, wheel3, wheel4});
        // Define and initialize ALL installed servos.

        pusher = hwMap.get(Servo.class, "pusher ");

        stopper = hwMap.get(Servo.class, "stopper ");

        ////touch = hwMap.get(TouchSensor.class, "touch");
        //liftLockServo = hwMap.get(Servo.class, "liftLockServo");
    }

//    /**
//     * @param engaged the desired state of the lock servo
//     */
//    public void liftBrake(boolean engaged) {
//
//        if (engaged == true) {
//
//            this.liftLockServo.setPosition(1);
//
//        } else if (engaged == false) {
//
//            this.liftLockServo.setPosition(0);
//
//        }
//
//    }
 }

