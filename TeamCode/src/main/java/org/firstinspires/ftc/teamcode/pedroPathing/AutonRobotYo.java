// note ; some of the constants input may be inaccurate when we get the robot, check everything before running


package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;


@Autonomous(name="Basic Autonomous", group="Linear Opmode")
public class AutonRobotYo extends LinearOpMode {
//declared motors n stuff
    private DcMotor leftFront, rightFront,rightBack,leftBack;

    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);
    private DcMotor intake, launcher;
    private Servo goonerOne; //goonerTwo;
    private ElapsedTime runtime = new ElapsedTime();



    // Drive constants - ADJUST AND CHECK BEFORE RUNNING
    static final double COUNTS_PER_MOTOR_REV = 537.7;  // eg: GoBILDA 312 RPM
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 3.77953;  // 96mm wheels
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.4;

    int xMove;
    int yMove;

    //graphing an area







    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // Initialize hardware
        initHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Run AUTON sequence
        if (opModeIsActive()) {
            // Example autonomous routine

            // reset all variable
            xMove =58;
            yMove=12;

            yMove=35;
            //forward 23 inches
            encoderDrive(DRIVE_SPEED,23,23,4.0 );
            //turns left
            encoderDrive(TURN_SPEED, -12, 12, 3.0);


            xMove=23;
            //forward 35 inches
            encoderDrive(DRIVE_SPEED, 35,35,3.0);
            //turns right
            encoderDrive(TURN_SPEED, 6, -6, 3.0);

            xMove=72;
            yMove=72;
            // robot move right
            strafe(DRIVE_SPEED, 60, 4.0);

            //angled to thing
            encoderDrive(DRIVE_SPEED,12,-12,3.0);

            //imaginary shoot in the launcher



            //turns left by 45+ 90, now the robot is completely facing west
            encoderDrive(TURN_SPEED, -12,12,4.0);
            encoderDrive(TURN_SPEED, -6, 6, 3.0);
            //position
            yMove= 62;
            //move to the next ball left
            strafe(DRIVE_SPEED, -10, 3.0);

            xMove= 20;
            //move forward + turn on intake
            encoderDrive(TURN_SPEED, 52,52,4.0);

            xMove=72;
            encoderDrive(TURN_SPEED, -52,-52,4.0);
            yMove=72;
            strafe(DRIVE_SPEED, 10, 3.0);

            encoderDrive(TURN_SPEED, 12,-12,4.0);
            encoderDrive(TURN_SPEED, 6, -6, 3.0);


            //facing north
            encoderDrive(TURN_SPEED,-6,6,3.0);
            xMove=44;
            strafe(DRIVE_SPEED, -28, 3.0);

            yMove=12;
            //facing west
            encoderDrive(TURN_SPEED, -12,12,4.0);
            //turn on intake
            encoderDrive(DRIVE_SPEED,12,-12,3.0);

            //
















                    // Drive forward 24 inches
           // encoderDrive(DRIVE_SPEED, 24, 24, 5.0);

            // Turn right 90 degrees (adjust distance for your robot)
            //encoderDrive(TURN_SPEED, 12, -12, 4.0);

            // Drive forward 12 inches
            //encoderDrive(DRIVE_SPEED, 12, 12, 3.0);

            // Strafe right (if using mecanum wheels)
            // strafe(DRIVE_SPEED, 12, 3.0);
            // Optional: Control arm or claw
            // moveArm(0.5, 500);
            // operateClaw(true);
            // sleep(1000);
            // operateClaw(false);

            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000);
        }
    }

    private void initHardware() {
        // Initialize drive motors
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        // Set motor directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // Set motors to brake when power is zero
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Optional: Initialize other hardware
        // armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        // clawServo = hardwareMap.get(Servo.class, "claw_servo");
    }

    public void encoderDrive(double speed, double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        if (opModeIsActive()) {
            // Calculate new target positions
            newLeftFrontTarget = leftFront.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFront.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftBackTarget = leftBack.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightBackTarget = rightBack.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            // Set target positions
            leftFront.setTargetPosition(newLeftFrontTarget);
            rightFront.setTargetPosition(newRightFrontTarget);
            leftBack.setTargetPosition(newLeftBackTarget);
            rightBack.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Reset runtime and start motion
            runtime.reset();
            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            leftBack.setPower(Math.abs(speed));
            rightBack.setPower(Math.abs(speed));

            // Keep looping while we are still active, and there is time left, and both motors are running
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {

                // Display info for driver
                telemetry.addData("Running to",  " %7d :%7d", newLeftFrontTarget, newRightFrontTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        leftFront.getCurrentPosition(), rightFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);  // Optional pause after each move
        }
    }

    // Optional: Strafe function for mecanum wheels
    public void strafe(double speed, double inches, double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        if (opModeIsActive()) {
            // For strafing: LF and RB go forward, RF and LB go backward (or vice versa)
            newLeftFrontTarget = leftFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFront.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            newLeftBackTarget = leftBack.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            newRightBackTarget = rightBack.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

            leftFront.setTargetPosition(newLeftFrontTarget);
            rightFront.setTargetPosition(newRightFrontTarget);
            leftBack.setTargetPosition(newLeftBackTarget);
            rightBack.setTargetPosition(newRightBackTarget);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            leftBack.setPower(Math.abs(speed));
            rightBack.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                    (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {
                telemetry.addData("Strafing", "To %7d", newLeftFrontTarget);
                telemetry.update();
            }

            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);

            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}