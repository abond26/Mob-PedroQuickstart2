package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Config
@TeleOp
public class botTest extends LinearOpMode {
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);
    private DcMotor intake, launcher;
    private Servo flicker;

    public void runOpMode() throws InterruptedException{
        intake = hardwareMap.get(DcMotor.class, "intake");
//        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);

        launcher = hardwareMap.get(DcMotor.class, "launcher");
//        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setPower(0);

        flicker = hardwareMap.get(Servo.class, "flicker");
        flicker.setPosition(1);
            //flickerTwo = hardwareMap.get(Servo.class, "flickerTwo");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        waitForStart();
        follower.startTeleopDrive();

        while (opModeIsActive()){
            // Reset values each loop
//            intake.setPower(0);
//            launcher.setPower(0);

            // Drive control
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
            follower.update();

            // Intake control
            if (gamepad1.left_bumper){
                intake.setPower(1);
            }
            else if (gamepad1.right_bumper){
                intake.setPower(-1);
            }
            else{
                intake.setPower(0);
            }

            // Servo control
            if (gamepad1.y){
                flicker.setPosition(1);
                //flickerTwo.setPosition(0.2);
            }
            else if (gamepad1.a){
                flicker.setPosition(0.95);
               // flickerTwo.setPosition(0);
            }

            // Launcher control
            launcher.setPower(gamepad1.right_trigger);



            // Telemetry
            telemetry.addData("Gooner 1 pos:",  flicker.getPosition());
           // telemetry.addData("Gooner 2 pos:", flickerTwo.getPosition());
            telemetry.addData("Launcher Power:", launcher.getPower());
            telemetry.addData("Intake Power:", intake.getPower());
            telemetry.update();
        }
    }

    // For organization. Sets intake power
    public void intake(int direction){
        intake.setPower(direction);
    }
}