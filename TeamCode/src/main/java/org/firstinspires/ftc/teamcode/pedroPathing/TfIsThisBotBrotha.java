package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
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

@TeleOp
public class TfIsThisBotBrotha extends LinearOpMode {
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);
    private DcMotor intake1, intake2, launcher;
    private Servo gooner;

    public void runOpMode() throws InterruptedException{
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake1.setPower(0);

        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setPower(0);

        launcher = hardwareMap.get(DcMotor.class, "launcher");
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setPower(0);

        gooner = hardwareMap.get(Servo.class, "flicker");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        waitForStart();
        follower.startTeleopDrive();

        while (opModeIsActive()){
            //reset values:
            intake1.setPower(0);
            intake2.setPower(0);
            launcher.setPower(0);

            follower.setTeleOpMovementVectors(-gamepad2.left_stick_y, -gamepad2.left_stick_x, -gamepad2.right_stick_x, false);
            follower.update();

            if (gamepad1.left_bumper){
                intake(1);
            }
            else if (gamepad1.right_bumper){
                intake(-1);
            }
            launcher.setPower(gamepad1.right_trigger);
            telemetry.addData("servo pos:", gooner.getPosition());
            telemetry.update();
        }
    }
    //for organization. Sets both intakes' power in the same direction
    public void intake(int direction){
        intake1.setPower(direction);
        intake2.setPower(direction);
    }
}
