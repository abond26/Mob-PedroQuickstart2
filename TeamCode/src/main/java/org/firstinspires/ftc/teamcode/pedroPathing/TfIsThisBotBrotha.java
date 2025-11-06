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

@TeleOp
@Config
public class TfIsThisBotBrotha extends LinearOpMode {
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);
    private DcMotor intake,launcher;
    private Servo gooner;

    public void runOpMode() throws InterruptedException{
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);


        launcher = hardwareMap.get(DcMotor.class, "launcher");
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setPower(0);

        //gooner = hardwareMap.get(Servo.class, "gooner");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        waitForStart();
        follower.startTeleopDrive();

        while (opModeIsActive()){
            //reset values:
            if (gamepad1.left_bumper){
                intake(1);
            }

            else if (gamepad1.right_bumper){
                intake(-1);
            }
            else {
                intake(0);
            }
           // if (gamepad1.a){
              //  flicker.setPosition(flickerPose);
           // }
            //if (gamepad1.b){
                //flicker.setPosition(flickerDownPose);
            //}
            if (gamepad1.x){
                launcher.setPower(1);
            }
            else{
                launcher.setPower(0);
            }

            //telemetry.addData("servo pos:", flicker.getPosition());
            telemetry.update();
        }
    }
    //for organization. Sets both intakes' power in the same direction
    public void intake(int direction){
        intake.setPower(direction);
    }
}
