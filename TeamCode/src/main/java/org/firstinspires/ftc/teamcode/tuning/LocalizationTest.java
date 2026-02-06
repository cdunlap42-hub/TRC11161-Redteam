package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.MecanumDrive;

//@TeleOp(group = "tuning")
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();

        while (opModeIsActive()) {
            // Drive the robot with joysticks to see if tracking works
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            // --- DEAD WHEEL TELEMETRY ---
            // We access the encoders we defined in MecanumDrive.java
            telemetry.addData("Left Parallel (par0)", drive.leftEncoder.getPositionAndVelocity().position);
            telemetry.addData("Right Parallel (intake)", drive.rightEncoder.getPositionAndVelocity().position);
            telemetry.addData("Perpendicular (conveyance)", drive.perpEncoder.getPositionAndVelocity().position);

            telemetry.addLine("--------------------------------");
            telemetry.addLine("Status: Drive around and check if numbers increase when moving forward!");
            telemetry.update();
        }
    }
}