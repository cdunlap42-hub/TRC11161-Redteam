package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Revolver Calibration Tool", group = "Calibration")
public class RevolverCalib_11161 extends LinearOpMode {

    private CRServo revolver;
    // Start with 0.8 seconds as a guess for 120 degrees
    private double testStepTime = 0.8;

    @Override
    public void runOpMode() {
        revolver = hardwareMap.get(CRServo.class, "revolver");
        revolver.setDirection(CRServo.Direction.FORWARD);

        telemetry.addLine("REVOLVER CALIBRATION");
        telemetry.addLine("Press A to test a 120 degree turn");
        telemetry.addLine("Use D-Pad Up/Down to change timing");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Adjust timing by 0.01 seconds
            if (gamepad1.dpad_up) {
                testStepTime += 0.001;
                sleep(100);
            } else if (gamepad1.dpad_down) {
                testStepTime -= 0.001;
                sleep(100);
            }

            // Test the turn
            if (gamepad1.a) {
                revolver.setPower(1.0);
                sleep((long)(testStepTime * 1000));
                revolver.setPower(0.0);
            }

            telemetry.addData("Current Step Time", "%.3f seconds", testStepTime);
            telemetry.addLine("\nHow to calibrate:");
            telemetry.addLine("1. Line up Chamber 1 with the kicker.");
            telemetry.addLine("2. Press A.");
            telemetry.addLine("3. If it undershoots, increase time with D-Pad Up.");
            telemetry.addLine("4. If it overshoots, decrease time with D-Pad Down.");
            telemetry.update();
        }
    }
}