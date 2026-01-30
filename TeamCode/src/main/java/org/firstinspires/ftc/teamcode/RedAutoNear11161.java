package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

// Added this import to fix the VisionPortal error
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;
import java.util.ArrayList;

@Autonomous(name = "Red Auto Near 11161", group = "Autonomous")
public class RedAutoNear11161 extends LinearOpMode {

    /* Hardware Members */
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotorEx spinnerMotor;
    private CRServo revolver;
    private Servo kicker, rampServo;

    /* Constants */
    private static final double KICKER_REST_POS = 0.7;
    private static final double KICKER_UP_POS   = 0.4;
    private static final double RAMP_POS_60     = 0.6;
    private static final int SPINNER_VELOCITY   = 1400;

    static final double COUNTS_PER_INCH = 45.0;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        initHardware();
        initAprilTag();

        while (opModeInInit()) {
            telemetry.addLine("RED AUTO NEAR - Ready");
            displayAprilTagTelemetry();
            telemetry.update();
        }

        waitForStart();

        // 1. BACK UP 40 INCHES
        driveStraight(0.5, -40.0);
        sleep(500);

        // 2. READ OBELISK
        List<AprilTagDetection> detections;
        long scanStartTime = System.currentTimeMillis();
        do {
            detections = aprilTag.getDetections();
            if (System.currentTimeMillis() - scanStartTime > 3000) break;
        } while (detections.isEmpty() && opModeIsActive());

        // 3. TURN 38 DEGREES CLOCKWISE
        turnRobot(0.4, -38);
        sleep(200);

        // 4. PREPARE FOR SHOOTING
        rampServo.setPosition(RAMP_POS_60);
        spinnerMotor.setVelocity(SPINNER_VELOCITY);

        driveStraight(0.5, 48);
        sleep(1000);

        // 5. EXECUTE PATTERN
        if (!detections.isEmpty()) {
            int tagId = detections.get(0).id;
            switch (tagId) {
                case 21: executeFullSequence(0, 1, 1); break; // G-P-P
                case 22: executeFullSequence(1, 2, 2); break; // P-G-P
                case 23: executeFullSequence(1, 1, 1); break; // P-P-G
            }
        }

        spinnerMotor.setVelocity(0);
        visionPortal.close();
    }

    private void executeFullSequence(int startSteps, int nextSteps, int lastSteps) {
        if (startSteps > 0) rotateRevolverSteps(startSteps);
        shootSingleBall();
        rotateRevolverSteps(nextSteps);
        shootSingleBall();
        rotateRevolverSteps(lastSteps);
        shootSingleBall();
    }

    private void shootSingleBall() {
        kicker.setPosition(KICKER_UP_POS);
        sleep(400);
        kicker.setPosition(KICKER_REST_POS);
        sleep(400);
    }

    private void rotateRevolverSteps(int steps) {
        revolver.setPower(0.5);
        sleep(450 * steps);
        revolver.setPower(0);
        sleep(200);
    }

    private void driveStraight(double speed, double inches) {
        int moveCounts = (int)(inches * COUNTS_PER_INCH);
        // Set targets for ALL 4 motors
        frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() + moveCounts);
        frontRightDrive.setTargetPosition(frontRightDrive.getCurrentPosition() + moveCounts);
        backLeftDrive.setTargetPosition(backLeftDrive.getCurrentPosition() + moveCounts);
        backRightDrive.setTargetPosition(backRightDrive.getCurrentPosition() + moveCounts);

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
        setDrivePower(speed);

        while (opModeIsActive() && frontLeftDrive.isBusy()) { idle(); }
        setDrivePower(0);
    }

    private void turnRobot(double speed, double degrees) {
        int turnCounts = (int)(degrees * 10);
        frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() - turnCounts);
        backLeftDrive.setTargetPosition(backLeftDrive.getCurrentPosition() - turnCounts);
        frontRightDrive.setTargetPosition(frontRightDrive.getCurrentPosition() + turnCounts);
        backRightDrive.setTargetPosition(backRightDrive.getCurrentPosition() + turnCounts);

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
        setDrivePower(speed);
        while (opModeIsActive() && frontLeftDrive.isBusy()) { idle(); }
        setDrivePower(0);
    }

    private void initHardware() {
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "back_right_drive");

        spinnerMotor = hardwareMap.get(DcMotorEx.class, "spinner_motor");
        revolver     = hardwareMap.get(CRServo.class, "revolver");
        kicker       = hardwareMap.get(Servo.class, "kicker");
        rampServo    = hardwareMap.get(Servo.class, "ramp_servo");

        spinnerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinnerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
    }

    private void setDrivePower(double p) {
        frontLeftDrive.setPower(p); frontRightDrive.setPower(p);
        backLeftDrive.setPower(p); backRightDrive.setPower(p);
    }

    private void setDriveMode(DcMotor.RunMode m) {
        frontLeftDrive.setMode(m); frontRightDrive.setMode(m);
        backLeftDrive.setMode(m); backRightDrive.setMode(m);
    }

    private void initAprilTag() {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        // FIXED: Using WebcamName.class to resolve the build error
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                aprilTag
        );
    }

    private void displayAprilTagTelemetry() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            telemetry.addData("Detected ID", detection.id);
        }
    }
}