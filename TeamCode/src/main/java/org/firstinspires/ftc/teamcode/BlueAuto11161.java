package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "BLUE - DECODE 11161", group = "Blue")
public class BlueAuto11161 extends LinearOpMode {

    // --- Hardware ---
    private com.qualcomm.robotcore.hardware.DcMotorEx frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private com.qualcomm.robotcore.hardware.DcMotorEx spinnerMotor, intake, conveyance;
    private Servo rampServo, kicker;
    private CRServo revolver;

    // --- Vision ---
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // --- CALIBRATION CONSTANTS ---
    static final double TICKS_PER_INCH = 60.0;
    static final double TICKS_PER_DEGREE = 10.5;
    static final double REVOLVER_STEP_TIME = 0.515;
    static final double KICKER_REST_POS = 0.7;
    static final double KICKER_UP_POS   = 0.3;

    // --- BLUE SPECIFIC CONSTANTS ---
    static final int TARGET_GOAL_ID = 24;     // Blue Goal ID
    static final double DESIRED_DISTANCE = 40.0; // Blue Distance Requirement

    // --- Auto Align Gains ---
    final double SPEED_GAIN  =  0.02;
    final double STRAFE_GAIN =  0.015;
    final double TURN_GAIN   =  0.01;

    @Override
    public void runOpMode() {
        initHardware();
        initAprilTag();

        // Simple telemetry to confirm we are in BLUE mode
        while (opModeInInit()) {
            telemetry.addData("ALLIANCE", "BLUE");
            telemetry.addData("TARGET GOAL", TARGET_GOAL_ID);
            telemetry.addData("DISTANCE", DESIRED_DISTANCE);
            telemetryAprilTag();
            telemetry.update();
        }

        waitForStart();



        // 1. Drive forward 60 inches
        driveStraight(0.5, 60.0);
        sleep(500);
        // 2. READ OBELISK (DECODE Tags 21, 22, or 23)
        List<AprilTagDetection> detections = new ArrayList<>();
        int detectedObeliskID = 0;
        do {
            detections = aprilTag.getDetections();
        } while (detections.isEmpty());

        // 3. Turn Counter-Clockwise 38 Degrees to find Blue Goal
        turnRobot(0.4, 38);

        driveStraight(0.5, 48);

        revolver.setPower(0.0);
        kicker.setPosition(KICKER_REST_POS);

        telemetry.addData("DETECTED ID: ", detections.get(0).id);
        telemetry.update();

        switch(detections.get(0).id) {

            case 22:
                // PGP
                rotateRevolverSteps(1);
                shootSingleBall();
                rotateRevolverSteps(2);
                shootSingleBall();
                rotateRevolverSteps(2);
                shootSingleBall();
                break;
            case 23:
                // P-P-G
                rotateRevolverSteps(1);
                shootSingleBall();
                rotateRevolverSteps(1);
                shootSingleBall();
                rotateRevolverSteps(1);
                shootSingleBall();
                break;
            case 21:
                // GPP
                shootSingleBall();
                rotateRevolverSteps(1);
                shootSingleBall();
                rotateRevolverSteps(1);
                shootSingleBall();
                break;
        }

        driveStraight(0.5, -10);

        visionPortal.close();
    }

    // --- Helper Methods ---

    private void executeFullSequence(int startSteps, int nextSteps, int lastSteps) {
        if (startSteps == 0) rotateRevolverSteps(startSteps);
        shootSingleBall();
        rotateRevolverSteps(nextSteps);
        shootSingleBall();
        rotateRevolverSteps(lastSteps);
        shootSingleBall();
    }

    private void shootSingleBall() {

        kicker.setPosition(KICKER_UP_POS);
        sleep(600);
        kicker.setPosition(KICKER_REST_POS);
        sleep(400);
    }

    public void rotateRevolverSteps(int steps) {
        revolver.setPower(1.0);
        sleep((long)(REVOLVER_STEP_TIME * steps * 1000));
        revolver.setPower(0.0);
        sleep(300);

    }

    public void turnRobot(double power, double degrees) {
        int target = (int)(degrees * TICKS_PER_DEGREE);
        frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() - target);
        backLeftDrive.setTargetPosition(backLeftDrive.getCurrentPosition() - target);
        frontRightDrive.setTargetPosition(frontRightDrive.getCurrentPosition() + target);
        backRightDrive.setTargetPosition(backRightDrive.getCurrentPosition() + target);

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
        moveRobot(0, 0, power);
        while (opModeIsActive() && frontLeftDrive.isBusy()) { idle(); }
        moveRobot(0, 0, 0);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveRobot(double x, double y, double yaw) {
        frontLeftDrive.setPower(x + y + yaw);
        frontRightDrive.setPower(x - y - yaw);
        backLeftDrive.setPower(x - y + yaw);
        backRightDrive.setPower(x + y - yaw);
    }

    public void driveStraight(double power, double inches) {
        int target = (int)(inches * TICKS_PER_INCH);
        frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() + target);
        frontRightDrive.setTargetPosition(frontRightDrive.getCurrentPosition() + target);
        backLeftDrive.setTargetPosition(backLeftDrive.getCurrentPosition() + target);
        backRightDrive.setTargetPosition(backRightDrive.getCurrentPosition() + target);

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
        moveRobot(power, 0, 0);
        while (opModeIsActive() && frontLeftDrive.isBusy()) { idle(); }
        moveRobot(0, 0, 0);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void initHardware() {
        frontLeftDrive  = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotorEx.class, "front_left_drive");
        backLeftDrive   = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotorEx.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotorEx.class, "front_right_drive");
        backRightDrive  = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotorEx.class, "back_right_drive");
        spinnerMotor    = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotorEx.class, "spinner_motor");
        intake          = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotorEx.class, "intake");
        conveyance      = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotorEx.class, "conveyance");
        rampServo       = hardwareMap.get(Servo.class, "ramp_servo");
        revolver        = hardwareMap.get(CRServo.class, "revolver");
        kicker          = hardwareMap.get(Servo.class, "kicker");

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        kicker.setDirection(Servo.Direction.FORWARD);
        revolver.setDirection(CRServo.Direction.FORWARD);

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void initAprilTag() {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "webcam1"), aprilTag);
    }

    private void setDriveMode(DcMotor.RunMode mode) {
        frontLeftDrive.setMode(mode); frontRightDrive.setMode(mode);
        backLeftDrive.setMode(mode);  backRightDrive.setMode(mode);
    }

    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            telemetry.addLine(String.format("ID %d (%s)", detection.id, detection.metadata != null ? detection.metadata.name : "Unknown"));
        }
    }
}