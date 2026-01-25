package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

@Autonomous(name = "Basic Auto - DECODE 11161", group = "Robot")
public class BasicAuto_11161 extends LinearOpMode {

    // --- Hardware ---
    private com.qualcomm.robotcore.hardware.DcMotorEx frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private com.qualcomm.robotcore.hardware.DcMotorEx spinnerMotor, intake, conveyance;
    private Servo rampServo, kicker;
    private CRServo revolver;

    // --- Vision ---
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // --- CALIBRATION CONSTANTS ---
    static final double TICKS_PER_INCH = 45.0;
    static final double TICKS_PER_DEGREE = 10.5;
    static final double REVOLVER_STEP_TIME = 0.515;
    static final double KICKER_REST_POS = 0.7;
    static final double KICKER_UP_POS   = 0.2;

    // --- Auto Align Constants ---
    final double SPEED_GAIN  =  0.02;
    final double STRAFE_GAIN =  0.015;
    final double TURN_GAIN   =  0.01;

    // --- Team Selection Variables ---
    private boolean isBlueTeam = true; // Default to Blue
    private int targetGoalID = 20;     // Blue Goal ID
    private double desiredDistance = 40.0; // Blue Distance Requirement

    @Override
    public void runOpMode() {
        initHardware();
        initAprilTag();

        // --- TEAM SELECTION MENU (During Init) ---
        while (opModeInInit()) {
            if (gamepad1.x) isBlueTeam = true;  // X for Blue
            if (gamepad1.b) isBlueTeam = false; // B for Red

            if (isBlueTeam) {
                targetGoalID = 20;
                desiredDistance = 40.0;
                telemetry.addData("TEAM SELECTED", "BLUE (Goal 20, Dist 40)");
            } else {
                targetGoalID = 24;
                desiredDistance = 45.0;
                telemetry.addData("TEAM SELECTED", "RED (Goal 24, Dist 45)");
            }

            telemetry.addLine("\nPress X for Blue / B for Red");
            telemetryAprilTag();
            telemetry.update();
        }

        waitForStart();

        // 1. READ OBELISK (DECODE Tags 21, 22, or 23)
        int detectedObeliskID = -1;
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (!detections.isEmpty()) {
            detectedObeliskID = detections.get(0).id;
        }

        // 2. Drive forward 30 inches
        driveStraight(0.5, 30.0);

        // 3. Turn Counter-Clockwise 45 Degrees
        // Note: For Red team, you might need to turn Clockwise (-45)
        // depending on your starting position.
        turnRobot(0.4, 45);

        // 4. Drive to Assigned Goal (ID 20 or 24)
        long searchTime = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - searchTime < 4000)) {
            AprilTagDetection targetTag = null;
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            for (AprilTagDetection detection : currentDetections) {
                if (detection.id == targetGoalID) {
                    targetTag = detection;
                    break;
                }
            }

            if (targetTag != null) {
                double rangeError = (targetTag.ftcPose.range - desiredDistance);
                double headingError = targetTag.ftcPose.bearing;
                double yawError = targetTag.ftcPose.yaw;

                if (Math.abs(rangeError) < 1.5) break;

                double drive  = Range.clip(rangeError * SPEED_GAIN, -0.5, 0.5);
                double strafe = Range.clip(-headingError * STRAFE_GAIN, -0.4, 0.4);
                double turn   = Range.clip(yawError * TURN_GAIN, -0.3, 0.3);

                moveRobot(drive, strafe, turn);
            } else {
                moveRobot(0, 0, 0);
            }
        }
        moveRobot(0,0,0);

        // 5. EXECUTE PLAYBOOK
        if (detectedObeliskID == 21) {
            executeFullSequence(0, 1, 1); // G-P-P
        } else if (detectedObeliskID == 22) {
            executeFullSequence(1, 2, 2); // P-G-P
        } else if (detectedObeliskID == 23) {
            executeFullSequence(1, 1, 1); // P-P-G
        } else {
            shootSingleBall();
        }

        // 6. Finish
        driveStraight(0.5, -10.0);
        visionPortal.close();
    }

    // ... (Keep your helper methods exactly as they are below) ...

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