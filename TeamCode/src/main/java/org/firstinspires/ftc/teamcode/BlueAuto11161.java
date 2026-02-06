package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "BLUE SIMPLE 11161", group = "Blue")
public class BlueAuto11161 extends LinearOpMode {

    /* Hardware */
    private DcMotorEx frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotorEx spinnerMotor;
    private Servo rampServo, kicker;
    private CRServo revolver;

    /* Constants */
    static final double REVOLVER_STEP_TIME = 0.540;
    static final double KICKER_REST_POS    = 0.7;
    static final double KICKER_UP_POS      = 0.3;
    static final double SPINNER_VELOCITY   = 1500;
    static final double RAMP_POS_60        = 0.20;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        initHardware();
        initAprilTag();

        while (opModeInInit()) {
            telemetry.addData("Status", "Ready - BLUE");
            telemetryAprilTag();
            telemetry.update();
        }

        waitForStart();

        // 1. READ OBELISK (Capture ID immediately)
        int detectedId = -1;
        long scanStartTime = System.currentTimeMillis();
        while (opModeIsActive() && detectedId == -1 && (System.currentTimeMillis() - scanStartTime < 4000)) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.id == 21 || detection.id == 22 || detection.id == 23) {
                    detectedId = detection.id;
                    break;
                }
            }
        }

        telemetry.addData("Memory Saved ID", detectedId);
        telemetry.update();

        // 2. DRIVE FORWARD (Power, Seconds)
        // Adjust time to replace the 60-inch move
        driveTime(0.5, 4.0);
        sleep(200);

        // 3. TURN (Power, Seconds) - Positive power turns CCW (Left)
        // Adjust time to replace 45-degree turn
        turnTime(0.4, 1.0);
        sleep(200);

        // 4. PREPARE FOR SHOOTING
        rampServo.setPosition(RAMP_POS_60);
        spinnerMotor.setVelocity(SPINNER_VELOCITY);
        sleep(1500); // Wait for spin-up

        // 5. EXECUTE SEQUENCE BASED ON MEMORY
        if (detectedId == 21) {
            // Pattern for ID 21: G-P-P
            shootSingleBall();         // Green
            rotateRevolverSteps(1);    // To Pos 2
            shootSingleBall();         // Purple
            rotateRevolverSteps(1);    // To Pos 3
            shootSingleBall();         // Purple
        }
        else if (detectedId == 22) {
            // Pattern for ID 22: P-G-P
            rotateRevolverSteps(1);    // To Pos 2 (Purple)
            shootSingleBall();
            rotateRevolverSteps(2);    // To Pos 1 (Green)
            shootSingleBall();
            rotateRevolverSteps(2);    // To Pos 3 (Purple)
            shootSingleBall();
        }
        else if (detectedId == 23) {
            // Pattern for ID 23: P-P-G
            rotateRevolverSteps(1);    // To Pos 2 (Purple)
            shootSingleBall();
            rotateRevolverSteps(1);    // To Pos 3 (Purple)
            shootSingleBall();
            rotateRevolverSteps(1);    // To Pos 1 (Green)
            shootSingleBall();
        }
        else {
            // Fallback if nothing seen: Just shoot what is in front
            shootSingleBall();
        }

        spinnerMotor.setVelocity(0);
        visionPortal.close();
    }

    /* Simple Movement Methods */

    public void driveTime(double power, double seconds) {
        setDrivePower(power, power, power, power);
        sleep((long)(seconds * 1000));
        setDrivePower(0, 0, 0, 0);
    }

    public void turnTime(double power, double seconds) {
        // Turning Left (CCW): Left motors back, Right motors forward
        setDrivePower(-power, -power, power, power);
        sleep((long)(seconds * 1000));
        setDrivePower(0, 0, 0, 0);
    }

    private void setDrivePower(double fl, double bl, double fr, double br) {
        frontLeftDrive.setPower(fl);
        backLeftDrive.setPower(bl);
        frontRightDrive.setPower(fr);
        backRightDrive.setPower(br);
    }

    /* Shooting Methods */

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

    private void initHardware() {
        frontLeftDrive  = hardwareMap.get(DcMotorEx.class, "front_left_drive");
        backLeftDrive   = hardwareMap.get(DcMotorEx.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        backRightDrive  = hardwareMap.get(DcMotorEx.class, "back_right_drive");
        spinnerMotor    = hardwareMap.get(DcMotorEx.class, "spinner_motor");
        rampServo       = hardwareMap.get(Servo.class, "ramp_servo");
        revolver        = hardwareMap.get(CRServo.class, "revolver");
        kicker          = hardwareMap.get(Servo.class, "kicker");

        // Directions
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Reset all motor modes to simple power mode
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        spinnerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void initAprilTag() {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        // Uses the correct WebcamName.class to avoid the previous error
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "webcam1"),
                aprilTag
        );
    }

    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            telemetry.addLine(String.format("ID %d detected", detection.id));
        }
    }
}