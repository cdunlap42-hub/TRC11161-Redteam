package org.firstinspires.ftc.teamcode.messages;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public final class MecanumDriveMessage {
    public long timestamp;
    public double leftFrontPower;
    public double leftBackPower;
    public double rightBackPower;
    public double rightFrontPower;

    public MecanumDriveMessage(MecanumDrive drive) {
        this.timestamp = System.currentTimeMillis();
        this.leftFrontPower = drive.leftFront.getPower();
        this.leftBackPower = drive.leftBack.getPower();
        this.rightBackPower = drive.rightBack.getPower();
        this.rightFrontPower = drive.rightFront.getPower();
    }
}
