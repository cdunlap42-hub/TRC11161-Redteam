package org.firstinspires.ftc.teamcode.messages;

import com.acmerobotics.roadrunner.PoseVelocity2d;

public final class DriveCommandMessage {
    public long timestamp;
    public double forwardVelocity;
    public double strafeVelocity;
    public double angularVelocity;

    public DriveCommandMessage(PoseVelocity2d vel) {
        this.timestamp = System.currentTimeMillis();
        this.forwardVelocity = vel.linearVel.x;
        this.strafeVelocity = vel.linearVel.y;
        this.angularVelocity = vel.angVel;
    }
}
