package org.firstinspires.ftc.teamcode.subsystems.driveTrain;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveWheelSpeeds;

public interface AutoableDriveTrain extends Subsystem {
    Pose2d getPosition();
    DifferentialDriveWheelSpeeds getWheelSpeeds();
    void moveSideMotors(double left, double right);
}
