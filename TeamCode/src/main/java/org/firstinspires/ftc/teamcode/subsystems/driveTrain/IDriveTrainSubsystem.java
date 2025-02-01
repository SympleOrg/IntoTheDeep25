package org.firstinspires.ftc.teamcode.subsystems.driveTrain;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.util.LoggerSubsystem;

public interface IDriveTrainSubsystem extends Subsystem, LoggerSubsystem {
    void moveSideMotors(double left, double right);
    Pose2d getPosition();
}
