package org.firstinspires.ftc.teamcode.opMode.auto;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoRobotController;
import org.firstinspires.ftc.teamcode.trajectories.Trajectories;
import org.firstinspires.ftc.teamcode.util.opModes.SympleCommandOpMode;

@Autonomous(name = "Chamber auto", group = "auto")
public class ChamberAutoOpMode extends SympleCommandOpMode {
    @Override
    public void initialize() {
        this.robotController = new AutoRobotController.Builder()
                .initializeDefaults(this)
                .setPath(Trajectories.Paths.CHAMBER)
                .setStartingPose(new Pose2d(0, 0, Rotation2d.fromDegrees(180)))
                .build();
    }
}
