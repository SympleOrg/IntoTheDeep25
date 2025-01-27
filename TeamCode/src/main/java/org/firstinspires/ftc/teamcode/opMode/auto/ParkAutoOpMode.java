package org.firstinspires.ftc.teamcode.opMode.auto;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoRobotController;
import org.firstinspires.ftc.teamcode.trajectories.Trajectories;
import org.firstinspires.ftc.teamcode.util.opModes.SympleCommandOpMode;

@Autonomous(name = "Park auto", group = "auto")
public class ParkAutoOpMode extends SympleCommandOpMode {
    @Override
    public void initialize() {
        this.robotController = new AutoRobotController.Builder()
                .initializeDefaults(this)
                .setPath(Trajectories.Paths.PARK)
                .setStartingPose(new Pose2d(0.38, -1.56, Rotation2d.fromDegrees(0)))
                .build();
    }
}
