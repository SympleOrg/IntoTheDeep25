package org.firstinspires.ftc.teamcode.opMode.auto;

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
                .setTrajectory(Trajectories.THREE_GAME_PIECE)
                .build();
    }
}
