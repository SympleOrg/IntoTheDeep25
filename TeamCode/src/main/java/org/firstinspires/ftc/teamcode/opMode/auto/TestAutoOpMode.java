package org.firstinspires.ftc.teamcode.opMode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoRobotController;
import org.firstinspires.ftc.teamcode.trajectories.Trajectories;
import org.firstinspires.ftc.teamcode.util.opModes.SympleCommandOpMode;

@Autonomous(name = "TEST auto", group = "auto")
public class TestAutoOpMode extends SympleCommandOpMode {
    @Override
    public void initialize() {
        this.robotController = new AutoRobotController.Builder()
                .initializeDefaults(this)
                .setTrajectory(Trajectories.TEST)
                .build();
    }
}

