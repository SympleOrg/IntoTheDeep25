package org.firstinspires.ftc.teamcode.trajectories;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.AutoRobotController;

public class Test extends ITrajectory {
    public Test(AutoRobotController.SubsystemContainer subsystemContainer) {
        super(subsystemContainer);
    }

    @Override
    public Command getPath() {
        return new SequentialCommandGroup(
                        drive(7),
                        new WaitCommand(2000),
                        driveBackwards(7),
                        new WaitCommand(2000),
                        driveBackwards(50),
                        new WaitCommand(2000),
                        drive(50),
                        new WaitCommand(2000),
                        rotate(180),
                        new WaitCommand(2000),
                        rotate(30),
                        new WaitCommand(2000),
                        rotate(-30)
            );
    }
}
