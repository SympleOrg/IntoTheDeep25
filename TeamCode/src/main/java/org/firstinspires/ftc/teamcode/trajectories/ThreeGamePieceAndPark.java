package org.firstinspires.ftc.teamcode.trajectories;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.AutoRobotController;
import org.firstinspires.ftc.teamcode.RobotConstants;

public class ThreeGamePieceAndPark extends ITrajectory {

    public ThreeGamePieceAndPark(AutoRobotController.SubsystemContainer subsystemContainer) {
        super(subsystemContainer);
    }

    @Override
    public Command getPath() {
        return new SequentialCommandGroup(
                this.subsystemContainer.getClawSubsystem().moveToState(RobotConstants.ClawConstants.ClawState.CLOSE),
                this.subsystemContainer.getElevatorSubsystem().goToState(RobotConstants.ElevatorConstants.ElevatorState.SCORE_TOP, subsystemContainer.getIntakePitchJointSubsystem()),
                strafe(180, 71), // fwd to submersable
                scoreOnChamber(),
                strafe(81.55, 90.3),
                strafe(175, 64),
                rotate(180),
                strafe(90, 27).withTimeout(2000),
                strafe(0, 86).withTimeout(2000), // Return element

                strafe(160, 35), // Go back and center
                strafe(0, 69), // go to collect // TODO CHANGE THIS
                waitForSpecimen(),
                strafe(0, 10).withTimeout(1500),
                takeFromHuman(),
                strafe(-110, 125),
                rotate(180),
                strafe(0, 50).withTimeout(1000),
                scoreOnChamber()
        );
    }
}

