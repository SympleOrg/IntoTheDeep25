package org.firstinspires.ftc.teamcode.subsystems.elevator;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class ElevatorScoreOnChamberCommand extends ElevatorGoToPositionCommand {
    private final ElevatorSubsystem subsystem;

    public ElevatorScoreOnChamberCommand(ElevatorSubsystem subsystem) {
        super(subsystem, 0);
        addRequirements(subsystem);

        this.subsystem = subsystem;
    }

    @Override
    public void initialize() {
        final RobotConstants.ElevatorConstants.ElevatorState state = (this.subsystem.getState() != RobotConstants.ElevatorConstants.ElevatorState.SCORE_TOP
                && this.subsystem.getState() != RobotConstants.ElevatorConstants.ElevatorState.SCORE_BOTTOM)
                ? RobotConstants.ElevatorConstants.ElevatorState.SCORE_TOP
                : this.subsystem.getState();

        this.meters = state.getMeters() + RobotConstants.ElevatorConstants.SCORE_OFFSET;
        super.initialize();
    }
}
