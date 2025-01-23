package org.firstinspires.ftc.teamcode.subsystems.elevator;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class ElevatorGoToPositionCommand extends CommandBase {
    protected double meters;
    private final PIDController pidController = new PIDController(RobotConstants.ElevatorConstants.P, RobotConstants.ElevatorConstants.I, RobotConstants.ElevatorConstants.D);
    private final ElevatorSubsystem subsystem;

    public ElevatorGoToPositionCommand(ElevatorSubsystem subsystem, double meters) {
        addRequirements(subsystem);

        this.subsystem = subsystem;
        this.meters = meters;
    }

    @Override
    public void initialize() {
        pidController.reset();
        pidController.setTolerance(0.01);
        pidController.setSetPoint(meters);
    }

    @Override
    public void execute() {
        double power = pidController.calculate(this.subsystem.getCurrentPosition());
        this.subsystem.setPower(power);
    }

    @Override
    public boolean isFinished() {
        return this.pidController.atSetPoint();
    }
}
