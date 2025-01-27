package org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PController;

import org.firstinspires.ftc.teamcode.subsystems.driveTrain.AutoableDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.IDriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.util.DataLogger;

public class RotateRobotByDegCommand extends CommandBase {
    public static final double DEFAULT_KP = 0.05;
    private static final double MAX_POWER = 1;

    private final PController pController;
    private final double degToRotate;

    private int timesDone = 0;
    private double STARTING_ANGLE;

    private final AutoableDriveTrain subsystem;

    public RotateRobotByDegCommand(AutoableDriveTrain subsystem, double degToRotate, double kp) {
        this.subsystem = subsystem;
        addRequirements(subsystem);

        this.pController = new PController(kp);
        this.pController.setTolerance(2.5);
        this.degToRotate = degToRotate;
    }

    public RotateRobotByDegCommand(AutoableDriveTrain driveBaseSubsystem, double degToRotate) {
        this(driveBaseSubsystem, degToRotate, DEFAULT_KP);
    }

    @Override
    public void initialize() {
        super.initialize();
        this.STARTING_ANGLE = this.subsystem.getPosition().getRotation().getDegrees();
        this.pController.setSetPoint(Math.IEEEremainder(degToRotate + STARTING_ANGLE, 360));
    }

    @Override
    public void execute() {
        double headingDist = this.subsystem.getPosition().getRotation().getDegrees();
        double distLeft = Math.IEEEremainder(this.pController.getSetPoint() - headingDist, 360);

        double rawPower = this.pController.calculate(this.pController.getSetPoint() - distLeft);

        double power = Math.min(Math.max(rawPower, -MAX_POWER), MAX_POWER);

        this.subsystem.moveSideMotors(power, -power);
    }

    @Override
    public void end(boolean interrupted) {
        this.subsystem.moveSideMotors(0, 0);
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        if (this.pController.atSetPoint()) {
            this.timesDone++;
        } else {
            this.timesDone = 0;
        }
        return this.timesDone > 5;
    }
}