package org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Transform2d;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.IDriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.util.DataLogger;

@Config
public class DriveDistanceDriveCommand extends CommandBase {
    public static double Kp = 2;
    public static double Ki = 0.8;
    public static double Kd = 0.25;
    public static double MAX_POWER = 0.75;

    private final PIDController pController;
    private final double finalPos;

    private Pose2d STARTING_POS;

    private final IDriveTrainSubsystem subsystem;

    public DriveDistanceDriveCommand(IDriveTrainSubsystem subsystem, double meters) {
        this.subsystem = subsystem;
        addRequirements(subsystem);

        this.finalPos = meters;

        this.pController = new PIDController(Kp, Ki, Kd);
        this.pController.setTolerance(0.02);
    }

    @Override
    public void initialize() {
        super.initialize();
        this.subsystem.getDataLogger().addData(DataLogger.DataType.INFO, "DriveDistanceCommand: " + "Moving " + this.finalPos + " meters");
        this.STARTING_POS = this.subsystem.getPosition();
        this.pController.reset();
        this.pController.setSetPoint(this.finalPos);
    }

    @Override
    public void execute() {
        super.execute();

        Transform2d driveDistance = this.subsystem.getPosition().minus(this.STARTING_POS);
        double dist = driveDistance.getTranslation().getNorm();
        if(finalPos < 0) {
            dist *= -1;
        }
        double rawPower = this.pController.calculate(dist);
        rawPower += Math.signum(rawPower) * RobotConstants.DriveConstants.Ks;

        double power = Math.min(Math.max(rawPower, -MAX_POWER), MAX_POWER);

        MultipleTelemetry telemetry = this.subsystem.getTelemetry();
        telemetry.addData("power", power);
        telemetry.addData("dist", driveDistance);
        telemetry.update();

        this.subsystem.moveSideMotors(-power, -power);
    }

    @Override
    public void end(boolean interrupted) {
        this.subsystem.moveSideMotors(0, 0);
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return this.pController.atSetPoint();
    }
}
