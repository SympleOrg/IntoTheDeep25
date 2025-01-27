package org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands.mecanumDrive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem.MecanumChassisWheelsSet.MotorNames;
import org.firstinspires.ftc.teamcode.util.MecanumChassisUtils;

public class MecanumArcadeDriveCommand extends CommandBase {
    private final GamepadEx gamepad;

    private final MecanumDriveSubsystem subsystem;

    public MecanumArcadeDriveCommand(MecanumDriveSubsystem subsystem, GamepadEx gamepad) {
        this.subsystem = subsystem;
        this.gamepad = gamepad;

        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        double vSpeed = -this.gamepad.getLeftY();
        double hSpeed = -this.gamepad.getLeftX();
        double rotationSpeed = -this.gamepad.getRightX();

        Vector2d vector = new Vector2d(hSpeed, vSpeed);
        vector = vector.rotateBy(-this.subsystem.getHeading());

        MecanumChassisUtils.MecanumWheelSpeeds mecanumWheelSpeeds = MecanumChassisUtils.chassisSpeedToWheelSpeeds(vector, rotationSpeed);

        this.subsystem.moveMotors(mecanumWheelSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        this.subsystem.moveMotor(MecanumDriveSubsystem.MotorNames.FRONT_RIGHT, 0);
        this.subsystem.moveMotor(MecanumDriveSubsystem.MotorNames.BACK_RIGHT, 0);
        this.subsystem.moveMotor(MecanumDriveSubsystem.MotorNames.FRONT_LEFT, 0);
        this.subsystem.moveMotor(MecanumDriveSubsystem.MotorNames.BACK_LEFT, 0);
        super.end(interrupted);
    }
}
