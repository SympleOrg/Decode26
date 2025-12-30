package org.firstinspires.ftc.teamcode.util.controlcommands;

import com.arcrobotics.ftclib.command.Command;

import org.firstinspires.ftc.teamcode.RobotConstants.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;

public class ActuatorCommands {
    private final MecanumDriveSubsystem mecanumDriveSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ShooterSubsystem shooterSubsystem;

    public ActuatorCommands(MecanumDriveSubsystem mecanumDriveSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
        this.mecanumDriveSubsystem = mecanumDriveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
    }

    // Commands here /ᐠ｡ꞈ｡ᐟ\
    public Command startIntake() {
        return this.intakeSubsystem.takeTheBall();
    }

    public Command stopIntake() {
        return this.intakeSubsystem.stopIntake();
    }

    public Runnable toggleShooterMode() {
        return () -> this.shooterSubsystem.toggleStates(
                ShooterConstants.ShooterState.IDLE,
                ShooterConstants.ShooterState.SHOOT
        ).schedule();
    }
}
