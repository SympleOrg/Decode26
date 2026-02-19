package org.firstinspires.ftc.teamcode.util.controlcommands;

import com.seattlesolvers.solverslib.command.Command;

import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.storage.StorageSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretSubsystem;

public class ActuatorCommands {
    private final MecanumDriveSubsystem mecanumDriveSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final StorageSubsystem storageSubsystem;
    private final TurretSubsystem turretSubsystem;

    public ActuatorCommands(
            MecanumDriveSubsystem mecanumDriveSubsystem,
            IntakeSubsystem intakeSubsystem,
            StorageSubsystem gateSubsystem,
            TurretSubsystem turretSubsystem
    ) {
        this.mecanumDriveSubsystem = mecanumDriveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.storageSubsystem = gateSubsystem;
        this.turretSubsystem = turretSubsystem;
    }

    // Commands here /ᐠ｡ꞈ｡ᐟ\
    public Command startIntake() {
        return this.intakeSubsystem.takeTheBall();
    }

    public Command stopIntake() {
        return this.intakeSubsystem.stopIntake();
    }
}