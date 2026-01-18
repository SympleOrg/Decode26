package org.firstinspires.ftc.teamcode.util.controlcommands;

import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.gate.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.storage.StorageSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;

public class DriverCommands {
    private final MecanumDriveSubsystem mecanumDriveSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final GateSubsystem gateSubsystem;
    private final StorageSubsystem storageSubsystem;
    private final ShooterSubsystem shooterSubsystem;

    public DriverCommands(
            MecanumDriveSubsystem mecanumDriveSubsystem,
            IntakeSubsystem intakeSubsystem,
            GateSubsystem gateSubsystem,
            StorageSubsystem storageSubsystem,
            ShooterSubsystem shooterSubsystem
    ) {
        this.mecanumDriveSubsystem = mecanumDriveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.gateSubsystem = gateSubsystem;
        this.storageSubsystem = storageSubsystem;
        this.shooterSubsystem = shooterSubsystem;
    }

    // Commands here ✍️(◔◡◔)
}
