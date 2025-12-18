package org.firstinspires.ftc.teamcode.util.controlcommands;

import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.gate.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;

public class DriverCommands {
    private final MecanumDriveSubsystem mecanumDriveSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final GateSubsystem gateSubsystem;

    public DriverCommands(MecanumDriveSubsystem mecanumDriveSubsystem, IntakeSubsystem intakeSubsystem, GateSubsystem gateSubsystem) {
        this.mecanumDriveSubsystem = mecanumDriveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.gateSubsystem = gateSubsystem;
    }

    // Commands here ✍️(◔◡◔)
}
