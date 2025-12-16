package org.firstinspires.ftc.teamcode.util.controlcommands;

import com.arcrobotics.ftclib.command.Command;

import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;

public class ActuatorCommands {
    private final MecanumDriveSubsystem mecanumDriveSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    public ActuatorCommands(MecanumDriveSubsystem mecanumDriveSubsystem, IntakeSubsystem intakeSubsystem) {
        this.mecanumDriveSubsystem = mecanumDriveSubsystem;
        this.intakeSubsystem = intakeSubsystem;

    }

    // Commands here /ᐠ｡ꞈ｡ᐟ\
    public Command startIntake() {
        return this.intakeSubsystem.takeTheBall();
    }

    public Command stopIntake() {
        return this.intakeSubsystem.stopIntake();
    }
}
