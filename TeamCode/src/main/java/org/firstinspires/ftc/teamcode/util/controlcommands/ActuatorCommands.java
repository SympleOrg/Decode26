package org.firstinspires.ftc.teamcode.util.controlcommands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.RobotConstants.StorageConstants;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.storage.StorageSubsystem;

public class ActuatorCommands {
    private final MecanumDriveSubsystem mecanumDriveSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final StorageSubsystem storageSubsystem;

    public ActuatorCommands(MecanumDriveSubsystem mecanumDriveSubsystem, IntakeSubsystem intakeSubsystem, StorageSubsystem storageSubsystem) {
        this.mecanumDriveSubsystem = mecanumDriveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.storageSubsystem = storageSubsystem;
    }

    // Commands here /ᐠ｡ꞈ｡ᐟ\
    public Command startIntake() {
        return this.intakeSubsystem.takeTheBall();
    }

    public Command stopIntake() {
        return this.intakeSubsystem.stopIntake();
    }

    public void storageGoToShooter() {
        new ParallelCommandGroup(
                this.intakeSubsystem.takeTheBall(),
                new SequentialCommandGroup(
                        new WaitCommand(200),
                        new ParallelCommandGroup(
                                this.storageSubsystem.goToState(StorageConstants.StorageState.SHOOTER),
                                new SequentialCommandGroup(
                                        new WaitCommand(2000),
                                        this.intakeSubsystem.stopIntake().asProxy()
                                )
                        )
                )
        ).schedule();

    }

    public void storageGoToIntake() {
        new ParallelCommandGroup(
                this.intakeSubsystem.stopIntake(),
                this.storageSubsystem.goToState(StorageConstants.StorageState.INTAKE)
        ).schedule();
    }
}
