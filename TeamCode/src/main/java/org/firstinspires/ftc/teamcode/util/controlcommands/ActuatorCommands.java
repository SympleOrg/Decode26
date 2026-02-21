package org.firstinspires.ftc.teamcode.util.controlcommands;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.RobotConstants.ElevatorConstants;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.storage.StorageSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretSubsystem;
import org.firstinspires.ftc.teamcode.util.BallColor;

public class ActuatorCommands {
    private final MecanumDriveSubsystem mecanumDriveSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final StorageSubsystem storageSubsystem;
    private final TurretSubsystem turretSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;

    public ActuatorCommands(
            MecanumDriveSubsystem mecanumDriveSubsystem,
            IntakeSubsystem intakeSubsystem,
            StorageSubsystem gateSubsystem,
            TurretSubsystem turretSubsystem, ElevatorSubsystem elevatorSubsystem
    ) {
        this.mecanumDriveSubsystem = mecanumDriveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.storageSubsystem = gateSubsystem;
        this.turretSubsystem = turretSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
    }

    // Commands here /ᐠ｡ꞈ｡ᐟ\
    public Command startIntake() {
        return this.intakeSubsystem.takeTheBall();
    }

    public Command stopIntake() {
        return this.intakeSubsystem.stopIntake();
    }

    public Command shootBall() {
        return new ParallelCommandGroup(
                this.elevatorSubsystem.goToState(ElevatorConstants.ElevatorStates.SHOOTER),
                new InstantCommand(() -> this.storageSubsystem.setBall(2, BallColor.NONE, true))
        );
    }

    public Command reloadBall() {
        return new SequentialCommandGroup(
                this.elevatorSubsystem.goToState(ElevatorConstants.ElevatorStates.INTAKE),
                new InstantCommand(() -> this.storageSubsystem.setReloadStatus(true))
        );
    }
}