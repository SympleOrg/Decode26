package org.firstinspires.ftc.teamcode.util.controlcommands;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.RobotConstants.GateConstants;
import org.firstinspires.ftc.teamcode.RobotConstants.StorageConstants;
import org.firstinspires.ftc.teamcode.RobotConstants.ShooterConstants;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.gate.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.storage.StorageSubsystem;

public class ActuatorCommands {
    private final MecanumDriveSubsystem mecanumDriveSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final GateSubsystem gateSubsystem;
    private final StorageSubsystem storageSubsystem;
    private final ShooterSubsystem shooterSubsystem;

    public ActuatorCommands(
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

    // Commands here /ᐠ｡ꞈ｡ᐟ\
    public Command startIntake() {
        return this.intakeSubsystem.takeTheBall();
    }

    public Command stopIntake() {
        return this.intakeSubsystem.stopIntake();
    }

    public Command pushTheBall() {
        return this.gateSubsystem.goToState(GateConstants.GateState.PUSH);
    }

    public Command returnToZero() {
        return this.gateSubsystem.goToState(GateConstants.GateState.ZERO);
    }

    public Command storageGoToShooter() {
        return new ParallelCommandGroup(
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
        );
    }

    public Command storageGoToIntake() {
        return new ParallelCommandGroup(
                this.intakeSubsystem.stopIntake(),
                this.storageSubsystem.goToState(StorageConstants.StorageState.INTAKE)
        );
    }

    public Runnable toggleShooterMode() {
        return () -> this.shooterSubsystem.toggleStates(
                RobotConstants.ShooterConstants.ShooterState.IDLE,
                RobotConstants.ShooterConstants.ShooterState.SHOOT
        );
    }
}