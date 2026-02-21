package org.firstinspires.ftc.teamcode.pedropathing;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.storage.StorageSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretSubsystem;
import org.firstinspires.ftc.teamcode.util.BallColor;

public class Trajectories {
    private final MecanumDriveSubsystem mecanumDriveSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final StorageSubsystem storageSubsystem;
    private final TurretSubsystem turretSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;

    public Trajectories(MecanumDriveSubsystem mecanumDriveSubsystem, IntakeSubsystem intakeSubsystem, StorageSubsystem storageSubsystem, TurretSubsystem turretSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this.mecanumDriveSubsystem = mecanumDriveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.storageSubsystem = storageSubsystem;
        this.turretSubsystem = turretSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
    }

    public Command createCloseRedTrajectory() {
        return new InstantCommand();
    }

    public Command createFarRedTrajectory() {
        return new InstantCommand();
    }

    public Command createCloseBlueTrajectory() {
        return new InstantCommand();
    }

    public Command createFarBlueTrajectory() {
        return new InstantCommand();
    }

    private Command startIntake() {
        return this.intakeSubsystem.takeTheBall();
    }

    private Command stopIntake() {
        return this.intakeSubsystem.stopIntake();
    }

    private Command shootBall() {
        return new ParallelCommandGroup(
                this.elevatorSubsystem.goToState(RobotConstants.ElevatorConstants.ElevatorStates.SHOOTER),
                new InstantCommand(() -> this.storageSubsystem.setBall(2, BallColor.NONE, true))
        );
    }

    private Command reloadBall() {
        return new SequentialCommandGroup(
                this.elevatorSubsystem.goToState(RobotConstants.ElevatorConstants.ElevatorStates.INTAKE),
                new InstantCommand(() -> this.storageSubsystem.setReloadStatus(true))
        );
    }
}
