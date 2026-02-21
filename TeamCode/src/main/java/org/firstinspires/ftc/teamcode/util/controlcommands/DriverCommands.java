package org.firstinspires.ftc.teamcode.util.controlcommands;

import org.firstinspires.ftc.teamcode.RobotConstants.DriveConstants;
import org.firstinspires.ftc.teamcode.managers.RobotPositionManager;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.storage.StorageSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretSubsystem;

public class DriverCommands {
    private final MecanumDriveSubsystem mecanumDriveSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final StorageSubsystem storageSubsystem;
    private final TurretSubsystem turretSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;

    public DriverCommands(
            MecanumDriveSubsystem mecanumDriveSubsystem,
            IntakeSubsystem intakeSubsystem,
            StorageSubsystem storageSubsystem,
            TurretSubsystem turretSubsystem,
            ElevatorSubsystem elevatorSubsystem
    ) {
        this.mecanumDriveSubsystem = mecanumDriveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.storageSubsystem = storageSubsystem;
        this.turretSubsystem = turretSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
    }

    // Commands here ✍️(◔◡◔)

    public void setSlowSpeedMode() {
        this.mecanumDriveSubsystem.setDriveSpeedModifier(DriveConstants.DriveSpeed.SLOW);
    }

    public void setNormalSpeedMode() {
        this.mecanumDriveSubsystem.setDriveSpeedModifier(DriveConstants.DriveSpeed.NORMAL);
    }

    public void resetRobotAngle() {
        RobotPositionManager.getInstance().resetHeading();
    }
}
