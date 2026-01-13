package org.firstinspires.ftc.teamcode.subsystems.storage;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.RobotConstants.StorageConstants;

public class MoveStorageCommand extends CommandBase {
    private final StorageSubsystem storageSubsystem;
    private final double point;

    private PIDController pidController;

    public MoveStorageCommand(StorageSubsystem storageSubsystem, double point) {
        this.storageSubsystem = storageSubsystem;
        this.point = point;

        addRequirements(storageSubsystem);
    }

    @Override
    public void initialize() {
        this.pidController = new PIDController(StorageConstants.kP, StorageConstants.kI, StorageConstants.kD);
        this.pidController.setSetPoint(this.point);
        this.pidController.setTolerance(1);
    }

    @Override
    public void execute() {
        double power = this.pidController.calculate(this.storageSubsystem.getCurrentPosition());

        this.storageSubsystem.moveMotor(power);
    }
}
