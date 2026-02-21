package org.firstinspires.ftc.teamcode.subsystems.elevator;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.teamcode.RobotConstants.ElevatorConstants;

public class HoldPositionElevatorCommand extends CommandBase {
    private final ElevatorSubsystem elevatorSubsystem;
    private final double setPoint;

    private PIDController pidController;

    public HoldPositionElevatorCommand(ElevatorSubsystem elevatorSubsystem, double setPoint) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.setPoint = setPoint;

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        this.pidController = new PIDController(ElevatorConstants.Kp, ElevatorConstants.Ki, ElevatorConstants.Kd);
        this.pidController.setSetPoint(this.setPoint);
        this.pidController.setTolerance(0.01);
    }

    @Override
    public void execute() {
        double power = this.pidController.calculate(this.elevatorSubsystem.getCurrentPosition());

        this.elevatorSubsystem.moveMotor(power);
    }

    @Override
    public boolean isFinished() {
        return this.pidController.atSetPoint();
    }
}
