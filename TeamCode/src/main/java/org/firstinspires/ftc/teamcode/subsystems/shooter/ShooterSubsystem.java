package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.bylazar.telemetry.JoinedTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.RobotConstants.ShooterConstants;
import org.firstinspires.ftc.teamcode.maps.MotorMap;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.subsystem.LoggerSubsystem;
import org.firstinspires.ftc.teamcode.util.subsystem.StateSubsystemBase;

public class ShooterSubsystem extends StateSubsystemBase<ShooterConstants.ShooterState> implements LoggerSubsystem {
    private final MotorEx motorEx;
    private final DataLogger dataLogger;
    private final JoinedTelemetry telemetry;

    public ShooterSubsystem(HardwareMap hardwareMap, JoinedTelemetry telemetry, DataLogger dataLogger) {
        super(ShooterConstants.ShooterState.OFF);
        this.motorEx = new MotorEx(hardwareMap, MotorMap.SHOOTER.getId());
        this.dataLogger = dataLogger;
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        telemetry.addData("Shooter State", this.getCurrentState().name());
        telemetry.addData("Shooter Vel", this.motorEx.getCorrectedVelocity());
        telemetry.addData("Is shooter fast enough?", this.isFastEnough());
    }

    private void moveMotor(double power) {
        this.motorEx.set(power);
    }

    public boolean isFastEnough() {
        return this.motorEx.getCorrectedVelocity() >= ShooterConstants.SHOOT_MIN_VEL;
    }

    @Override
    protected Command getChangeStateCommand(ShooterConstants.ShooterState state, Subsystem... requirements) {
        return new RunCommand(() -> this.moveMotor(state.getUnit()), mergeSubsystemLists(requirements, this));
    }

    @Override
    public JoinedTelemetry getTelemetry() {
        return this.telemetry;
    }

    @Override
    public DataLogger getDataLogger() {
        return this.dataLogger;
    }
}
