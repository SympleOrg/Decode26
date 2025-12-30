package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants.ShooterConstants;
import org.firstinspires.ftc.teamcode.maps.MotorMap;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.subsystem.LoggerSubsystem;
import org.firstinspires.ftc.teamcode.util.subsystem.StateSubsystemBase;

@Config
public class ShooterSubsystem extends StateSubsystemBase<ShooterConstants.ShooterState> implements LoggerSubsystem {
    public static double MAX_VELOCITY_OFFSET = -100;

    private final MotorEx motorEx;
    private final DataLogger dataLogger;
    private final MultipleTelemetry telemetry;

    public ShooterSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry, DataLogger dataLogger) {
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

    private boolean isFastEnough() {
        double maxVelocity = (MotorMap.SHOOTER.getMaxRPM() / 60f) * MotorMap.SHOOTER.getTicksPerRev();
        return this.motorEx.getCorrectedVelocity() >= maxVelocity + MAX_VELOCITY_OFFSET;
    }

    @Override
    protected Command getChangeStateCommand(ShooterConstants.ShooterState state, Subsystem... requirements) {
        return new RunCommand(() -> this.moveMotor(state.getUnit()), mergeSubsystemLists(requirements, this));
    }

    @Override
    public MultipleTelemetry getTelemetry() {
        return this.telemetry;
    }

    @Override
    public DataLogger getDataLogger() {
        return this.dataLogger;
    }
}
