package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.bylazar.telemetry.JoinedTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.RobotConstants.ShooterConstants;
import org.firstinspires.ftc.teamcode.maps.MotorMap;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.subsystem.LoggerSubsystem;
import org.firstinspires.ftc.teamcode.util.subsystem.StateSubsystemBase;

public class ShooterSubsystem extends StateSubsystemBase<ShooterConstants.ShooterState> implements LoggerSubsystem {
    private final MotorEx motorEx;
    private final VoltageSensor voltageSensor;

    private final ElapsedTime timer;

    private final DataLogger dataLogger;
    private final JoinedTelemetry telemetry;

    public ShooterSubsystem(HardwareMap hardwareMap, JoinedTelemetry telemetry, DataLogger dataLogger) {
        super(ShooterConstants.ShooterState.OFF);
        this.motorEx = new MotorEx(hardwareMap, MotorMap.SHOOTER.getId(), MotorMap.SHOOTER.getTicksPerRev(), MotorMap.SHOOTER.getMaxRPM());
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();

        this.timer = new ElapsedTime();

        this.dataLogger = dataLogger;
        this.telemetry = telemetry;

        this.motorEx.setRunMode(Motor.RunMode.VelocityControl);
        this.motorEx.setVeloCoefficients(ShooterConstants.Kp, ShooterConstants.Ki, ShooterConstants.Kd);
        this.motorEx.setFeedforwardCoefficients(ShooterConstants.Ks, ShooterConstants.Kv, ShooterConstants.Ka);
    }

    @Override
    public void periodic() {
        telemetry.addData("Shooter State", this.getCurrentState().name());
        telemetry.addData("Shooter Vel", this.motorEx.getCorrectedVelocity());
        telemetry.addData("Is shooter fast enough?", this.isFastEnough());

        if (!this.innerIsFastEnough()) {
            this.timer.reset();
        }
    }

    private void setVelocity(double velocity) {
        double voltageModifier = 12f / this.voltageSensor.getVoltage();
        this.motorEx.setVelocity(velocity * voltageModifier);
    }

    private boolean innerIsFastEnough() {
        return this.motorEx.getCorrectedVelocity() >= ShooterConstants.ShooterState.SHOOT.getUnit() + ShooterConstants.SHOOT_OFFSET_VEL;
    }

    public boolean isFastEnough() {
        return this.innerIsFastEnough() && this.timer.milliseconds() >= ShooterConstants.SHOOTER_DELAY;
    }

    @Override
    protected Command getChangeStateCommand(ShooterConstants.ShooterState state, Subsystem... requirements) {
        return new RunCommand(() -> this.setVelocity(state.getUnit()), mergeSubsystemLists(requirements, this));
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
