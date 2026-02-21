package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.bylazar.telemetry.JoinedTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.RobotConstants.TurretConstants;
import org.firstinspires.ftc.teamcode.maps.MotorMap;
import org.firstinspires.ftc.teamcode.maps.ServoMap;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.SympleServo;
import org.firstinspires.ftc.teamcode.util.TeamColor;
import org.firstinspires.ftc.teamcode.util.subsystem.LoggerSubsystem;

import java.util.function.Supplier;

public class TurretSubsystem extends SubsystemBase implements LoggerSubsystem {
    private final MotorEx shooterMotor;
    private final MotorEx turretMotor;
    private final SympleServo hoodServo;
    private final VoltageSensor voltageSensor;

    private final TurretController controller;

    private final DataLogger dataLogger;
    private final JoinedTelemetry telemetry;

    public TurretSubsystem(HardwareMap hardwareMap, JoinedTelemetry telemetry, DataLogger dataLogger) {
        this.shooterMotor = new MotorEx(hardwareMap, MotorMap.SHOOTER.getId(), MotorMap.SHOOTER.getTicksPerRev(), MotorMap.SHOOTER.getMaxRPM());
        this.turretMotor = new MotorEx(hardwareMap, MotorMap.TURRET.getId(), MotorMap.TURRET.getTicksPerRev(), MotorMap.TURRET.getMaxRPM());
        this.hoodServo = new SympleServo(hardwareMap, ServoMap.HOOD.getId(), 0, 300);

        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();

        this.controller = new TurretController(this);

        this.dataLogger = dataLogger;
        this.telemetry = telemetry;

        this.shooterMotor.setRunMode(Motor.RunMode.VelocityControl);
        this.shooterMotor.setVeloCoefficients(TurretConstants.Shooter.Kp, TurretConstants.Shooter.Ki, TurretConstants.Shooter.Kd);
        this.shooterMotor.setFeedforwardCoefficients(TurretConstants.Shooter.Ks, TurretConstants.Shooter.Kv, TurretConstants.Shooter.Ka);

        this.turretMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    protected void setVelocity(double velocity) {
        double voltageModifier = 12f / this.voltageSensor.getVoltage();
        this.shooterMotor.setVelocity(meterPerSecToTicksPerSec(velocity) * voltageModifier);
    }

    protected double getFlywheelVelocity() {
        return ticksPerSecToMeterPerSec(this.shooterMotor.getCorrectedVelocity());
    }

    protected double getCurrentTurretAngle() {
        return MathUtil.ticksToDeg(this.turretMotor.getCurrentPosition(), MotorMap.TURRET.getTicksPerRev()) * TurretConstants.Turret.GEAR_RATIO;
    }

    protected void moveTurret(double power) {
        this.turretMotor.set(power);
    }

    protected void moveHood(double angle) {
        this.hoodServo.set(angle);
    }

    public Command updateCommand(Supplier<Pose> robotPose, TeamColor teamColor) {
        return new RunCommand(() -> this.controller.update(robotPose.get(), teamColor), this);
    }

    public static double ticksPerSecToMeterPerSec(double ticks) {
        return MathUtil.encoderTicksToMeter(ticks, TurretConstants.Shooter.WHEEL_RADIUS, MotorMap.INTAKE.getTicksPerRev(), TurretConstants.Shooter.SHOOTER_GEAR_RATIO);
    }

    public static double meterPerSecToTicksPerSec(double meters) {
        return MathUtil.meterToEncoderTicks(meters, TurretConstants.Shooter.WHEEL_RADIUS, MotorMap.INTAKE.getTicksPerRev(), TurretConstants.Shooter.SHOOTER_GEAR_RATIO);
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
