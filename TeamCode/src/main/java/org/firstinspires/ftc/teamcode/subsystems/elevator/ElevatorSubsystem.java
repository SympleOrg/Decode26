package org.firstinspires.ftc.teamcode.subsystems.elevator;

import com.bylazar.telemetry.JoinedTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.RobotConstants.ElevatorConstants;
import org.firstinspires.ftc.teamcode.maps.MotorMap;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.subsystem.LoggerSubsystem;
import org.firstinspires.ftc.teamcode.util.subsystem.StateSubsystemBase;

public class ElevatorSubsystem extends StateSubsystemBase<ElevatorConstants.ElevatorStates> implements LoggerSubsystem {
    private final MotorEx motor;

    private final JoinedTelemetry telemetry;
    private final DataLogger dataLogger;

    public ElevatorSubsystem(HardwareMap hardwareMap, JoinedTelemetry telemetry, DataLogger dataLogger) {
        super(ElevatorConstants.ElevatorStates.INTAKE);

        this.motor = new MotorEx(hardwareMap, MotorMap.ELEVATOR.getId());

        this.telemetry = telemetry;
        this.dataLogger = dataLogger;

        this.motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    protected double getCurrentPosition() {
        return MathUtil.encoderTicksToMeter(this.motor.getCurrentPosition(), ElevatorConstants.WHEEL_RADIUS, MotorMap.ELEVATOR.getTicksPerRev(), ElevatorConstants.GEAR_RATIO);
    }

    protected void moveMotor(double power) {
        this.motor.set(power);
    }

    @Override
    protected Command getChangeStateCommand(ElevatorConstants.ElevatorStates state, Subsystem... requirements) {
        return new HoldPositionElevatorCommand(this, state.getUnit());
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
