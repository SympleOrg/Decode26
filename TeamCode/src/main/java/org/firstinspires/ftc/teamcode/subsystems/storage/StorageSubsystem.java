package org.firstinspires.ftc.teamcode.subsystems.storage;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.RobotConstants.StorageConstants;
import org.firstinspires.ftc.teamcode.maps.MotorMap;
import org.firstinspires.ftc.teamcode.maps.SensorMap;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.subsystem.LoggerSubsystem;
import org.firstinspires.ftc.teamcode.util.subsystem.StateSubsystemBase;

public class StorageSubsystem extends StateSubsystemBase<StorageConstants.StorageState> implements LoggerSubsystem {

    private final MotorEx motorEx;
    private final TouchSensor touchSensor;
    private final DataLogger dataLogger;
    private final MultipleTelemetry telemetry;

    public StorageSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry, DataLogger dataLogger) {
        super(StorageConstants.StorageState.INTAKE);

        this.motorEx = new MotorEx(hardwareMap, MotorMap.STORAGE.getId());
        this.dataLogger = dataLogger;
        this.telemetry = telemetry;

        this.motorEx.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        this.motorEx.resetEncoder();

        this.touchSensor = hardwareMap.get(TouchSensor.class, SensorMap.STORAGE_TOUCH.getId());
    }

    @Override
    public void periodic() {
        this.telemetry.addData("Storage pos", this.getCurrentPosition());
        this.telemetry.addData("Storage touch", this.touchSensor.isPressed());

        if (this.touchSensor.isPressed()) {
            this.motorEx.resetEncoder();
        }
    }

    public double getCurrentPosition() {
        return MathUtil.ticksToDeg(this.motorEx.getCurrentPosition(), MotorMap.STORAGE.getTicksPerRev());
    }

    public void moveMotor(double power) {
        this.motorEx.set(power);
    }

    @Override
    protected Command getChangeStateCommand(StorageConstants.StorageState state, Subsystem... requirements) {
        return new MoveStorageCommand(this, state.getUnit());
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
