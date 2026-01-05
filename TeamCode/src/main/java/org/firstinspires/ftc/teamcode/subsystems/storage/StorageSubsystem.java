package org.firstinspires.ftc.teamcode.subsystems.storage;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants.StorageConstants;
import org.firstinspires.ftc.teamcode.maps.MotorMap;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.subsystem.LoggerSubsystem;
import org.firstinspires.ftc.teamcode.util.subsystem.StateSubsystemBase;

import kotlin.NotImplementedError;

public class StorageSubsystem extends StateSubsystemBase<StorageConstants.StorageState> implements LoggerSubsystem {

    private final MotorEx motorEx;
    private final DataLogger dataLogger;
    private final MultipleTelemetry telemetry;

    public StorageSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry, DataLogger dataLogger) {
        super(StorageConstants.StorageState.INTAKE);

        this.motorEx = new MotorEx(hardwareMap, MotorMap.STORAGE.getId());
        this.dataLogger = dataLogger;
        this.telemetry = telemetry;
    }

    @Override
    protected Command getChangeStateCommand(StorageConstants.StorageState state, Subsystem... requirements) {
//        return;
        throw new NotImplementedError("getChangeStateCommand method in StorageSubsystem is not implemented!");
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
