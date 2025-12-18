package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.maps.MotorMap;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.subsystem.LoggerSubsystem;

public class IntakeSubsystem extends SubsystemBase implements LoggerSubsystem {
    private final MultipleTelemetry telemetry;
    private final DataLogger dataLogger;

    private final MotorEx motor;

    public IntakeSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry, DataLogger dataLogger) {
        this.telemetry = telemetry;
        this.dataLogger = dataLogger;
        this.motor = new MotorEx(hardwareMap, MotorMap.INTAKE.getId());
    }

    private void moveMotor(double power) {
        this.motor.set(power);
    }

    public Command takeTheBall() {
        return new RunCommand(() -> this.moveMotor(1), this);
    }

    public Command stopIntake() {
        return new RunCommand(() -> this.moveMotor(0), this);

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

