package org.firstinspires.ftc.teamcode.subsystems.gate;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants.GateConstants;
import org.firstinspires.ftc.teamcode.maps.ServoMap;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.SympleServo;
import org.firstinspires.ftc.teamcode.util.subsystem.LoggerSubsystem;
import org.firstinspires.ftc.teamcode.util.subsystem.StateSubsystemBase;

public class GateSubsystem extends StateSubsystemBase<GateConstants.GateState> implements LoggerSubsystem {

    private final SympleServo servo;
    private final MultipleTelemetry telemetry;
    private final DataLogger dataLogger;

    public GateSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry, DataLogger dataLogger) {
        super(GateConstants.GateState.ZERO);

        this.dataLogger = dataLogger;
        this.telemetry = telemetry;
        this.servo = new SympleServo(hardwareMap, ServoMap.GATE.getId(), 0, 300);
    }

    private void moveServo(double deg) {
        this.servo.turnToAngle(deg);
    }

    @Override
    protected Command getChangeStateCommand(GateConstants.GateState state, Subsystem... requirements) {
        return new InstantCommand(() -> moveServo(state.getUnit()), mergeSubsystemLists(requirements, this));
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
