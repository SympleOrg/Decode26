package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.maps.MotorMap;

public class IntakeSubsystem extends SubsystemBase {

    private final MotorEx motor;

    public IntakeSubsystem(HardwareMap hardwareMap) {
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
}

