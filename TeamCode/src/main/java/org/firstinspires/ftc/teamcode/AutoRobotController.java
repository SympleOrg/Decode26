package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedropathing.Paths;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.gate.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.storage.StorageSubsystem;
import org.firstinspires.ftc.teamcode.util.TeamColor;
import org.firstinspires.ftc.teamcode.util.controlcommands.ActuatorCommands;
import org.firstinspires.ftc.teamcode.util.opModes.SympleCommandOpMode;

public class AutoRobotController extends RobotControllerBase {
    private final MecanumDriveSubsystem mecanumDriveSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final GateSubsystem gateSubsystem;
    private final StorageSubsystem storageSubsystem;
    private final TurretSubsystem turretSubsystem;

    private final TeamColor teamColor;

    private final ActuatorCommands actuatorCommands;

    private Command autoCommand;

    public AutoRobotController(HardwareMap hMap, Telemetry telemetry, Gamepad driverController, Gamepad actionController, String logFilePrefix, boolean logData, TeamColor teamColor) {
        super(hMap, telemetry, driverController, actionController, logFilePrefix, logData);

        this.mecanumDriveSubsystem = new MecanumDriveSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());
        this.intakeSubsystem = new IntakeSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());
        this.storageSubsystem = new StorageSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());
        this.gateSubsystem = new GateSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());
        this.turretSubsystem = new TurretSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());

        this.teamColor = teamColor;

        this.actuatorCommands = new ActuatorCommands(
                this.mecanumDriveSubsystem,
                this.intakeSubsystem,
                this.gateSubsystem,
                this.storageSubsystem,
                this.turretSubsystem
        );
    }

    @Override
    public void createKeyBindings() {

    }

    @Override
    public void initialize() {
        if (this.teamColor == TeamColor.RED) {
            getPathFollower().setStartingPose(RobotConstants.AutoConstants.RED_GOAL_POSE);
            this.autoCommand = new ParallelCommandGroup(
                    this.actuatorCommands.storageGoToShooter(),
                    new SequentialCommandGroup(
                            new FollowPathCommand(getPathFollower(), Paths.createRedPath(getPathFollower())),
                            new RepeatCommand(
                                    new SequentialCommandGroup(
                                            new WaitUntilCommand(this.turretSubsystem::isFastEnough),
                                            this.gateSubsystem.goToState(RobotConstants.GateConstants.GateState.PUSH),
                                            new WaitUntilCommand(() -> !this.turretSubsystem.isFastEnough()),
                                            this.gateSubsystem.goToState(RobotConstants.GateConstants.GateState.ZERO)
                                    )
                            ).withTimeout(20_000)
                    )
            );
        }
    }

    @Override
    public void initializeLoop() {

    }

    @Override
    public void postInitialize() {
        this.autoCommand.schedule();
    }

    @Override
    public void run() {

    }

    @Override
    public void postRun() {

    }

    public static class Builder extends RobotControllerBase.Builder {
        private TeamColor teamColor;

        public Builder() {
            this.logFilePrefix = "Auto";
        }

        @Override
        public Builder initializeDefaults(SympleCommandOpMode opMode) {
            super.initializeDefaults(opMode);
            return this;
        }

        public Builder setTeamColor(TeamColor teamColor) {
            this.teamColor = teamColor;
            return this;
        }

        @Override
        public AutoRobotController build() {
            if (teamColor == null) throw new IllegalArgumentException("No set team color!");
            return new AutoRobotController(this.hardwareMap, this.telemetry, this.driverController, this.actionController, this.logFilePrefix, this.logData, this.teamColor);
        }
    }
}
