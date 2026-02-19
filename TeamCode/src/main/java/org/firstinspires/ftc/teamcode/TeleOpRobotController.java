package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands.mecanumDrive.MecanumArcadeDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.storage.StorageSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretSubsystem;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.TeamColor;
import org.firstinspires.ftc.teamcode.util.controlcommands.ActuatorCommands;
import org.firstinspires.ftc.teamcode.util.controlcommands.DriverCommands;
import org.firstinspires.ftc.teamcode.util.opModes.SympleCommandOpMode;

public class TeleOpRobotController extends RobotControllerBase {
    private final MecanumDriveSubsystem mecanumDriveSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final StorageSubsystem storageSubsystem;
    private final TurretSubsystem turretSubsystem;

    private final DriverCommands driverCommands;
    private final ActuatorCommands actuatorCommands;

    private final TeamColor teamColor;

    private TeleOpRobotController(HardwareMap hMap, Telemetry telemetry, Gamepad driverController, Gamepad actionController, TeamColor teamColor, String logFilePrefix, boolean logData) {
        super(hMap, telemetry, driverController, actionController, logFilePrefix, logData);

        if (teamColor == null) {
            RuntimeException exception = new RuntimeException("Team color cannot be null!");
            this.getDataLogger().addThrowable(exception);
            throw exception;
        }

        this.teamColor = teamColor;

        this.mecanumDriveSubsystem = new MecanumDriveSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());
        this.intakeSubsystem = new IntakeSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());
        this.storageSubsystem = new StorageSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());
        this.turretSubsystem = new TurretSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());

        this.driverCommands = new DriverCommands(
                this.mecanumDriveSubsystem,
                this.intakeSubsystem,
                this.storageSubsystem,
                this.turretSubsystem
        );

        this.actuatorCommands = new ActuatorCommands(
                this.mecanumDriveSubsystem,
                this.intakeSubsystem,
                this.storageSubsystem,
                this.turretSubsystem
        );
    }

    @Override
    public void createKeyBindings() {
        this.driverController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(this.driverCommands::setNormalSpeedMode);

        this.driverController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(this.driverCommands::setSlowSpeedMode);

        this.driverController.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(this.driverCommands::resetRobotAngle);

        this.actionController.getGamepadButton(GamepadKeys.Button.X)
                .toggleWhenPressed(
                        this.actuatorCommands.startIntake(),
                        this.actuatorCommands.stopIntake()
                );

        this.actionController.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(() -> this.storageSubsystem.setBall(2, BallColor.NONE));
//
//        this.actionController.getGamepadButton(GamepadKeys.Button.Y)
//                .toggleWhenPressed(
//                        this.actuatorCommands.storageGoToShooter(),
//                        this.actuatorCommands.storageGoToIntake()
//                );
//
//        this.actionController.getGamepadButton(GamepadKeys.Button.B)
//                .toggleWhenPressed(
//                        this.actuatorCommands.pushTheBall(),
//                        this.actuatorCommands.returnToZero()
//                );

//        new Trigger(this.turretSubsystem::isFastEnough)
//                .whenActive(() -> this.actionController.gamepad.rumble(500));

//        new Trigger(() -> this.actionController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5f)
//                .whileActiveContinuous(this.actuatorCommands.shootBallNow())
//                .whenInactive(this.actuatorCommands.returnToZero());
    }

    @Override
    public void initialize() {
        this.mecanumDriveSubsystem.setDefaultCommand(new MecanumArcadeDriveCommand(this.mecanumDriveSubsystem, this.driverController));
    }

    @Override
    public void initializeLoop() {

    }

    @Override
    public void postInitialize() {
        this.turretSubsystem.setDefaultCommand(
                this.turretSubsystem.updateCommand(this.getPathFollower()::getPose, this.teamColor)
        );
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
            this.logFilePrefix = "TeleOp";
        }

        public Builder teamColor(TeamColor value) {
            this.teamColor = value;
            return this;
        }

        @Override
        public Builder initializeDefaults(SympleCommandOpMode opMode) {
            super.initializeDefaults(opMode);
            return this;
        }

        @Override
        public TeleOpRobotController build() {
            return new TeleOpRobotController(this.hardwareMap, this.telemetry, this.driverController, this.actionController, this.teamColor, this.logFilePrefix, this.logData);
        }
    }
}
