package org.firstinspires.ftc.teamcode.subsystems.storage;

import com.bylazar.telemetry.JoinedTelemetry;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.RobotConstants.StorageConstants;
import org.firstinspires.ftc.teamcode.maps.SensorMap;
import org.firstinspires.ftc.teamcode.maps.ServoMap;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.ColorRGBA;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.SympleServo;
import org.firstinspires.ftc.teamcode.util.subsystem.LoggerSubsystem;

public class StorageSubsystem extends SubsystemBase implements LoggerSubsystem {
    private final SympleServo shooterServo;
    private final SympleServo storageServo;
    private final ColorSensor shooterColorSensor;
    private final ColorSensor storageColorSensor;

    private StorageState storageState;
    private ShooterState shooterState;
    private boolean isReadyToReload;
    private final Timer indexerTimer;
    private final BallColor[] indexer;

    private final JoinedTelemetry telemetry;
    private final DataLogger dataLogger;

    public StorageSubsystem(HardwareMap hardwareMap, JoinedTelemetry telemetry, DataLogger dataLogger) {
        this.shooterServo = new SympleServo(hardwareMap, ServoMap.SHOOTER.getId(), 0, 300);
        this.storageServo = new SympleServo(hardwareMap, ServoMap.STORAGE.getId(), 0, 300);
        this.shooterColorSensor = hardwareMap.get(ColorSensor.class, SensorMap.SHOOTER_COLOR.getId());
        this.storageColorSensor = hardwareMap.get(ColorSensor.class, SensorMap.STORAGE_COLOR.getId());

        this.storageState = StorageState.IDLE;
        this.shooterState = ShooterState.IDLE;
        this.indexerTimer = new Timer();
        this.indexer = new BallColor[] {BallColor.NONE, BallColor.NONE, BallColor.NONE};

        this.isReadyToReload = indexer[2] == BallColor.NONE;

        this.dataLogger = dataLogger;
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {

        double elapsed = indexerTimer.getElapsedTime();

        ColorRGBA storageColors = new ColorRGBA(
                storageColorSensor.red(),
                storageColorSensor.green(),
                storageColorSensor.blue(),
                storageColorSensor.alpha());

        ColorRGBA shooterColors = new ColorRGBA(
                shooterColorSensor.red(),
                shooterColorSensor.green(),
                shooterColorSensor.blue(),
                shooterColorSensor.alpha());

        BallColor storageBall = getBallColor(storageColors);
        BallColor shooterBall = getBallColor(shooterColors);

        // Storage
        switch (storageState) {

            case IDLE:
                if (storageBall != BallColor.NONE && indexer[0] == BallColor.NONE) {
                    indexer[0] = storageBall;
                }

                if (indexer[0] != BallColor.NONE && indexer[1] == BallColor.NONE) {
                    moveStorageServo(StorageConstants.HIGHER_STORAGE_SERVO_ANGLE);
                    indexerTimer.resetTimer();
                    storageState = StorageState.RAISING;
                }
                break;

            case RAISING:
                if (elapsed > StorageConstants.TRANSFER_TIME) {
                    indexer[1] = indexer[0];
                    indexer[0] = BallColor.NONE;
                    storageState = StorageState.WAITING_TRANSFER;
                }
                break;

            case WAITING_TRANSFER:
                if (elapsed > StorageConstants.TRANSFER_TIME * 2) {
                    moveStorageServo(StorageConstants.LOWER_STORAGE_SERVO_ANGLE);
                    storageState = StorageState.LOWERING;
                }
                break;

            case LOWERING:
                if (elapsed > StorageConstants.TRANSFER_TIME * 3) {
                    storageState = StorageState.IDLE;
                }
                break;
        }

        // shooter
        switch (shooterState) {

            case IDLE:
                if (indexer[1] != BallColor.NONE && indexer[2] == BallColor.NONE && isReadyToReload) {
                    moveShooterServo(StorageConstants.HIGHER_SHOOTER_SERVO_ANGLE);
                    shooterState = ShooterState.RAISING;
                }
                break;

            case RAISING:
                if (shooterBall != BallColor.NONE) {
                    indexer[2] = indexer[1];
                    indexer[1] = BallColor.NONE;

                    moveShooterServo(StorageConstants.LOWER_SHOOTER_SERVO_ANGLE);
                    shooterState = ShooterState.LOWERING;
                }
                break;

            case LOWERING:
                if (shooterBall == BallColor.NONE) {
                    shooterState = ShooterState.IDLE;
                }
                break;
        }

        telemetry.addData("Storage Color Sensor RGBA", String.format("%s, %s, %s, %s", storageColors.red(), storageColors.green(), storageColors.blue(), storageColors.alpha()));
        telemetry.addData("Shooter Color Sensor RGBA", String.format("%s, %s, %s, %s", shooterColors.red(), shooterColors.green(), shooterColors.blue(), shooterColors.alpha()));
        telemetry.addData("Indexer Timer", indexerTimer.getElapsedTime());
        telemetry.addData("Storage State", storageState);
        telemetry.addData("Shooter State", shooterState);
        telemetry.addData("Is Ready To Reload", isReadyToReload);
        telemetry.addData("Storage Ball Type (Sensor)", storageBall.name());
        telemetry.addData("Shooter Ball Type (Sensor)", shooterBall.name());
        telemetry.addData("Indexer [0] Ball Type", indexer[0].name());
        telemetry.addData("Indexer [1] Ball Type", indexer[1].name());
        telemetry.addData("Indexer [2] Ball Type", indexer[2].name());
    }

    public void setReloadStatus(boolean ready) {
        this.isReadyToReload = ready;
    }

    public void setBall(int index, BallColor ballColor) {
       this.setBall(index, ballColor, false);
    }

    public void setBall(int index, BallColor ballColor, boolean updateReloadStatus) {
        if (index < 0 || index >= indexer.length) throw new RuntimeException(String.format("Index must be between 0 and %s", indexer.length - 1));
        this.indexer[index] = ballColor;

        this.isReadyToReload = indexer[2] == BallColor.NONE;
    }

    private void moveShooterServo(double deg) {
        this.shooterServo.set(deg);
    }

    private void moveStorageServo(double deg) {
        this.storageServo.set(deg);
    }

    private BallColor getBallColor(ColorRGBA color) {
        if (
                color.red() >= StorageConstants.GreenBall.MIN_RED
                && color.green() >= StorageConstants.GreenBall.MIN_GREEN
                && color.blue() >= StorageConstants.GreenBall.MIN_BLUE
        ) return BallColor.GREEN;

        if (
                color.red() >= StorageConstants.PurpleBall.MIN_RED
                && color.green() >= StorageConstants.PurpleBall.MIN_GREEN
                && color.blue() >= StorageConstants.PurpleBall.MIN_BLUE
        ) return BallColor.PURPLE;

        return BallColor.NONE;
    }

    @Override
    public JoinedTelemetry getTelemetry() {
        return this.telemetry;
    }

    @Override
    public DataLogger getDataLogger() {
        return this.dataLogger;
    }

    private enum StorageState {
        IDLE,
        RAISING,
        WAITING_TRANSFER,
        LOWERING
    }

    private enum ShooterState {
        IDLE,
        RAISING,
        WAITING_DETECTION,
        LOWERING
    }
}
