package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    private final CANCoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANCoder(absoluteEncoderId);

        absoluteEncoder.setPositionToAbsolute();

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless); //移動馬達
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless); //轉動馬達

        driveMotor.setIdleMode(IdleMode.kCoast);
        turningMotor.setIdleMode(IdleMode.kCoast);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder(); //移動馬達編碼器
        turningEncoder = turningMotor.getEncoder(); //轉動馬達編碼器

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);


        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        resetEncoders();
        putDashboard();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition(); //取得移動馬達角度
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition(); //取得轉動馬達角度
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity(); //取得移動馬達速度
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity(); //取得轉動馬達速度
    }

    public double getAbsoluteEncoderRad() {
        double angle = (absoluteEncoder.getAbsolutePosition() - absoluteEncoderOffsetRad) / 360.;
        angle *= 2.0 * Math.PI;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());
        putDashboard();
    }


    public void stop() { //停止
        driveMotor.set(0);
        turningMotor.set(0);
    }
    
    public void ResetTurningMotor() { //重設轉動馬達
        if (Math.abs(turningEncoder.getPosition()) < 0.1) {
            stop();
            return;
        }
        turningMotor.set(turningPidController.calculate(turningEncoder.getPosition(), 0));
        driveMotor.set(.0);
        putDashboard();
    }
    
    
    public void putDashboard () { 
        SmartDashboard.putNumber("ABS angle " + absoluteEncoder.getDeviceID(), getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Abs Position " + absoluteEncoder.getDeviceID(), absoluteEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Position " + absoluteEncoder.getDeviceID(), absoluteEncoder.getPosition());
        SmartDashboard.putNumber("Turing position " + turningMotor.getDeviceId(), turningEncoder.getPosition());
    }
}
