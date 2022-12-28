package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.DriverPort;

public class ElevatorSubsystem extends SubsystemBase {

    private final CANSparkMax elevatorRightMotor;
    private final CANSparkMax elevatorLeftMotor;

    elevatorRightMotor = new CANSparkMax(RobotMap.kElevatorRightMotorPort);
    elevatorLeftMotor = new CANSparkMax(RobotMap.kElevatorLeftMotorPort);

    elevatorRightMotor.setIdleMode(IdleMode.kCoast);
    elevatorLeftMotor.setIdleMode(IdleMode.kCoast);

    public ElevatorSubsystem() {
    }

    public void setMotor(double speed) {
      elevatorRightMotor.set(speed); //升降
      elevatorLeftMotor.set(-speed); //轉向與另一顆馬達相反
    }
}