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

public class ArmSubsystem extends SubsystemBase {

    private final CANSparkMax armMotor1;
    private final CANSparkMax armMotor2;

    armMotor1 = new CANSparkMax(RobotMap.kArmMotor1Port);
    armMotor2 = new CANSparkMax(RobotMap.kArmMotor2Port);

    armMotor1.setIdleMode(IdleMode.kCoast);
    armMotor2.setIdleMode(IdleMode.kCoast);

    public ArmSubsystem() {
    }

    public void setMotor(double speed) {
      armMotor1.set(speed); //控制夾子的手臂
      armMotor2.set(speed); //馬達同向
    }
}