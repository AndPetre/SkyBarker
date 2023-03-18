package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Servo;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotCommander;
import static frc.robot.Constants.*;

import org.hotutilites.hotlogger.HotLogger;

public class Hopper {
    public static int PixieCam = 0;
    public static Servo fingerServo = new Servo(FINGER_SERVO);
    public static DigitalInput sensorTop= new DigitalInput(TOP_SENSOR);
    public static DigitalInput sensorLeft= new DigitalInput(LEFT_SENSOR);
    public static DigitalInput sensorRight= new DigitalInput(RIGHT_SENSOR);
    public static DigitalInput sensorBottom= new DigitalInput(BOTTOM_SENSOR);
    public static CANSparkMax hopperMotor = new CANSparkMax(HOPPER_MOTOR, MotorType.kBrushless);
    public static HopperCam hopperCam = HopperCam.CUBE;
    public static boolean hopperOverrideActive = false;

    enum HopperCam {
      NONE,
      CUBE,
      CONE
    }

    public void HopperPeriodic(RobotCommander commander){
        hopperMotor.set(commander.operator.getRightX() * .8);
    }

  public void logData(){
    SmartDashboard.putBoolean("HopSensor Bottom", sensorBottom.get());
    SmartDashboard.putBoolean("HopSensor Left", sensorLeft.get());
    SmartDashboard.putBoolean("HopSensor Right", sensorRight.get());
    SmartDashboard.putBoolean("HopSensor Top", sensorTop.get());
    SmartDashboard.putBoolean("Hopper Override", hopperOverrideActive);

    HotLogger.Log("HopSensor Bottom", sensorBottom.get());
    HotLogger.Log("HopSensor Left", sensorLeft.get());
    HotLogger.Log("HopSensor Right", sensorRight.get());
    HotLogger.Log("HopSensor Top", sensorTop.get());
    HotLogger.Log("Hopper Override", hopperOverrideActive);
    }
}
  
