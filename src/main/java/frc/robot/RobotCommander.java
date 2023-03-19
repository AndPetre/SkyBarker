package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.ArmPos.ArmBumpDirection;
import frc.robot.subsystems.Intake.IntakePos;
import frc.robot.subsystems.Intake.IntakeSpeed;

public abstract class RobotCommander {
    IntakePos intakePos = IntakePos.none;
    IntakeSpeed intakeSpeed = IntakeSpeed.none;

    public Alliance allaince;

    public XboxController driver;
    public XboxController operator;

    public abstract double getForwardCommand();
    public abstract double getStrafeCommand();
    public abstract double getTurnCommand();
    public abstract boolean getResetIMU();
    public abstract double armShoulder();
    public abstract double armExtension();
    public abstract Arm.ArmPos getArmPosition();
    public abstract IntakePos getIntakePosition();
    public abstract IntakeSpeed getIntakeSpeed();
    public abstract boolean getPickUpObject();
    public abstract boolean getArmReset();
    public abstract double armElbow();
    public abstract boolean hopperOverrideLeft();
    public abstract boolean hopperOverrideRight();
    public abstract boolean getAutoBalance();
    public abstract double getGripperCommand();
    public abstract boolean useNegativeSide();
    public abstract ArmBumpDirection getArmBumpDirection();
    public abstract boolean runHopper();
}
