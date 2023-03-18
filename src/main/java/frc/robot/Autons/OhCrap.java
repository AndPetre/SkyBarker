package frc.robot.Autons;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sensors.Camera;
import frc.robot.sensors.Pigeon;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Arm.ArmPos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;

public class OhCrap extends AutonBase{
    public Timer timer = new Timer();

    int point = 0;

    Trajectory trajectory;

    public OhCrap(){
        reset();
    }

    public void reset(){
        desState = new State();
        targetTheta = Rotation2d.fromDegrees(-90);
        
        timer.reset();
        timer.start();
    }

    public void runAuto(){
        driving = false;
        if(timer.get() < 3){
            if(timer.get() < 2.5){
                gripperSpeed = -.4;
            }else{
                gripperSpeed = .4;
            }
            armPos = ArmPos.topNodeCone;
        } else {
            gripperSpeed = 0;
            armPos = ArmPos.packagePos;
        }
    }
}