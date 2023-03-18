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

public class RedAutoLeft1Bal extends AutonBase{
    enum AutoState {
        firstPlace,
        driveToStation,
        balance,
        end
    }

    public AutoState autoState;

    public Timer timer = new Timer();

    public Timer totalTime = new Timer();

    int point = 0;

    List<Pose2d> path = List.of(new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(90)), //4.82, .5
                                new Pose2d(new Translation2d(.3,-.75), Rotation2d.fromDegrees(90)));

    Trajectory trajectory;

    public RedAutoLeft1Bal(){
        reset();
    }

    public void reset(){
        desState = new State();
        targetTheta = Rotation2d.fromDegrees(90);

        point = 0;
        
        timer.reset();
        timer.start();

        totalTime.reset();
        totalTime.start();

        autoState = AutoState.firstPlace;
    }

    public void runAuto(){
        switch(autoState){
            case firstPlace:
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

                if(timer.get() > 5){
                    trajectory = createTrajectory(path.get(point), path.get(point+1));
            
                    point++;

                    timer.reset();

                    autoState = AutoState.driveToStation;
                }
            break;
            case driveToStation:
                driving = true;
                armPos = ArmPos.packagePos;
                intakeOn = false;
                
                desState = trajectory.sample(timer.get());
                targetTheta = path.get(point).getRotation();

                if(timer.get() > trajectory.getTotalTimeSeconds()){
                    timer.reset();

                    autoState = AutoState.balance;
                }
            break;
            case balance:
                driving = false;
                autoBalance = true;
                if(totalTime.get() > 14){
                    autoState = AutoState.end;
                }
            break;
            case end:
                driving = false;
                autoBalance = false;
                xMode = true;
            break;
        }
    }
}