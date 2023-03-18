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
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;

public class RedAutoMid1Bal extends AutonBase{
    enum AutoState {
        firstPlace,
        driveToStation,
        mobility,
        balance,
        end
    }

    public AutoState autoState;

    public Timer timer = new Timer();
    public Timer totalTime = new Timer();

    int point = 0;

    Pose2d startingPose = new Pose2d(0,0, Rotation2d.fromDegrees(90));
    Pose2d endPose = new Pose2d(4,0, Rotation2d.fromDegrees(90));

    Trajectory trajectory;

    public RedAutoMid1Bal(){
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

                if(timer.get() > 4){
                    double headingAngle = Math.toDegrees(Math.atan2(endPose.getY()-startingPose.getY(), 
                                             endPose.getX()-startingPose.getX()));

                    trajectory = TrajectoryGenerator.generateTrajectory(
                        new Pose2d(startingPose.getTranslation(), Rotation2d.fromDegrees(headingAngle)),
                        List.of(),
                        new Pose2d(endPose.getTranslation(), Rotation2d.fromDegrees(headingAngle)),
                        new TrajectoryConfig(1, 1).setKinematics(Drivetrain.kinematics));

                    timer.reset();

                    autoState = AutoState.mobility;
                }
            break;
            case mobility:
                driving = true;
                desState = trajectory.sample(timer.get());
                targetTheta = endPose.getRotation();

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