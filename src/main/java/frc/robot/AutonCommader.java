package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import frc.robot.Autons.AutonBase;
import frc.robot.subsystems.Arm.ArmPos;

public class AutonCommader extends RobotCommander{

    AutonBase auton;

    public void initAuton(AutonBase passedAuton){
        auton = passedAuton;
    }

    public void runAuto(){
        auton.runAuto();
    }

    @Override
    public double getForwardCommand() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getStrafeCommand() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getTurnCommand() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public boolean getResetIMU() {
        // TODO Auto-generated method stub
        // return auton.getResetIMU;
        return false;
    }

    public State getDesiredState(){
        return auton.desState;
    }

    public Rotation2d getTargetTheta(){
        return auton.targetTheta;
    }

    public PathPlannerState getInitalState(){
        return auton.initalState;
    }

    @Override
    public double armShoulder() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double armExtension() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public ArmPos getArmPosition() {
        // TODO Auto-generated method stub
        return auton.armPos;
    }

    @Override
    public double[] getIntakePosition() {
        // TODO Auto-generated method stub
        if(auton.intakeOn){
            intakeArray[0] = Constants.INTAKE_COLLECT_POSITION;
            intakeArray[1] = Constants.INTAKE_SPEED_CUBE;
        } else {
            intakeArray[0] = Constants.INTAKE_PACKAGE_POSITION;
            intakeArray[1] = 0;
        }

        return intakeArray;
    }

    @Override
    public boolean getPickUpObject() {
        // TODO Auto-generated method stub
        return auton.pickUpObject;
    }

    @Override
    public boolean getArmReset() {
        // TODO Auto-generated method stub
        return false;
    }
}