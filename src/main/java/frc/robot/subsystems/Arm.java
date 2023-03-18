package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotCommander;
import frc.robot.subsystems.Arm.ArmPos.ArmBumpDirection;

import org.hotutilites.hotlogger.HotLogger;

//handoff position intake: 180, extension:21, shoulder:0, elbow0

public class Arm {

    private Intake intake = new Intake(); 

    double desiredIntakePos;

    public static enum ArmPos {

        packagePos(0,.1,0),
        readyPosition(-23,.1,100),
        topNodeCone(50,20,181),
        topNodeCube(57,20,181),
        middleNodeCone(51,.2,181),
        middleNodeCube(58,.2,181),
        lowerNode(27,.2,69),
        manual(0,0,0), // manual motor commands
        Zero(0,0,0), // No motor command
        intake(0,10,0), 
        outOfHopperToDirection(-20,.5,5), 
        outOfDirectionToHopper1(-0,.5,80),
        outOfPostiveToHopper2(-20,.5,10),
        outOfHopperToMid(-20,.5,80),
        outOfHumanPlayerInitialExtension(6.8,7.5,-12),
        humanPlayerReady(6.8,22,-67),
        humanPlayerPickup(-7,21.5,-56.2),
        outOfReturnFromHumanPlayer(20,22,-25),
        intakeConeGrab(0,10,0);

        private final double shoulder;
        public double getShoulder() {
            return shoulder;
        }

        private final double extension;
        public double getExtension() {
            return extension;
        }

        private final double elbow;

        public double getElbow() {
            return elbow;
        }

        ArmPos(double shoulder, double extension, double elbow) {
            this.shoulder = shoulder;
            this.extension = extension;
            this.elbow = elbow;
        }


        public enum ArmBumpDirection {
            bumpUp(-1.0/15.0),
            bumpDown(1.0/15.0),
            bumpZero(0);

        private final double shoulder;
        public double getShoulder() {
            return shoulder;
        }

        ArmBumpDirection(double shoulder) {
            this.shoulder = shoulder;
        }
    }

        
    }

    public static enum IntakePos{
        station(Constants.INTAKE_STATION_POSITION , Constants.INTAKE_SPEED_CUBE/2,0),
        pack(Constants.INTAKE_PACKAGE_POSITION , Constants.INTAKE_SPEED_CUBE/2,0),
        collect(Constants.INTAKE_COLLECT_POSITION, Constants.INTAKE_SPEED_CUBE/2,0),
        handoff(90,0,0),
        manual(0,0,0),
        none(0,0,0),
        noneManualMode(0,0,0);


        public double speedReading1;
        public double positionReading;
        public double speedReading2;
    
        public double getSpeedReading1(){
            return speedReading1;
        }
    
        public double getPositionReading(){
            return positionReading;
        }

        public double getSpeedReading2(){
            return speedReading2;
        }

        IntakePos(double positionReading, double speedReading1, double speedReading2) {
            this.speedReading1 = speedReading1;
            this.positionReading = positionReading;
            this.speedReading2 = speedReading2;
        }
    }

    public enum IntakeZone{
        low,
        high,
        handoff,
        none

    }

    public IntakeZone determineIntakeZone(){
        if (intake.angleEncoder.getAbsolutePosition() > 91){
            return IntakeZone.low;
        }
        else if (intake.angleEncoder.getAbsolutePosition() == 91){
            return IntakeZone.handoff;
        }
        else if (intake.angleEncoder.getAbsolutePosition() > 91){
            return IntakeZone.high;
        }
        else{
            return IntakeZone.none;
        }
    }

    public enum ArmZoneForHandoff{
        upNotCenter,
        upCenter,
        downNotCenter,
        downCenter,
        handoff,
        none

    }

    public ArmZoneForHandoff determineArmZoneForHandoff(){
        if(extension.getExtensionPosition() > 22 && (Math.abs(shoulder.getShoulderAngle()) > 3 || Math.abs(elbow.getElbowAngle()) > 3)){
            return ArmZoneForHandoff.upNotCenter;
        }
        else if(extension.getExtensionPosition() > 22 && (Math.abs(shoulder.getShoulderAngle()) < 3 && Math.abs(elbow.getElbowAngle()) < 3)){
            return ArmZoneForHandoff.upCenter;
        }
        else if(extension.getExtensionPosition() < 20 && (Math.abs(shoulder.getShoulderAngle()) > 3 || Math.abs(elbow.getElbowAngle()) > 3)){
            return ArmZoneForHandoff.downNotCenter;
        }
        else if(extension.getExtensionPosition() < 20 && (Math.abs(shoulder.getShoulderAngle()) < 3 && Math.abs(elbow.getElbowAngle()) < 3)){
            return ArmZoneForHandoff.downCenter;
        }
        else if((extension.getExtensionPosition() >= 20 && extension.getExtensionPosition() <= 22) && Math.abs(shoulder.getShoulderAngle()) < 3 && Math.abs(elbow.getElbowAngle()) < 3){
            return ArmZoneForHandoff.handoff;
        }
        else{
            return ArmZoneForHandoff.none;
        }
    }


    double intakeAbsEncoderPos;
    double shoulderDesPos;
    double extensionDesPos;
    double elbowDesPos;
    private double shoulderAngleMotor;
    private double shoulderAngleCANCODER;

    private Extension extension;
    private Elbow elbow;
    private Shoulder shoulder;
    private ArmPos armTargetPrevious;
    private ArmZone currentZone;
    private ArmZone currentCommandedZone = ArmZone.none;
    private ArmPos actualCommand;
    private boolean achivedPostion;
    private boolean transitionStateInProgress;

    private boolean useNegativeSide;
    private double shoulderBumpOffSet;
    private int bumpLatchTimer;
    private ArmPos bumpLatchCommand;

    public Arm(){
        shoulder = new Shoulder(Constants.SHOULDER, Constants.SHOULDER_ENCODER);
        extension = new Extension(Constants.EXTENSION);
        elbow = new Elbow(Constants.ELBOW, Constants.ELBOW_ENCODER);
        shoulderBumpOffSet = 0;
        bumpLatchTimer = 10000;
    }

    public void initilizeOffsets() {
        shoulder.intilizeOffset();
        elbow.intilizeOffset();

        currentCommandedZone = ArmZone.hopper;
    }
    
    public void armPercentOutZero(){
        shoulder.setMotorCommand(0.0);
        elbow.setMotorCommand(0.0);
        extension.setMotorCommand(0.0);
    }

    public enum ArmZone {
        hopper,
        negative,
        postive, anyZone, none
    }



    public ArmZone determineArmZone(double shoulder, double extension, double elbow) {

        if (elbow < -40 || (shoulder > 17 && shoulder < 22 && elbow < 10)) {
            return ArmZone.negative;
        } else if (elbow > 40 || (shoulder < -17 && shoulder > -22  && elbow > -10)) {
            return ArmZone.postive;
        }
        else {
            return ArmZone.hopper;
        }
    }

    public enum ArmZoneHandoff {
        armUpNotCenterIntakeDown,
        armUpCenterIntakeDown,
        armUpNotCenterIntakeUp,
        armUpCenterIntakeUp,
        armDownNotCenterIntakeDown,
        armDownCenterIntakeDown,
        armDownNotCenterIntakeUp,
        armDownCenterIntakeUp,
        handoff,
        none
    }

    public ArmZoneHandoff determineZoneHandoff(){
        if(determineArmZoneForHandoff() == ArmZoneForHandoff.upCenter && determineIntakeZone() == IntakeZone.low){
            return ArmZoneHandoff.armUpCenterIntakeDown;
        }
        else if(determineArmZoneForHandoff() == ArmZoneForHandoff.upNotCenter && determineIntakeZone() == IntakeZone.low){
            return ArmZoneHandoff.armUpNotCenterIntakeDown;
        }
        else if(determineArmZoneForHandoff() == ArmZoneForHandoff.upCenter && determineIntakeZone() == IntakeZone.high){
            return ArmZoneHandoff.armUpCenterIntakeUp;
        }
        else if(determineArmZoneForHandoff() == ArmZoneForHandoff.upNotCenter && determineIntakeZone() == IntakeZone.high){
            return ArmZoneHandoff.armUpNotCenterIntakeUp;
        }
        else if (determineArmZoneForHandoff() == ArmZoneForHandoff.downCenter && determineIntakeZone() == IntakeZone.high){
            return ArmZoneHandoff.armDownCenterIntakeUp;
        }
        else if (determineArmZoneForHandoff() == ArmZoneForHandoff.downNotCenter && determineIntakeZone() == IntakeZone.low){
            return ArmZoneHandoff.armDownNotCenterIntakeDown;
        }
        else if (determineArmZoneForHandoff() == ArmZoneForHandoff.downCenter && determineIntakeZone() == IntakeZone.low){
            return ArmZoneHandoff.armDownCenterIntakeDown;
        }
        else if (determineArmZoneForHandoff() == ArmZoneForHandoff.handoff && determineIntakeZone() == IntakeZone.handoff){
            return ArmZoneHandoff.handoff;
        }
        else{
            return ArmZoneHandoff.none;
        }

    }

    

    //commander.getArmPosition is the commanded position using the joysticks in the teleop commander.
    //armTargetPrevious is set to commander.getarmposition at the end of the teleop action.

    public void action(RobotCommander commander) {
        
        intake.IntakePeriodic(commander);

        if(Math.abs(elbow.getElbowAngle()) < 10){
            useNegativeSide = commander.useNegativeSide();
        }

        if (commander.getArmPosition() == ArmPos.manual) {
            actualCommand = ArmPos.manual;
            transitionStateInProgress = false;
        } else if (commander.getArmPosition() == ArmPos.Zero) {
            elbow.setMotorCommand(0.0);
            extension.setMotorCommand(0.0);
            shoulder.setMotorCommand(0.0);
            actualCommand = ArmPos.Zero;
            transitionStateInProgress = false;
        } 
        //If the commanded position is != the previously set position
        else if (commander.getArmPosition() != armTargetPrevious) {
            if (useNegativeSide) {

                currentCommandedZone = this.determineArmZone(-commander.getArmPosition().getShoulder(), 
                                                             commander.getArmPosition().getExtension(), 
                                                             -commander.getArmPosition().getElbow());
            } else {
                currentCommandedZone = this.determineArmZone(commander.getArmPosition().getShoulder(), 
                                                             commander.getArmPosition().getExtension(), 
                                                             commander.getArmPosition().getElbow());
            }

            //Final stage of the command line. If the commandedzone is equal to the currentzone 
            //we aren't transitioning and the actual command to the motors is set to the command from the joystick.
            if (currentCommandedZone == currentZone) {
                actualCommand = commander.getArmPosition();
                transitionStateInProgress = false;
            } 
            
            //Next else if happens when the robot isn't in it's final desired zone. This is for the positive side if the arm is in the hopper.
            else if (currentCommandedZone == ArmZone.postive && currentZone == ArmZone.hopper) {

                //First intermediary step is set for the human player final position.
                if (commander.getArmPosition() == ArmPos.humanPlayerPickup || commander.getArmPosition() == ArmPos.humanPlayerReady) {
                    actualCommand = ArmPos.outOfHumanPlayerInitialExtension;
                }
                
                //else if it's not going to the human player it will go to the generic out of hopper direction.
                else {
                    actualCommand = ArmPos.outOfHopperToDirection;
                }
                transitionStateInProgress = true;
            } 


            
            //This happens if the arm is in the positive zone and wants to go back to the hopper.
            else if (currentCommandedZone == ArmZone.hopper && currentZone == ArmZone.postive) {
                if (extension.getExtensionPosition() > 15 && shoulder.getShoulderAngle() < 15) {
                    actualCommand = ArmPos.outOfReturnFromHumanPlayer;
                } else {
                    actualCommand = ArmPos.outOfDirectionToHopper1;
                }
                transitionStateInProgress = true;
            } 
            
            //This happens if we want to go negative and we are in the hopper.
            else if (currentCommandedZone == ArmZone.negative && currentZone == ArmZone.hopper) {
                if (commander.getArmPosition() == ArmPos.humanPlayerPickup || commander.getArmPosition() == ArmPos.humanPlayerReady) {
                    actualCommand = ArmPos.outOfHumanPlayerInitialExtension;

                } else {
                    actualCommand = ArmPos.outOfHopperToDirection;
                }
                transitionStateInProgress = true;
            } 
            
            //his happens if we are negative and want to go back to the hopper.
            else if (currentCommandedZone == ArmZone.hopper && currentZone == ArmZone.negative) {
                if (extension.getExtensionPosition() > 15 && shoulder.getShoulderAngle() > -15) {
                    actualCommand = ArmPos.outOfReturnFromHumanPlayer;
                } else {
                    actualCommand = ArmPos.outOfDirectionToHopper1;
                }
                transitionStateInProgress = true;
            }
        } 
        
        //If the command stays the same since last loop
        else if (commander.getArmPosition() == armTargetPrevious ) {
            if (transitionStateInProgress) {
                if (achivedPostion) {
                    if (actualCommand == ArmPos.outOfDirectionToHopper1) {
                        actualCommand = ArmPos.outOfPostiveToHopper2;
                        transitionStateInProgress = true;
                    } else if (actualCommand == ArmPos.outOfHopperToDirection && 
                               (commander.getArmPosition() == ArmPos.middleNodeCone || commander.getArmPosition() == ArmPos.topNodeCone
                               || commander.getArmPosition() == ArmPos.middleNodeCube || commander.getArmPosition() == ArmPos.topNodeCube
                                || commander.getArmPosition() == ArmPos.lowerNode)) {
                                actualCommand = ArmPos.outOfHopperToMid;
                                transitionStateInProgress = true;
                    } else {
                        actualCommand = commander.getArmPosition();
                        transitionStateInProgress = true;
                    }
                }
            }
        }
        SmartDashboard.putString("Commanded Position", commander.getArmPosition().name());
        SmartDashboard.putString("Commanded Position Actual", actualCommand.name());
        SmartDashboard.putString("Commanded Zone", currentCommandedZone.name());
        SmartDashboard.putString("Commanded Position Previuos", commander.getArmPosition().name());
        armTargetPrevious = commander.getArmPosition();
        
        double shoulderBump = this.determineShoulderBump(commander);

        if (actualCommand != ArmPos.Zero && actualCommand != ArmPos.manual) {
            if (useNegativeSide) {
                shoulder.goToPostion(commander,-actualCommand.getShoulder() + shoulderBump);
                extension.goToPostion(actualCommand.getExtension());
                elbow.goToPostion(-actualCommand.getElbow());
            } else {
                shoulder.goToPostion(commander, actualCommand.getShoulder() + shoulderBump);
                extension.goToPostion(actualCommand.getExtension());
                elbow.goToPostion(actualCommand.getElbow());
            }
        } else if (actualCommand == ArmPos.manual) {
            elbow.setMotorCommand(commander.armElbow());
            extension.setMotorCommand(commander.armExtension());
            shoulder.setMotorCommand(commander.armShoulder());
        } else {
            shoulder.setMotorCommand(0);
            elbow.setMotorCommand(0);
            extension.setMotorCommand(0);
        }

        if(commander.getArmPosition() == ArmPos.intakeConeGrab){

            if(determineZoneHandoff() == ArmZoneHandoff.armDownNotCenterIntakeDown){
                if(extension.getExtensionPosition() < 20.5){
                desiredIntakePos = 120;
                extension.goToPostion(21);
                shoulder.goToPostion(commander, 0);
                elbow.goToPostion(0);
                this.setIntakeSpeedPos(desiredIntakePos,0,0);
                }
                else{
                desiredIntakePos = 90;
                extension.goToPostion(21);
                shoulder.goToPostion(commander, 0);
                elbow.goToPostion(0);
                this.setIntakeSpeedPos(desiredIntakePos,0,0);
                }
            }
            else if(determineZoneHandoff() == ArmZoneHandoff.armDownCenterIntakeDown){
                    desiredIntakePos = 90;
                    extension.goToPostion(21);
                    shoulder.goToPostion(commander, 0);
                    elbow.goToPostion(0);
                    this.setIntakeSpeedPos(desiredIntakePos,0,0);
                    

            }
            //if the arm is up and not center, move intake down to let the arm center.
            //if the arm is up and center, the intake will move into position with the arm
            else if(determineZoneHandoff() == ArmZoneHandoff.armUpNotCenterIntakeUp){
            if(Math.abs(shoulder.getShoulderAngle()) > 3 || Math.abs(elbow.getElbowAngle()) > 3){
                desiredIntakePos = 120;
                extension.goToPostion(21);
                shoulder.goToPostion(commander, 0);
                elbow.goToPostion(0);
                this.setIntakeSpeedPos(desiredIntakePos,0,0);
            }
                else{
                desiredIntakePos = 91;
                extension.goToPostion(21);
                shoulder.goToPostion(commander, 0);
                elbow.goToPostion(0);
                this.setIntakeSpeedPos(desiredIntakePos,0,0);
                }
                
            }
            else if(determineZoneHandoff() == ArmZoneHandoff.armUpCenterIntakeUp){
                desiredIntakePos = 91;
                extension.goToPostion(21);;
                shoulder.goToPostion(commander, 0);
                elbow.goToPostion(0);
                this.setIntakeSpeedPos(desiredIntakePos,0,0);
                
            }
            else if(determineZoneHandoff() == ArmZoneHandoff.armUpNotCenterIntakeDown){
                desiredIntakePos = 91;
                extension.goToPostion(21);
                shoulder.goToPostion(commander, 0);
                elbow.goToPostion(0);
                this.setIntakeSpeedPos(desiredIntakePos,0,0);
                
            }
            else if(determineZoneHandoff() == ArmZoneHandoff.armUpCenterIntakeDown){
                desiredIntakePos = 91;
                extension.goToPostion(21);
                shoulder.goToPostion(commander, 0);
                elbow.goToPostion(0);
                this.setIntakeSpeedPos(desiredIntakePos,0,0);
                
            }
            else if(determineZoneHandoff() == ArmZoneHandoff.armDownNotCenterIntakeUp){
                desiredIntakePos = 120;
                if(intake.angleEncoder.getAbsolutePosition() > 90){
                extension.goToPostion(0);
                shoulder.goToPostion(commander, 0);
                elbow.goToPostion(0);
                this.setIntakeSpeedPos(desiredIntakePos,0,0);
                }
                else if (intake.angleEncoder.getAbsolutePosition() <= 90 && extension.getExtensionPosition() < 20.5){
                desiredIntakePos = 120;
                extension.goToPostion(21);
                shoulder.goToPostion(commander, 0);
                elbow.goToPostion(0);
                this.setIntakeSpeedPos(desiredIntakePos,0,0);
                }
                else{
                desiredIntakePos = 91;
                extension.goToPostion(21);
                shoulder.goToPostion(commander, 0);
                elbow.goToPostion(0);
                this.setIntakeSpeedPos(desiredIntakePos,0,0);
                }
                
            }
            else if(determineZoneHandoff() == ArmZoneHandoff.armDownCenterIntakeUp){
                if(intake.angleEncoder.getAbsolutePosition() > 90){
                    desiredIntakePos = 120;
                    extension.goToPostion(0);
                    shoulder.goToPostion(commander, 0);
                    elbow.goToPostion(0);
                    this.setIntakeSpeedPos(desiredIntakePos,0,0);
                    }
                    else if (intake.angleEncoder.getAbsolutePosition() <= 90 && extension.getExtensionPosition() < 20.5){
                    desiredIntakePos = 120;
                    extension.goToPostion(21);
                    shoulder.goToPostion(commander, 0);
                    elbow.goToPostion(0);
                    this.setIntakeSpeedPos(desiredIntakePos,0,0);
                    }
                    else{
                    desiredIntakePos = 91;
                    extension.goToPostion(21);
                    shoulder.goToPostion(commander, 0);
                    elbow.goToPostion(0);
                    this.setIntakeSpeedPos(desiredIntakePos,0,0);
                    }
                
            }
            else if(determineZoneHandoff() == ArmZoneHandoff.handoff){
                desiredIntakePos = 91;
                extension.goToPostion(21);
                shoulder.goToPostion(commander, 0);
                elbow.goToPostion(0);
                this.setIntakeSpeedPos(desiredIntakePos,0,0);
                
            }

        }
        
        SmartDashboard.putNumber("intake commanded pos", shoulderBump);
        HotLogger.Log("determineArmZoneHandOff", this.determineZoneHandoff().toString());
        HotLogger.Log("DesiredIntakeAngle",desiredIntakePos);

    }

    public void setIntakeSpeedPos(double intakeAngle, double intakeSpeed1, double intakeSpeed2){
        if(intakeAngle == 0){
            intake.angleMotor.set(0);
        } else {
            double ourAngle = intake.angleEncoder.getAbsolutePosition();
            //double ourAngle = 180;
            double change = intake.pidController.calculate(ourAngle, intakeAngle);
            SmartDashboard.putNumber("Change", change);
            if(Math.abs(ourAngle - intakeAngle) > .2){
                intake.angleMotor.set(change);
            } else {
                intake.angleMotor.set(0);
            }
        }

        SmartDashboard.putNumber("intakeAngle", intakeAngle);
        intake.speedMotor1.set(TalonFXControlMode.PercentOutput, -intakeSpeed1);
    }

    private double determineShoulderBump(RobotCommander commander) {
        double negativeModifier;
        if (commander.useNegativeSide()) {
            negativeModifier = -1;
        } else {
            negativeModifier = 1;
        }

        double humanPlayerModifier = 1;
        if (commander.getArmPosition() == ArmPos.humanPlayerReady || commander.getArmPosition() == ArmPos.humanPlayerPickup) {
            humanPlayerModifier = -1;
        }
        
        if (commander.getArmPosition() == ArmPos.Zero && 
            (armTargetPrevious != ArmPos.intake && armTargetPrevious != ArmPos.packagePos && armTargetPrevious != ArmPos.manual)) {
            bumpLatchCommand = armTargetPrevious;
            if (bumpLatchTimer >= Constants.ARM_BUMP_LATCH_TIME) {
                shoulderBumpOffSet = 0.0;
                bumpLatchTimer = Constants.ARM_BUMP_LATCH_TIME + 1;
            } else {
                bumpLatchTimer++;
            }
        } else if (commander.getArmPosition() != ArmPos.Zero && 
                   commander.getArmPosition() != ArmPos.intake && 
                   commander.getArmPosition() != ArmPos.packagePos && 
                   commander.getArmPosition() != ArmPos.manual) {
            shoulderBumpOffSet+=commander.getArmBumpDirection().getShoulder();
            bumpLatchTimer = 0;
        } else {
            shoulderBumpOffSet = 0.0;
            bumpLatchTimer = Constants.ARM_BUMP_LATCH_TIME + 1;
        }

        SmartDashboard.putNumber("shoulderBumpOffSet", shoulderBumpOffSet);
        SmartDashboard.putNumber("bumpLatchTimer", bumpLatchTimer);

        return shoulderBumpOffSet * humanPlayerModifier * negativeModifier;
    }



    public void updatePose(){
        extension.updatePose();
        elbow.updatePose();
        shoulder.updatePose();
        achivedPostion = this.determineAchivePosition();
        currentZone = this.determineArmZone(shoulder.getShoulderAngle(), extension.getExtensionPosition(), elbow.getElbowAngle());
        SmartDashboard.putString("ArmActualZone",currentZone.name());
        SmartDashboard.putBoolean("Achived Position",achivedPostion);
    }

    private boolean determineAchivePosition() {
        return shoulder.getAchivedTarget() && extension.getAchivedTarget() && elbow.getAchivedTarget();
    }

    public void brakeMode(){
        shoulder.setBreakMode();
        extension.setBreakMode();
        elbow.setBreakMode();
        intake.setBrakeMode();
    }

    public void coastMode(){
        shoulder.setCoastMode();;
        extension.setCoastMode();
        elbow.setCoastMode();
        intake.setCoastMode();
    }
    public void logdata(){
        intake.logData();
    }

}