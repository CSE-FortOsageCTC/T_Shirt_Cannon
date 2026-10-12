package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.AutoRotateUtil;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase{
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    private static Swerve swerve;
    public double gyroOffset;
    public PIDController rotationPidController;
    public PIDController yTranslationPidController;
    public PIDController xTranslationPidController;
    public AutoRotateUtil s_AutoRotateUtil;
    public Debouncer pieceSeenDebouncer;
    public Field2d field;
    public StructPublisher<Pose3d> publisher;
    public StructArrayPublisher<Pose3d> arrayPublisher;
    public Pose3d poseA = new Pose3d();
    public Pose3d poseB = new Pose3d();

    public boolean readyToPickUp = false;
    public DriveParams autoPickupDriveParams;

    public static Swerve getInstance() {
        if (swerve == null) {
            swerve = new Swerve();
        }
        return swerve;
    }

    private Swerve() {
        pieceSeenDebouncer = new Debouncer(.1, Debouncer.DebounceType.kBoth);
        s_AutoRotateUtil = new AutoRotateUtil(0);
        field = new Field2d();
        
        publisher = NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose3d.struct).publish();
        arrayPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("MyPoseArray", Pose3d.struct).publish();

        
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation,
                                    null // getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    public void autoDrive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        if (readyToPickUp) {
            drive(autoPickupDriveParams.translation, autoPickupDriveParams.rotation, autoPickupDriveParams.fieldRelative, autoPickupDriveParams.isOpenLoop);
        } else {
            drive(translation, rotation, fieldRelative, isOpenLoop);
        }
        System.out.println(readyToPickUp);
    }

    public void setAutoDriveParams(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        this.autoPickupDriveParams = new DriveParams(translation, rotation, fieldRelative, isOpenLoop);
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }


    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }
    public ChassisSpeeds getRobotRelativeSpeeds(){
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }
    

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds){

        robotRelativeSpeeds.omegaRadiansPerSecond *= -1;

        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, .02);
        SwerveModuleState[] setpointStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, Constants.Swerve.maxSpeed);

        for(int i = 0; i < 4; i++){
            mSwerveMods[i].setDesiredState(setpointStates[i], false);
        }
    }

    public void resetAutoRotateUtil(){
        s_AutoRotateUtil.end();
    }
    
    @Override
    public void periodic(){
        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
        }
    }

    public class DriveParams {

        public Translation2d translation;
        public double rotation;
        public boolean fieldRelative;
        public boolean isOpenLoop;

        public DriveParams(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop){
            this.translation = translation;
            this.rotation = rotation;
            this.fieldRelative = fieldRelative;
            this.isOpenLoop = isOpenLoop;
        }
    }
}