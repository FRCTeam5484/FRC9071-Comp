package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private static final String kDoNothing = "DoNothing";
  private static final String kCrossLineAuto = "CrossLine";
  private static final String kCubeAuto = "CubeCrossLine";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  Timer time = new Timer();

  // Variables
  Double drivePowerFactor = 0.5;
  Double armPower = 0.25;
  Double intakePower = 0.25;

  // Drive Objects
  CANSparkMax driveLeftFrontSpark = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax driveLeftRearSpark = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax driveRightFrontSpark = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax driveRightRearSpark = new CANSparkMax(4, MotorType.kBrushless);

  DifferentialDrive m_Drivetrain;

  // Arm Objects
  CANSparkMax arm = new CANSparkMax(7, MotorType.kBrushless);
  RelativeEncoder armEncoder = arm.getEncoder();  

  // Intake Objects
  CANSparkMax intake = new CANSparkMax(6, MotorType.kBrushless);

  // Operators
  PS4Controller DriverController = new PS4Controller(0);
  PS4Controller OpController = new PS4Controller(1);


  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Do Nothing", kDoNothing);
    m_chooser.addOption("Cross Line", kCrossLineAuto);
    m_chooser.addOption("Drop Cube, Cross Line", kCubeAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    CameraServer.startAutomaticCapture();

    ConfigureDrive();
    ConfigureArm();
    ConfigureIntake();

    m_Drivetrain = new DifferentialDrive(driveLeftFrontSpark, driveRightFrontSpark);
  }
  
  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    time.reset();
    time.start();
    m_autoSelected = m_chooser.getSelected();
  }

  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCrossLineAuto:
        autoCrossLine();
        break;
      case kCubeAuto:
        autoCubeCrossLine();
        break;
      default:
        drive(0, 0);
        break;
    }
  }

  @Override
  public void teleopInit() {
    m_Drivetrain.stopMotor();
    armStop();
    intakeStop();
  }

  @Override
  public void teleopPeriodic() {
    // Set Drive to Drive Controller Values
    drive(-MathUtil.applyDeadband(DriverController.getLeftY(), 0.01), -MathUtil.applyDeadband(DriverController.getRightY(), 0.01));

    // Activate Arm when L1 or R2 pressed
    if (OpController.getL1Button()) {
      armLower();
    }  else if (OpController.getR1Button()) {
      armRaise();
    } else {
      arm.stopMotor();
    }

    // Activate Intake when Square or TriangleButton pressed
    if(OpController.getSquareButton()){
      intakeForward();
    }
    else if (OpController.getSquareButton()){
      intakeReverse();
    }
    else{
      intake.stopMotor();
    }
  }

  @Override
  public void disabledInit() {
    m_Drivetrain.stopMotor();
    armStop();
    intakeStop();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  private void ConfigureDrive() {
    // Restore controllers to default
    driveLeftFrontSpark.restoreFactoryDefaults();
    driveLeftRearSpark.restoreFactoryDefaults();
    driveRightFrontSpark.restoreFactoryDefaults();
    driveRightRearSpark.restoreFactoryDefaults();

    // Set Idle Mode
    driveLeftFrontSpark.setIdleMode(IdleMode.kBrake);
    driveLeftRearSpark.setIdleMode(IdleMode.kBrake);
    driveRightFrontSpark.setIdleMode(IdleMode.kBrake);
    driveRightRearSpark.setIdleMode(IdleMode.kBrake);

    // Invert Motors
    driveLeftFrontSpark.setInverted(false);
    driveLeftRearSpark.setInverted(false);
    driveRightFrontSpark.setInverted(true);
    driveRightRearSpark.setInverted(true);

    // Set rear motors to follow front motors
    driveLeftRearSpark.follow(driveLeftFrontSpark);
    driveRightRearSpark.follow(driveRightFrontSpark);
  }

  private void ConfigureArm() {
    // Restore controllers to default
    arm.restoreFactoryDefaults();
    
    // Set Idle Mode
    arm.setIdleMode(IdleMode.kBrake);
    
    // Invert Motors
    arm.setInverted(true);
  }

  private void ConfigureIntake() {
    // Restore controllers to default
    intake.restoreFactoryDefaults();
    
    // Set Idle Mode
    intake.setIdleMode(IdleMode.kCoast);
    
    // Invert Motors
    intake.setInverted(false);
  }

  // ************************************************************
  // Autonomous Methods
  // ************************************************************
  private void autoCrossLine() {
    if(time.hasElapsed(4)){
      driveStop();
    }
    else{
      drive(-0.5, -0.5);
    }
  }

  private void autoCubeCrossLine() {
    if(time.hasElapsed(1.5)){
      armStop();
    }
    else{
      armRaise();
    }

    if(time.hasElapsed(3)){
      intakeStop();
    }
    else{
      intakeReverse();
    }

    if(time.hasElapsed(7)){
      driveStop();
    }
    else{
      drive(-0.5, -0.5);
    }
  }

  // ************************************************************
  // Drive Methods
  // ************************************************************
  private void drive(double leftPower, double rightPower){
    m_Drivetrain.tankDrive(leftPower*drivePowerFactor, rightPower*drivePowerFactor);
  }

  private void driveStop(){
    m_Drivetrain.stopMotor();
  }

  // ************************************************************
  // Arm Methods
  // ************************************************************
  private void armLower(){
    arm.set(armPower);
  }

  private void armRaise(){
    arm.set(-armPower);
  }

  private void armStop(){
    arm.stopMotor();
  }


  // ************************************************************
  // Intake Methods
  // ************************************************************
  private void intakeForward(){
    intake.set(armPower);
  }

  private void intakeReverse(){
    intake.set(-armPower);
  }

  private void intakeStop(){
    intake.stopMotor();
  }
}