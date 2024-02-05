// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
//import com.ctre.phoenix.sensors.CANCoder;

import com.ctre.phoenix6.hardware.CANcoder;
//import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  SwerveModule frontLeftModule;
  SwerveModule frontRightModule;
  SwerveModule backLeftModule;
  SwerveModule backRightModule;

  CANSparkMax frontLeftDriver;
  CANSparkMax frontLeftTurner;
  CANSparkMax frontRightDriver;
  CANSparkMax frontRightTurner;
  CANSparkMax backLeftDriver;
  CANSparkMax backLeftTurner;
  CANSparkMax backRightDriver;
  CANSparkMax backRightTurner;

  CANcoder fLSensor;
  CANcoder fRSensor;
  CANcoder bLSensor;
  CANcoder bRSensor;
  Pigeon2 pigeon;

  double strafeMagnatude;
  double strafeDirection;

  double rotationalMagnatude;
  double rotationalDirection;

  double combinedMagnatude;
  double combinedDirection;
  
  double targetRobotDegree;

  double originalDegree;
  double degreeOffset;
  
  boolean once;
  boolean instance;
  
  PIDController directionCorrector;
  PIDController rotateToDegreeController;
  PIDController yDisplacementController;
  PIDController xDisplacementController;
  double yDisplacement;
  double xDisplacement;
  double yAcceleration;
  double xAcceleration;
  double directionCorrectorValue;

  Vector frontLeftVector;
  Vector frontRightVector;
  Vector backLeftVector;
  Vector backRightVector;
  Vector driveVector;

  boolean isRotating;
  double originalInstance;
  Timer timer;
  double lastTime;
  double currentTime;
  double xiVelocity;
  double xfVelocity;
  double yiVelocity;
  double yfvelocity;
  double dx;
  double dy;
  boolean highSpeed;
  double strafeMultiplier;
  SwerveOdometer odometer;


  public SwerveSubsystem(){
    //Change device ID to the normal orientation
    frontLeftDriver = new CANSparkMax(8,MotorType.kBrushless);
    frontLeftTurner = new CANSparkMax(7,MotorType.kBrushless);
    frontRightDriver = new CANSparkMax(2,MotorType.kBrushless);
    frontRightTurner = new CANSparkMax(1,MotorType.kBrushless);
    backLeftDriver = new CANSparkMax(6,MotorType.kBrushless);
    backLeftTurner = new CANSparkMax(5,MotorType.kBrushless);
    backRightDriver = new CANSparkMax(4,MotorType.kBrushless);
    backRightTurner = new CANSparkMax(3,MotorType.kBrushless);
    directionCorrector = new PIDController(Constants.SwerveDriveConstants.rKP,Constants.SwerveDriveConstants.rKI,Constants.SwerveDriveConstants.rKD);
    rotateToDegreeController = new PIDController(/*Constants.SwerveDriveConstants.rKP*/0.007,Constants.SwerveDriveConstants.rKI,Constants.SwerveDriveConstants.rKD);
    directionCorrector.enableContinuousInput(0, 360);
    directionCorrector.setTolerance(0.01);
    rotateToDegreeController.enableContinuousInput(0,360);
    rotateToDegreeController.setTolerance(0.006);
    yDisplacementController = new PIDController(0,0,0);
    xDisplacementController = new PIDController(0,0,0);
    yDisplacementController.setTolerance(0.003);
    xDisplacementController.setTolerance(0.003);
    yDisplacement = 0;
    xDisplacement = 0;
    yAcceleration = 0;
    xAcceleration = 0;
    timer = new Timer();
    xiVelocity = 0;
    xfVelocity = 0;
    strafeMultiplier = 0;
    

    

    
    instance = true;
    directionCorrectorValue = 0;
    fLSensor = new CANcoder(24);//24
 
    fRSensor = new CANcoder(21);//21

    bLSensor = new CANcoder(23);//23

    bRSensor = new CANcoder(22);//22
    frontLeftModule = new SwerveModule(frontLeftDriver, frontLeftTurner,fLSensor);
    frontRightModule = new SwerveModule(frontRightDriver, frontRightTurner,fRSensor);
    backLeftModule = new SwerveModule(backLeftDriver, backLeftTurner,bLSensor);
    backRightModule = new SwerveModule(backRightDriver, backRightTurner,bRSensor);
    pigeon = new Pigeon2(25);//SET RIGHT DEVICE NUMBER
    once = true;
    strafeMagnatude = 0;
    strafeDirection = 0;
    highSpeed = false;

    combinedMagnatude = 0;
    combinedDirection = 0;
    originalDegree = 0;
    degreeOffset = 0;
    isRotating = true;
    targetRobotDegree = 0;
    originalDegree = 0;
    lastTime = 0;
    currentTime = 0;
    dx = 0;
    dy = 0;
    odometer = new SwerveOdometer();
  }

  public void drive (double strafeMagnatude1,double strafeDirection1,double rotationalMagnatude1){
    this.strafeMagnatude = strafeMagnatude1 * 0.5;
    this.strafeDirection = strafeDirection1 - degreeOffset;
    strafeDirection = strafeDirection % 360;
    if (strafeDirection <0){
      strafeDirection += 360;
    }
    
    if (rotationalMagnatude1 != 0){
      isRotating = true;
      this.rotationalMagnatude = rotationalMagnatude1 * 0.5;
    }
    else{
      isRotating = false;
      rotationalMagnatude = directionCorrectorValue * 0.5;
    }
    if (strafeMagnatude != 0 && rotationalMagnatude !=0){
      driveVector = new Vector(strafeMagnatude,strafeDirection, true);
      //Rotational Vectors
      if (rotationalMagnatude<0){
        rotationalMagnatude = Math.abs(rotationalMagnatude);
        frontLeftVector = new Vector(rotationalMagnatude,(135+180) % 360,true);
        frontRightVector = new Vector(rotationalMagnatude,(45+180) % 360,true);
        backLeftVector = new Vector(rotationalMagnatude,(225+180) % 360,true);
        backRightVector = new Vector(rotationalMagnatude,(315+180) % 360,true);
      }
      else{
        frontLeftVector = new Vector(rotationalMagnatude,135,true);
        frontRightVector = new Vector(rotationalMagnatude,(45),true);
        backLeftVector = new Vector(rotationalMagnatude,225,true);
        backRightVector = new Vector(rotationalMagnatude,315,true);
      }
      frontLeftVector = frontLeftVector.addVector(driveVector);
      frontRightVector = frontRightVector.addVector(driveVector);
      backLeftVector = backLeftVector.addVector(driveVector);
      backRightVector = backRightVector.addVector(driveVector);

      frontLeftModule.drive(frontLeftVector.getMagnatude(),frontLeftVector.getDegree());
      frontRightModule.drive(-frontRightVector.getMagnatude(),frontRightVector.getDegree());
      //System.out.println(frontRightVector.getDegree());
      backLeftModule.drive(backLeftVector.getMagnatude(),backLeftVector.getDegree());
      backRightModule.drive(backRightVector.getMagnatude(),backRightVector.getDegree());
    }
    else if (strafeMagnatude !=0 && rotationalMagnatude == 0){
      frontLeftModule.drive(strafeMagnatude,strafeDirection);
      frontRightModule.drive(-strafeMagnatude,strafeDirection);
      backLeftModule.drive(strafeMagnatude,strafeDirection);
      backRightModule.drive(strafeMagnatude,strafeDirection);
      
    }
    else if (rotationalMagnatude != 0){
      if (rotationalMagnatude<0){
        rotationalMagnatude = Math.abs(rotationalMagnatude);
        frontLeftModule.drive(-rotationalMagnatude,135);
        frontRightModule.drive(rotationalMagnatude,45);
        backLeftModule.drive(-rotationalMagnatude,225);
        backRightModule.drive(-rotationalMagnatude,315);
      }
      else{
        frontLeftModule.drive(rotationalMagnatude,135);
        frontRightModule.drive(-rotationalMagnatude,45);
        backLeftModule.drive(rotationalMagnatude,225);
        backRightModule.drive(rotationalMagnatude,315);
      }
      
    }
    
  }
  public void autoDrive(double xfeet,double yfeet,double rotation){
    double xMeters = xfeet/3.28084;
    double yMeters = yfeet/3.28084;
    double xCalculate = xDisplacementController.calculate(dx,xMeters);
    double yCalculate = yDisplacementController.calculate(dy,yMeters);
    Vector vector = new Vector(xCalculate,yCalculate);
    double rCalculate = driveToDegree((getDegreeOffset()*-1),rotation);
    drive(vector.getMagnatude(),vector.getDegree(),rCalculate);
  }
  public double getAbsoluteRotation(){
    //return pigeon.getAbsoluteCompassHeading();//WHCH WAY DOES IT ROTATE
    return pigeon.getAngle();
  }
  
  public double driveToDegree(double currentDegree, double targetDegree){
    double value = 0;
    rotateToDegreeController.calculate(currentDegree,targetDegree);
    if (!rotateToDegreeController.atSetpoint()){
      value = rotateToDegreeController.calculate(currentDegree,targetDegree);
    }
    if (value > 1){
      value = 1;
    }
    else if (value < -1){
      value = -1;
    }
    return value;
  }
  public double getDegreeOffset(){
    double v = (pigeon.getAngle() - originalDegree) % 360;
    if (v < 0){
      v += 360;
    }
    return v;
  }
  public void toggleChangeSpeed(){
    highSpeed = !highSpeed;
    if (highSpeed){
      strafeMultiplier = 0.90;
    }
    else{
      strafeMultiplier = 0.5;
    }
  }
  public double getXDisplacement(){
    return xDisplacement;
  }
  public double getYDisplacement(){
    return yDisplacement;
  }
  @Override
  public void periodic() {
    if (once){
      originalDegree = pigeon.getAngle();
      originalDegree =originalDegree%360;
      if (originalDegree < 0){
        originalDegree += 360;
      }
      timer.start();
      once = false;
      
    }
    //System.out.println(pigeon.getAngle());
    degreeOffset = pigeon.getAngle()-originalDegree;
    degreeOffset = degreeOffset % 360;
    if (degreeOffset < 0){
      degreeOffset += 360;
    }
    degreeOffset = -degreeOffset;
    degreeOffset += 360;
    degreeOffset = degreeOffset % 360;
    
    if (!isRotating){
      if (instance){
        originalInstance = degreeOffset;
        instance = false;
      }
      directionCorrector.calculate(degreeOffset,originalInstance);
      if (!directionCorrector.atSetpoint()){
        directionCorrectorValue = directionCorrector.calculate(degreeOffset,originalInstance);
        if (directionCorrectorValue > 1){
          directionCorrectorValue = 1;
        }
        else if (directionCorrectorValue < -1){
          directionCorrectorValue = -1;
        }
      }
    }
    else{
      instance = true;
    }  
    /*if (timer.get()>.25){
      double xA = pigeon.getAccelerationY().getValueAsDouble();
      double yA = pigeon.getAccelerationX().getValueAsDouble()-0.03;
      if (yA>-0.0035 && yA < 0.0035){//0.0035
        yA = 0;
      }
      if (yA<0){
        System.out.println("!!!!");
      }
      xAcceleration = xA * 9.80665;
      yAcceleration = yA* 9.80665;
      
      dx += (xiVelocity * timer.get()) + ((1.0/2.0) * xAcceleration * Math.pow(timer.get(),2));
      xiVelocity += xAcceleration * timer.get();
      dy += (yiVelocity * timer.get()) + ((1.0/2.0) * yAcceleration * Math.pow(timer.get(),2));
      yiVelocity += yAcceleration * timer.get();
      
      SmartDashboard.putNumber("TIME",timer.get());
      SmartDashboard.putNumber("y Acceleration",yAcceleration);
      SmartDashboard.putNumber("y distance",dy);
      
      timer.reset();
      timer.start();
    }*/
    //Vector flm = new Vector()

    
    
    //System.out.println(xiVelocity);
    //SmartDashboard.putNumber("MODULE CALCULATE:",frontLeftModule.getCalculate());
    SmartDashboard.putNumber("TARGET STRAFE DEGREE:",strafeDirection);
    SmartDashboard.putNumber("CURRENT MODULE DEGREE", frontLeftModule.getAbsolutePosition());
    SmartDashboard.putNumber("Current Robot Degree", degreeOffset);
    SmartDashboard.putNumber("front left speed",frontLeftModule.getSpeed());
    SmartDashboard.putNumber("front Right speed",frontRightModule.getSpeed());
    //SmartDashboard.putNumber("Current Inverted: ", frontLeftModule.getInvertedAbsolutePosition());
    //SmartDashboard.putNumber("Current Rotational Magnatude:", rotationalMagnatude);
    
  }
}
