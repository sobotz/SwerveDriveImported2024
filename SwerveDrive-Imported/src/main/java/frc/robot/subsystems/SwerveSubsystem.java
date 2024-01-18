// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
//import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderJNI;

//import com.ctre.phoenix.sensors.CANCoder;
//import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
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

  CANCoderJNI fLSensor;
  CANCoderJNI fRSensor;
  CANCoderJNI bLSensor;
  CANCoderJNI bRSensor;
  WPI_PigeonIMU pigeon;

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
  double directionCorrectorValue;

  Vector frontLeftVector;
  Vector frontRightVector;
  Vector backLeftVector;
  Vector backRightVector;
  Vector driveVector;

  boolean isRotating;
  double originalInstance;

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
    
    instance = true;
    directionCorrectorValue = 0;
    fLSensor = new CANCoderJNI();//24
    fLSensor.Create(24, getName());
    fRSensor = new CANCoderJNI();//21
    fRSensor.Create(21, getName());
    bLSensor = new CANCoderJNI();//23
    bLSensor.Create(23, getName());
    bRSensor = new CANCoderJNI();//22
    bRSensor.Create(22, getName());
    frontLeftModule = new SwerveModule(frontLeftDriver, frontLeftTurner,fLSensor);
    frontRightModule = new SwerveModule(frontRightDriver, frontRightTurner,fRSensor);
    backLeftModule = new SwerveModule(backLeftDriver, backLeftTurner,bLSensor);
    backRightModule = new SwerveModule(backRightDriver, backRightTurner,bRSensor);
    pigeon = new WPI_PigeonIMU(25);//SET RIGHT DEVICE NUMBER
    once = true;
    strafeMagnatude = 0;
    strafeDirection = 0;

    combinedMagnatude = 0;
    combinedDirection = 0;
    originalDegree = 0;
    degreeOffset = 0;
    isRotating = true;
    targetRobotDegree = 0;
    originalDegree = 0;
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
      rotationalMagnatude = directionCorrectorValue * 0.5; //AAAAAAAAAAAAAAAAAAAAAAAAA
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
  public double getAbsoluteRotation(){
    return pigeon.getAbsoluteCompassHeading();//WHCH WAY DOES IT ROTATE
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
  @Override
  public void periodic() {
    if (once){
      originalDegree = pigeon.getAngle();
      originalDegree =originalDegree%360;
      if (originalDegree < 0){
        originalDegree += 360;
      }
      once = false;
    }
    degreeOffset = pigeon.getAngle();
    degreeOffset-= originalDegree;
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
