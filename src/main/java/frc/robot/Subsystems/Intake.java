// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GlobalVariables;

public class Intake extends SubsystemBase {
  CANSparkMax frontRoller;
  CANSparkMax backRoller;

  TalonFX retractor;

  /** Creates a new Intake. */
  public Intake() {
    this.frontRoller = new CANSparkMax(Constants.IntakeConstants.FRONT_INTAKE_ROLLER, MotorType.kBrushless);
    frontRoller.restoreFactoryDefaults();
    frontRoller.setIdleMode(IdleMode.kCoast);
    frontRoller.setSmartCurrentLimit(20);
    frontRoller.setInverted(true);
    frontRoller.enableVoltageCompensation(10);
    frontRoller.setOpenLoopRampRate(0.1);
    frontRoller.setClosedLoopRampRate(0.1);
    this.backRoller = new CANSparkMax(Constants.IntakeConstants.BACK_INTAKE_ROLLER, MotorType.kBrushless);
    backRoller.restoreFactoryDefaults();
    backRoller.setIdleMode(IdleMode.kCoast);
    backRoller.setSmartCurrentLimit(20);
    backRoller.setInverted(true);
    backRoller.enableVoltageCompensation(10);
    backRoller.setOpenLoopRampRate(0.1);
    backRoller.setClosedLoopRampRate(0.1);

    this.retractor = new TalonFX(Constants.IntakeConstants.INTAKE_RETRACTION);
    retractor.configFactoryDefault();
    retractor.setNeutralMode(NeutralMode.Coast);
    retractor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 30, 0.5));
    retractor.configVoltageCompSaturation(10);
    retractor.enableVoltageCompensation(true);
    retractor.configOpenloopRamp(0.1);
    retractor.configClosedloopRamp(0.1);
    //retractor.configNominalOutputForward(0.0);
    //retractor.configNominalOutputReverse(0.0);
    //retractor.configPeakOutputForward(0.0);
    //retractor.configPeakOutputReverse(0.0);
    retractor.config_kP(0, 0.02);
    retractor.config_kI(0, 0.0);
    retractor.config_kD(0, 0.0);
    retractor.config_kF(0, 0.0);
    retractor.configMotionAcceleration(0.0);
    retractor.configMotionCruiseVelocity(0.0);
    setIntakeEncoder(0.0);
    intakeStop();
    setIntakeAnglePID(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("IntakeEncoder", getIntakeEncoder());
  }

  public void intakeSpeed(double frontSpeed, double backSpeed) {
    frontRoller.set(frontSpeed);
    backRoller.set(backSpeed);
  }

  public void intakeStop() {
    frontRoller.stopMotor();
    backRoller.stopMotor();
  }

  public void retractorStop() {
    retractor.set(ControlMode.PercentOutput, 0.0);
  }

  public double getIntakeRetractorPosition() {
    return retractor.getSelectedSensorPosition();
  }

  public void setIntakeAnglePID(double angle) {
    retractor.set(ControlMode.Position, angle);
  }

  public void setIntakeAngleMotionMagic(double angle) {
    retractor.set(ControlMode.MotionMagic, angle);
  }

  public void setIntakeEncoder(double angle) {
    retractor.setSelectedSensorPosition(angle);
  }

  public double getIntakeEncoder() {
    return retractor.getSelectedSensorPosition();
  }

  public boolean isIntakeDown() {
    if (retractor.getSelectedSensorPosition() >= -20000) {
      GlobalVariables.INTAKE_LOWERED = false;
    }else{
      GlobalVariables.INTAKE_LOWERED = true;
    }
    return GlobalVariables.INTAKE_LOWERED;
  }
}
