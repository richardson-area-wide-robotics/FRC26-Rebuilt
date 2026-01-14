package frc.robot.kadiri;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import frc.robot.common.components.EasyMotor;

public class Shooter {

    SparkFlex motor = EasyMotor.createEasySparkFlex(-1, SparkLowLevel.MotorType.kBrushless, SparkBaseConfig.IdleMode.kBrake);

}
