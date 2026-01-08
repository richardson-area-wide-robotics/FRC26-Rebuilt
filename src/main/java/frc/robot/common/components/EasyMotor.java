package frc.robot.common.components;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import lombok.experimental.UtilityClass;

@UtilityClass
public class EasyMotor {

    /** Make a {@link SparkMax}
     *
     * @param motorID The CAN Bus ID for this motor
     * @param motorType The type of the motor, Brushless etc
     * @param idleMode The idle mode of the motor, Break etc
     *
     * @author Hudson Strub
     * @since 2025
     */
    public SparkMax createEasySparkMax(int motorID, SparkLowLevel.MotorType motorType, SparkBaseConfig.IdleMode idleMode){
        SparkMaxConfig config = new SparkMaxConfig();
        SparkMax motor;

        motor = new SparkMax(motorID, motorType);
        config.idleMode(idleMode);
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        return motor;
    }

    /** Make a {@link SparkFlex}
     *
     * @param motorID The CAN Bus ID for this motor
     * @param motorType The type of the motor, Brushless etc
     * @param idleMode The idle mode of the motor, Break etc
     *
     * @author Hudson Strub
     * @since 2025
     */
    public SparkFlex createEasySparkFlex(int motorID, SparkLowLevel.MotorType motorType, SparkBaseConfig.IdleMode idleMode){
        SparkFlexConfig config = new SparkFlexConfig();
        SparkFlex motor;

        motor = new SparkFlex(motorID, motorType);
        config.idleMode(idleMode);
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        return motor;
    }


    /** Make a {@link SparkFlex} that follows another motor
     *
     * @param motorID The CAN Bus ID for this motor
     * @param followMotorID The motor to follow
     * @param inverted Is the motor inverted?
     * @param motorType The type of the motor, Brushless etc
     * @param idleMode The idle mode of the motor, Break etc
     *
     * @author Hudson Strub
     * @since 2025
     */
    public SparkFlex createEasySparkFlex(int motorID, int followMotorID, boolean inverted,  SparkLowLevel.MotorType motorType, SparkBaseConfig.IdleMode idleMode){
        SparkFlexConfig config = new SparkFlexConfig();
        SparkFlex motor;

        motor = new SparkFlex(motorID, motorType);
        config.idleMode(idleMode);
        config.follow(followMotorID, inverted);

        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        return motor;
    }

}
