package frc.robot.common.components.dashboard.diagnostics;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.toxicrain.instanceable.BaseInstanceable;

/**
 * Utility class for monitoring CAN bus issues.
 * This does not work in the simulator
 *
 *
 * @author Hudson Strub
 * @since 2025 Offseason
 */
public class CANDiagnostics extends BaseInstanceable<CANDiagnostics> implements IDiagnostic {

    /**
     * Gets the singleton instance we are using for CANDiagnostics.
     */
    public static CANDiagnostics getInstance(){
        return BaseInstanceable.getInstance(CANDiagnostics.class);
    }

    /**
     * Checks CAN bus health using RobotController.getCANStatus().
     */
    @Override
    public void checkHealth() {
        CANStatus status = RobotController.getCANStatus();

        SmartDashboard.putNumber("CAN/Utilization (%)", status.percentBusUtilization * 100.0);
        SmartDashboard.putNumber("CAN/BusOffCount", status.busOffCount);
        SmartDashboard.putNumber("CAN/TxFullCount", status.txFullCount);
        SmartDashboard.putNumber("CAN/RxErrorCount", status.receiveErrorCount);
        SmartDashboard.putNumber("CAN/TxErrorCount", status.transmitErrorCount);

        if (status.percentBusUtilization > 0.85) {
            System.err.println("[CAN WARNING] Bus utilization above 85%: "
                    + String.format("%.1f%%", status.percentBusUtilization * 100.0));
        }
        if (status.busOffCount > 0) {
            System.err.println("[CAN ERROR] Bus-off events detected: " + status.busOffCount);
        }
        if (status.txFullCount > 0) {
            System.err.println("[CAN ERROR] TX buffer full events: " + status.txFullCount);
        }
    }
}
