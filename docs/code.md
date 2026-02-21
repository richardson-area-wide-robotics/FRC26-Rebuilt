# Code Structure Flowchart

```mermaid
flowchart TD
    subgraph frc_robot[frc.robot]
        BuildConstants[BuildConstants]
        CommonConstants[CommonConstants]
        Main[Main]
        Robot[Robot]
    end
    subgraph frc_robot_common_annotations[frc.robot.common.annotations]
        DashboardVariable[DashboardVariable]
        NamedAuto[NamedAuto]
        Robot[Robot]
    end
    subgraph frc_robot_common_components_dashboard[frc.robot.common.components.dashboard]
        DashboardAutoUpdater[DashboardAutoUpdater]
    end
    subgraph frc_robot_common_components_dashboard_diagnostics[frc.robot.common.components.dashboard.diagnostics]
        CANDiagnostics[CANDiagnostics]
    end
    subgraph frc_robot_common_components[frc.robot.common.components]
        EasyBreakBeam[EasyBreakBeam]
        EasyMotor[EasyMotor]
        NamedAutoRegistry[NamedAutoRegistry]
        RobotContainerRegistry[RobotContainerRegistry]
        RobotExceptionHandler[RobotExceptionHandler]
        RobotUtils[RobotUtils]
        TeamUtils[TeamUtils]
    end
    subgraph frc_robot_common_components_hardware[frc.robot.common.components.hardware]
        SwerveHardware[SwerveHardware]
        SwerveHardwareParams[SwerveHardwareParams]
        SwerveModuleHardware[SwerveModuleHardware]
        TankHardware[TankHardware]
    end
    subgraph frc_robot_common[frc.robot.common]
        DefaultContainer[DefaultContainer]
        LocalADStarAK[LocalADStarAK]
    end
    subgraph frc_robot_common_gyro[frc.robot.common.gyro]
        RAWRNavX2[RAWRNavX2]
        RAWRQuestNav[RAWRQuestNav]
    end
    subgraph frc_robot_common_interfaces[frc.robot.common.interfaces]
        IDiagnostic[IDiagnostic]
        IMU[IMU]
        IRobotContainer[IRobotContainer]
    end
    subgraph frc_robot_common_subsystems[frc.robot.common.subsystems]
        DashboardSubsystem[DashboardSubsystem]
        SingleMotorSubsystem[SingleMotorSubsystem]
    end
    subgraph frc_robot_common_subsystems_drive[frc.robot.common.subsystems.drive]
        SwerveDriveSubsystem[SwerveDriveSubsystem]
        TankDriveSubsystem[TankDriveSubsystem]
    end
    subgraph frc_robot_common_subsystems_vision[frc.robot.common.subsystems.vision]
        AssumedPoseSubsystem[AssumedPoseSubsystem]
    end
    subgraph frc_robot_common_swerve[frc.robot.common.swerve]
        RAWRSwerveModule[RAWRSwerveModule]
    end
    subgraph frc_robot_pearce_components[frc.robot.pearce.components]
        HubStatus[HubStatus]
        RobotSector[RobotSector]
        SmartSequentialCommand[SmartSequentialCommand]
        SmartSequentialCommandContainer[SmartSequentialCommandContainer]
    end
    subgraph frc_robot_pearce[frc.robot.pearce]
        PearceConstants[PearceConstants]
        PearceContainer[PearceContainer]
    end
    subgraph frc_robot_pearce_subsystems[frc.robot.pearce.subsystems]
        ProtoClimber[ProtoClimber]
        ProtoFeeder[ProtoFeeder]
        ProtoIntake[ProtoIntake]
        ProtoShooter[ProtoShooter]
    end
    subgraph frc_robot_pearce_subsystems_smart[frc.robot.pearce.subsystems.smart]
        DynamicPather[DynamicPather]
        RobotSectorEvaluator[RobotSectorEvaluator]
        SmartSequentialCommandSequencer[SmartSequentialCommandSequencer]
    end
    BaseInstanceable_CANDiagnostics[BaseInstanceable<CANDiagnostics]
    CANDiagnostics -->|extends| BaseInstanceable_CANDiagnostics
    LoggableHardware[LoggableHardware]
    RAWRNavX2 -->|extends| LoggableHardware
    AutoCloseable[AutoCloseable]
    IMU -->|extends| AutoCloseable
    SubsystemBase[SubsystemBase]
    DashboardSubsystem -->|extends| SubsystemBase
    SwerveDriveSubsystem -->|extends| DashboardSubsystem
    TankDriveSubsystem -->|extends| SubsystemBase
    SingleMotorSubsystem -->|extends| DashboardSubsystem
    AssumedPoseSubsystem -->|extends| SubsystemBase
    SwerveModule[SwerveModule]
    RAWRSwerveModule -->|extends| SwerveModule
    ProtoClimber -->|extends| DashboardSubsystem
    ProtoIntake -->|extends| DashboardSubsystem
    ProtoShooter -->|extends| DashboardSubsystem
    RobotSectorEvaluator -->|extends| DashboardSubsystem
    LoggedRobot[LoggedRobot]
    Robot -->|extends| LoggedRobot
    CANDiagnostics -.implements.-> IDiagnostic
    Thread_UncaughtExceptionHandler[Thread.UncaughtExceptionHandler]
    RobotExceptionHandler -.implements.-> Thread_UncaughtExceptionHandler
    DefaultContainer -.implements.-> IRobotContainer
    RAWRNavX2 -.implements.-> IMU
    org_lasarobotics_hardware_IMU[org.lasarobotics.hardware.IMU]
    RAWRNavX2 -.implements.-> org_lasarobotics_hardware_IMU
    RAWRQuestNav -.implements.-> IMU
    Pathfinder[Pathfinder]
    LocalADStarAK -.implements.-> Pathfinder
    SwerveDriveSubsystem -.implements.-> AutoCloseable
    TankDriveSubsystem -.implements.-> AutoCloseable
    Sendable[Sendable]
    RAWRSwerveModule -.implements.-> Sendable
    PearceContainer -.implements.-> IRobotContainer
    SwerveDriveSubsystem --> SwerveHardware
    SwerveDriveSubsystem --> AssumedPoseSubsystem
    TankDriveSubsystem --> TankHardware
    AssumedPoseSubsystem --> RAWRNavX2
    SmartSequentialCommand --> SmartSequentialCommand
    RobotSectorEvaluator --> SwerveDriveSubsystem
    SmartSequentialCommandSequencer --> SmartSequentialCommand
    SmartSequentialCommandSequencer --> SmartSequentialCommand
    Robot --> IRobotContainer
    style BuildConstants fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style DashboardVariable fill:#66bb6a,stroke:#333,stroke-width:2px,color:#fff
    style NamedAuto fill:#66bb6a,stroke:#333,stroke-width:2px,color:#fff
    style Robot fill:#66bb6a,stroke:#333,stroke-width:2px,color:#fff
    style DashboardAutoUpdater fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style CANDiagnostics fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style EasyBreakBeam fill:#ba68c8,stroke:#333,stroke-width:2px,color:#fff
    style EasyMotor fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style SwerveHardware fill:#ba68c8,stroke:#333,stroke-width:2px,color:#fff
    style SwerveHardwareParams fill:#ba68c8,stroke:#333,stroke-width:2px,color:#fff
    style SwerveModuleHardware fill:#ba68c8,stroke:#333,stroke-width:2px,color:#fff
    style TankHardware fill:#ba68c8,stroke:#333,stroke-width:2px,color:#fff
    style NamedAutoRegistry fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style RobotContainerRegistry fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style RobotExceptionHandler fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style RobotUtils fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style TeamUtils fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style DefaultContainer fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style RAWRNavX2 fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style RAWRQuestNav fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style IDiagnostic fill:#66bb6a,stroke:#333,stroke-width:2px,color:#fff
    style IMU fill:#66bb6a,stroke:#333,stroke-width:2px,color:#fff
    style IRobotContainer fill:#66bb6a,stroke:#333,stroke-width:2px,color:#fff
    style LocalADStarAK fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style DashboardSubsystem fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style SwerveDriveSubsystem fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style TankDriveSubsystem fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style SingleMotorSubsystem fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style AssumedPoseSubsystem fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style RAWRSwerveModule fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style CommonConstants fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style Main fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style HubStatus fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style RobotSector fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style SmartSequentialCommand fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style SmartSequentialCommandContainer fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style PearceConstants fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style PearceContainer fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style ProtoClimber fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style ProtoFeeder fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style ProtoIntake fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style ProtoShooter fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style DynamicPather fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style RobotSectorEvaluator fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style SmartSequentialCommandSequencer fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style Robot fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style BaseInstanceable_CANDiagnostics fill:#eeeeee,stroke:#999,color:#333,stroke-dasharray: 5 5
    style LoggableHardware fill:#eeeeee,stroke:#999,color:#333,stroke-dasharray: 5 5
    style AutoCloseable fill:#eeeeee,stroke:#999,color:#333,stroke-dasharray: 5 5
    style SubsystemBase fill:#eeeeee,stroke:#999,color:#333,stroke-dasharray: 5 5
    style SwerveModule fill:#eeeeee,stroke:#999,color:#333,stroke-dasharray: 5 5
    style LoggedRobot fill:#eeeeee,stroke:#999,color:#333,stroke-dasharray: 5 5
    style Thread_UncaughtExceptionHandler fill:#eeeeee,stroke:#999,color:#333,stroke-dasharray: 5 5
    style org_lasarobotics_hardware_IMU fill:#eeeeee,stroke:#999,color:#333,stroke-dasharray: 5 5
    style Pathfinder fill:#eeeeee,stroke:#999,color:#333,stroke-dasharray: 5 5
    style Sendable fill:#eeeeee,stroke:#999,color:#333,stroke-dasharray: 5 5
```
