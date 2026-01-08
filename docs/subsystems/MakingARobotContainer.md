# Making A Robot Container

RobotContainer's control the functionaity of a robot per team.  

1. Implement IRobotContainer and set up the required methods (createContainer, simulationPeriodic, and getAutonomousCommand).
2. Annotate with @NoArgsConstructor(access = AccessLevel.PRIVATE), as Robot Containers should not have a public constructor. (createContainer() will be used to create a new container).
3. Annotate the class with @Robot; this will automatically set the robot container to be used based on the team number. Example: (@Robot(team = 1745) for team 1745, etc).
