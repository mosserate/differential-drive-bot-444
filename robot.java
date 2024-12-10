Drive drive = new Drive(); 

private void configureBindings() {
	drive.setDefaultCommand(drive.drive(driver::getLeftY, driver::getRightY));

private final RelativeEncoder leftEncoder = LEFT_FRONT.getEncoder(); 
private final RelativeEncoder rightEncoder = RIGHT_FRONT.getEncoder(); 

Monologue.setupMonologue(this, "/Robot", false, true);
addPeriodic(Monologue::updateAll, kDefaultPeriod);
addPeriodic(FaultLogger::update, 1);

@Log.NT 
private final Field2d field2d = new Field2d();

field2d.setRobotPose(pose());
}