package edu.wpi.first.wpilibj.command;

public abstract class BlockingCommand extends Command {

	private Thread blockingCommandThread;
	private Command curCommand;
	
	public abstract void runBlockingCommand();
	
	@Override
	protected void initialize() {
		blockingCommandThread = new Thread(new Runnable() {
			@Override
			public void run() {
				runBlockingCommand();
			}
		});
		blockingCommandThread.start();

	}

	@Override
	protected void execute() { }

	@Override
	protected boolean isFinished() {
		return !blockingCommandThread.isAlive();
	}

	@SuppressWarnings("deprecation")
	@Override
	protected void end() {
		blockingCommandThread.stop();
		if (curCommand != null && !curCommand.isCanceled()) {
			curCommand.cancel();
		}
	}

	@Override
	protected void interrupted() {
		end();
	}
	
	public void addBlockingCommand(Command... commands) {
		
		CommandGroup commandGroup = new CommandGroup();
		for (Command c : commands) {
			commandGroup.addParallel(c);
		}
		
		curCommand = commandGroup;
		curCommand.startRunning(); // sets running to true
		while (curCommand != null) {
			if (!curCommand.run()) { // runs init if not initialized and execute regardless
				curCommand.removed(); // runs end if finished and interrupted if cancelled
				curCommand = null;
			}
			try {
				Thread.sleep(20);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}
	
}
