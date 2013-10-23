/*
 * 18-649 Fall 2013
 * Group 22
 * David Chow davidcho
 * Brody Anderson bcanders
 * Yang Liu yangliu2
 * Jeffrey Lau jalau
 * CarPositionControl.java
 * @author: Brody Anderson (bcanders)
 */
package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.elevatormodules.AtFloorCanPayloadTranslator;
import simulator.elevatormodules.CarLevelPositionCanPayloadTranslator;
import simulator.framework.Controller;
import simulator.framework.Elevator;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.CarPositionIndicatorPayload;
import simulator.payloads.CarPositionIndicatorPayload.WriteableCarPositionIndicatorPayload;
import simulator.payloads.DrivePayload.WriteableDrivePayload;

public class CarPositionControl extends Controller {

	//enumerate states
    private enum State {
        POSITION,
    }
    
    private int currentFloor;
    private State state;
    private SimTime period;
    private int lastValidFloor;
    
    // Inputs:
    // mAtFloor
	ReadableCanMailbox[] networkAtFloor;
	private AtFloorCanPayloadTranslator[] mAtFloor;
    // mCarLevelPosition
	ReadableCanMailbox networkCarLevelPosition;
	private CarLevelPositionCanPayloadTranslator mCarLevelPosition;
    // mDesiredFloor
	ReadableCanMailbox networkDesiredFloor; 
	private DesiredFloorCanPayloadTranslator mDesiredFloor;
    // mDriveSpeed
	ReadableCanMailbox networkDriveSpeed;
    DriveSpeedCanPayloadTranslator mDriveSpeed;
    
    // Outputs:
    // mCarPositionIndicator
    private WriteableCanMailbox networkCarPositionIndicator;
    private CarPositionCanPayloadTranslator mCarPositionIndicator;
    // carPositionIndicator
    private WriteableCarPositionIndicatorPayload carPositionIndicator;
	
	public CarPositionControl(SimTime period, boolean verbose) {
		super("CarPositionControl", verbose);
		// Initialize currentFloor
		this.currentFloor = 1;
		// Initialize state
		this.state = State.POSITION;
		// Initialize period
		this.period = period;
		this.lastValidFloor = 1;
		
		// Initialize mAtFloor
		networkAtFloor = new CanMailbox.ReadableCanMailbox[Elevator.numFloors*2];
        mAtFloor = new AtFloorCanPayloadTranslator[Elevator.numFloors *2];

		for (int floor = 1; floor <= Elevator.numFloors; floor++) {
            for (Hallway hall : Hallway.replicationValues) {
                int index = ReplicationComputer.computeReplicationId(floor, hall);
                networkAtFloor[index] = CanMailbox.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID + 
                											ReplicationComputer.computeReplicationId(floor, hall));
                mAtFloor[index] = new AtFloorCanPayloadTranslator(networkAtFloor[index], floor, hall);
                canInterface.registerTimeTriggered(networkAtFloor[index]);
            }
        }
		
		// initialize mCarLevelPosition
		
	    // initialize mDesiredFloor
		
	    // initialize mDriveSpeed
		
		// Initialize carPositionIndicator (output)
		carPositionIndicator = CarPositionIndicatorPayload.getWriteablePayload();
		physicalInterface.sendTimeTriggered(carPositionIndicator, period);
		
		// Initialize mCarPositionIndicator (output)
		networkCarPositionIndicator = CanMailbox.getWriteableCanMailbox(
				MessageDictionary.CAR_POSITION_CAN_ID);
		mCarPositionIndicator = new CarPositionCanPayloadTranslator(networkCarPositionIndicator);
		canInterface.sendTimeTriggered(networkCarPositionIndicator,period);
		
        timer.start(period);		

	}
	
	/** 
	 * CurrentFloor is a simple helper function that returns an int
	 * representing the floor at which mAtFloor[f,d] is true
	 * @return - the floor at which mAtFloor[f,d] is true
	 */
	private int CurrentMAtFloor() {
		int f;
		int cFloor = -1;
		for (int floor = 1; floor <= Elevator.numFloors; floor++) {
            for (Hallway hall : Hallway.replicationValues) {
                f = ReplicationComputer.computeReplicationId(floor,hall);
                if (mAtFloor[f].getValue()){
                	if (cFloor != -1) { // This should never happen!
                		// Errormessage
                	}
                	else {
                    	cFloor = floor;
                	}
                }
            }
        }
		return cFloor;
	}

	@Override
	public void timerExpired(Object callbackData) {
		if (CurrentMAtFloor() != -1) {
    		this.lastValidFloor = CurrentMAtFloor();
    	}
		this.currentFloor = this.lastValidFloor;
    	this.mCarPositionIndicator.set(this.currentFloor);
    	this.carPositionIndicator.set(this.currentFloor);
        
    	State newState = State.POSITION;
		//update the state variable
		state = newState;
		
		//report the current state
		setState(STATE_KEY,newState.toString());
		
		//schedule the next iteration of the controller
		//you must do this at the end of the timer callback in order to restart
		//the timer
		timer.start(period);
	}

}
