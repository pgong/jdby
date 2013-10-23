/*
 * 18-649 Fall 2013
 * Group 22
 * David Chow davidcho
 * Brody Anderson bcanders
 * Yang Liu yangliu2
 * Jeffrey Lau jalau
 * Dispatcher.java
 * @author: Jeff Lau (jalau)
 */
package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.elevatormodules.AtFloorCanPayloadTranslator;
import simulator.elevatormodules.CarWeightCanPayloadTranslator;
import simulator.elevatormodules.DoorClosedCanPayloadTranslator;
import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.Elevator;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.BooleanCanPayloadTranslator;

/**
 * 
 * @author Jeff Lau
 *
 */
public class Dispatcher extends Controller{

	/************************
	 * Declarations
	 ***********************/
	//Network interface
	//Outputs: mDesiredFloor and mDesiredDwell
	private WriteableCanMailbox networkDesiredFloor;
	private WriteableCanMailbox networkDesiredDwellF;
	private WriteableCanMailbox networkDesiredDwellB;

	//Respective translators for output messages
	private DesiredFloorCanPayloadTranslator mDesiredFloor;
	private DesiredDwellCanPayloadTranslator mDesiredDwellF;
	private DesiredDwellCanPayloadTranslator mDesiredDwellB;

	/*Inputs:mAtFloor, mDoorClosed, mHallCall,
	 * mCarCall, mCarWeight
	 */
	private ReadableCanMailbox[] networkAtFloor;
	private ReadableCanMailbox[] networkDoorClosed;
	private ReadableCanMailbox[] networkHallCall;
	private ReadableCanMailbox[] networkCarCall;
	private ReadableCanMailbox networkCarWeight;

	//Respective translators for input messages
	private AtFloorCanPayloadTranslator[] mAtFloor;
	private DoorClosedCanPayloadTranslator[] mDoorClosed;
	private BooleanCanPayloadTranslator[] mHallCall;
	private BooleanCanPayloadTranslator[] mCarCall;
	private CarWeightCanPayloadTranslator mCarWeight;

	private int numFloors;
	private int floor;
	private int curr_f;
	private Hallway curr_h;
	private Hallway hallway;
	private Direction direction;
    private Direction curr_d;
	private SimTime dwellTime;
    

	//Store he period for the controller
	private SimTime period;

	//Enumerate states
	private enum State{
		STATE_ATFLOOR,
		STATE_EMERGENCY,
		STATE_BETWEEN,
	}

	//State variable initialized at ATFLOOR
	private State state = State.STATE_BETWEEN;

	public Dispatcher(int numFloors, SimTime period, boolean verbose){
		super("Dispatcher", verbose);

		this.period = period;
		this.numFloors = numFloors;
		curr_f = 1;
		curr_h = Hallway.BOTH;
        curr_d = Direction.UP;
		floor = 1;
		hallway = Hallway.BOTH;
		direction = Direction.STOP;
		dwellTime = new SimTime(2, SimTime.SimTimeUnit.SECOND);
		
		log("Created Dispatcher with period = ", period);

		//Initialize network interface
		//Outputs
		networkDesiredFloor = CanMailbox.getWriteableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
		mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloor);

		networkDesiredDwellF = CanMailbox.getWriteableCanMailbox(
						MessageDictionary.DESIRED_DWELL_BASE_CAN_ID +
						ReplicationComputer.computeReplicationId(Hallway.FRONT));
		mDesiredDwellF = new DesiredDwellCanPayloadTranslator(networkDesiredDwellF,Hallway.FRONT);
		
		networkDesiredDwellB = CanMailbox.getWriteableCanMailbox(
				MessageDictionary.DESIRED_DWELL_BASE_CAN_ID +
				ReplicationComputer.computeReplicationId(Hallway.BACK));
		mDesiredDwellB = new DesiredDwellCanPayloadTranslator(networkDesiredDwellB,Hallway.BACK);


		canInterface.sendTimeTriggered(networkDesiredFloor,period);
		canInterface.sendTimeTriggered(networkDesiredDwellF,period);
		canInterface.sendTimeTriggered(networkDesiredDwellB,period);



		//Inputs
		//CarWeight
		networkCarWeight = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_WEIGHT_CAN_ID);
		mCarWeight = new CarWeightCanPayloadTranslator(networkCarWeight);
		canInterface.registerTimeTriggered(networkCarWeight);

		//AtFloor
		networkAtFloor = new CanMailbox.ReadableCanMailbox[Elevator.numFloors*2];
		mAtFloor = new AtFloorCanPayloadTranslator[Elevator.numFloors *2];
		for (int floors = 1; floors <= Elevator.numFloors; floors++) {
			for (Hallway h : Hallway.replicationValues) {
				int index = ReplicationComputer.computeReplicationId(floors, h);
				networkAtFloor[index] = CanMailbox.getReadableCanMailbox(
						MessageDictionary.AT_FLOOR_BASE_CAN_ID + 
						ReplicationComputer.computeReplicationId(floors, h));
				mAtFloor[index] = new AtFloorCanPayloadTranslator(networkAtFloor[index], floors, h);
				canInterface.registerTimeTriggered(networkAtFloor[index]);
			}
		}

		//DoorClosed
		networkDoorClosed = new CanMailbox.ReadableCanMailbox[4];
		mDoorClosed = new DoorClosedCanPayloadTranslator[4];
		for (Hallway h : Hallway.replicationValues) {
			for (Side s : Side.values()) {
				int index = ReplicationComputer.computeReplicationId(h, s);
				networkDoorClosed[index] = CanMailbox.getReadableCanMailbox(
						MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + 
						ReplicationComputer.computeReplicationId(h, s));
				mDoorClosed[index] = new DoorClosedCanPayloadTranslator(networkDoorClosed[index],h,s);
				canInterface.registerTimeTriggered(networkDoorClosed[index]);
			}
		}

		//HallCall
		networkHallCall = new CanMailbox.ReadableCanMailbox[Elevator.numFloors*4];
		mHallCall = new BooleanCanPayloadTranslator[Elevator.numFloors*4];
		for(int floors = 1; floors <= Elevator.numFloors; floors++) {
			for (Hallway h : Hallway.replicationValues) {
				for (Direction d : Direction.replicationValues) {
					int index = ReplicationComputer.computeReplicationId(floors,h,d);
					networkHallCall[index] = CanMailbox.getReadableCanMailbox(
							MessageDictionary.HALL_CALL_BASE_CAN_ID + 
							ReplicationComputer.computeReplicationId(floors,h,d));
					mHallCall[index] = new BooleanCanPayloadTranslator(networkHallCall[index]);
					canInterface.registerTimeTriggered(networkHallCall[index]);
				}
			}
		}

		//CarCall
		networkCarCall = new CanMailbox.ReadableCanMailbox[Elevator.numFloors*2];
		mCarCall = new BooleanCanPayloadTranslator[Elevator.numFloors*2];
		for(int floors = 1; floors <= Elevator.numFloors; floors++) {
			for (Hallway h : Hallway.replicationValues) {
				int index = ReplicationComputer.computeReplicationId(floors,h);
				networkCarCall[index] = CanMailbox.getReadableCanMailbox(
						MessageDictionary.CAR_CALL_BASE_CAN_ID + 
						ReplicationComputer.computeReplicationId(floors,h));
				mCarCall[index] = new BooleanCanPayloadTranslator(networkCarCall[index]);
				canInterface.registerTimeTriggered(networkCarCall[index]);
			}
		}
		//initially set target to lobby.
		mDesiredDwellF.set(dwellTime);
		mDesiredDwellB.set(dwellTime);
		mDesiredFloor.set(floor,hallway,direction);
		
		timer.start(period);	    
	}

	@Override
	public void timerExpired(Object callbackData) {
		State newState = state;

		//State Machine
		switch(state){
		//#state State 1: At Floor
		case STATE_ATFLOOR:
			//State actions for 'ATFLOOR'
			//Check both sides for closed doors if that is the case.
			curr_f = floor;
			if(hallway == Hallway.BOTH)
				curr_h = Hallway.FRONT;
			else
				curr_h = hallway;
            // Set Direction
            if (floor == Elevator.numFloors) {
                curr_d = Direction.DOWN;
            }
            else if (floor == 1) {
                curr_d = Direction.UP;
            }
            // Set next floor
            if (curr_d == Direction.UP) {
       			floor = (floor % Elevator.numFloors) + 1;
            }
            else if (curr_d == Direction.DOWN) {
                floor = (floor - 1) % Elevator.numFloors;
			}
			//Decide which hallway is valid. If front and back, then both, else either front or back.
			if (Elevator.hasLanding(floor, Hallway.FRONT))	{
				if (Elevator.hasLanding(floor, Hallway.BACK))
					hallway = Hallway.BOTH;
				else
					hallway = Hallway.FRONT;
			}
			else 
				hallway = Hallway.BACK;

			//Now set the next target.
			mDesiredFloor.set(floor, direction, hallway);
			if(curr_h == Hallway.BOTH) {
				//If either side of doors open and we're not at the floor, emergency!
				//#transition 11.T.2
				if ((!mDoorClosed[ReplicationComputer.computeReplicationId(Hallway.FRONT, Side.LEFT)].getValue() ||
						!mDoorClosed[ReplicationComputer.computeReplicationId(Hallway.BACK, Side.LEFT)].getValue()) && 
						!mAtFloor[ReplicationComputer.computeReplicationId(curr_f,Hallway.FRONT)].getValue()) {
					newState = State.STATE_EMERGENCY;
				}
				else
					newState = State.STATE_BETWEEN;
			}
			//if it is not at ANY floor and ANY doors are open, jump to emergency state
			//#transition 11.T.2
			else if ((!mDoorClosed[ReplicationComputer.computeReplicationId(Hallway.FRONT, Side.LEFT)].getValue() ||
					!mDoorClosed[ReplicationComputer.computeReplicationId(Hallway.BACK, Side.LEFT)].getValue()) && 
					!mAtFloor[ReplicationComputer.computeReplicationId(curr_f,curr_h)].getValue()) {
				newState = State.STATE_EMERGENCY;
			}
			//if no issues, move to the next state
			//#transition 11.T.1
			else
				newState = State.STATE_BETWEEN;
			break;

			//State actions for 'BETWEEN'
			//#state State 2: Between
			case STATE_BETWEEN:
				
				//Check both sides for closed doors if that is the case.
				if(hallway == Hallway.BOTH) {
					//If we're at the next floor AND doors are opening, jump to ATFLOOR
					//#transition 11.T.3
					if(mAtFloor[ReplicationComputer.computeReplicationId(floor,Hallway.FRONT)].getValue() &&
					   !mDoorClosed[ReplicationComputer.computeReplicationId(Hallway.FRONT, Side.LEFT)].getValue())
						newState = State.STATE_ATFLOOR;
					//If either side of doors open and we're not at any floor, emergency!
					//#transition 11.T.4
					else if ((!mDoorClosed[ReplicationComputer.computeReplicationId(Hallway.FRONT, Side.LEFT)].getValue() ||
						!mDoorClosed[ReplicationComputer.computeReplicationId(Hallway.BACK, Side.LEFT)].getValue()) && 
						!mAtFloor[ReplicationComputer.computeReplicationId(curr_f,curr_h)].getValue() &&
						!mAtFloor[ReplicationComputer.computeReplicationId(floor,Hallway.FRONT)].getValue()) {
						newState = State.STATE_EMERGENCY;
					}
					else
						newState = state;
				}
				//If we're at the next floor AND doors are opening, jump to ATFLOOR
				//#transition 11.T.3
				else if(mAtFloor[ReplicationComputer.computeReplicationId(floor,hallway)].getValue() &&
						!mDoorClosed[ReplicationComputer.computeReplicationId(hallway, Side.LEFT)].getValue())
					newState = State.STATE_ATFLOOR;

				//If either side of doors open and we're not at any floor, emergency!
				//#transition 11.T.4
				else if ((!mDoorClosed[ReplicationComputer.computeReplicationId(Hallway.FRONT, Side.LEFT)].getValue() ||
						!mDoorClosed[ReplicationComputer.computeReplicationId(Hallway.BACK, Side.LEFT)].getValue()) && 
						!mAtFloor[ReplicationComputer.computeReplicationId(curr_f,curr_h)].getValue() &&
						!mAtFloor[ReplicationComputer.computeReplicationId(floor,hallway)].getValue()) {
					newState = State.STATE_EMERGENCY;
				}
				else
					newState = state;
				break;

				//State actions for 'EMERGENCY'
				//#state State 3: Emergency
			case STATE_EMERGENCY:
				floor = 1;
				hallway = Hallway.BOTH;
				//Set the desired floor back down to the lobby!
				mDesiredFloor.set(floor, hallway, direction);
				
				//Once the doors have opened at the lobby, we can return to normal operation.
				//#transition 11.T.5
				if(!mDoorClosed[ReplicationComputer.computeReplicationId(Hallway.FRONT,Side.LEFT)].getValue() &&
					mAtFloor[ReplicationComputer.computeReplicationId(1,Hallway.FRONT)].getValue())
					newState = State.STATE_ATFLOOR;
				else
					newState = state;
				break;
				
			default:
				throw new RuntimeException("State " + state + " was not recognized.");
		}

		/*log the results of this iteration
		if (state == newState) {
			log("remains in state: ",state);
		} else {
			log("Transition:",state,"->",newState);
		}
		*/
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


