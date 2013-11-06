/*
 * 18-649 Fall 2013
 * Group 22
 * David Chow davidcho
 * Brody Anderson bcanders
 * Yang Liu yangliu2
 * Jeffrey Lau jalau
 * HallButtonControl.java
 * @author: Yang Liu yangliu2
 */

package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.elevatormodules.DoorClosedCanPayloadTranslator;
import simulator.elevatormodules.AtFloorCanPayloadTranslator;
import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.HallCallPayload;
import simulator.payloads.HallCallPayload.ReadableHallCallPayload;
import simulator.payloads.HallLightPayload;
import simulator.payloads.HallLightPayload.WriteableHallLightPayload;
import simulator.payloads.translators.BooleanCanPayloadTranslator;

public class HallButtonControl extends Controller {
	
    private ReadableHallCallPayload localHallCall;
    private WriteableHallLightPayload localHallLight;

    private WriteableCanMailbox networkHallCallOut;
    private HallCallCanPayloadTranslator mHallCall;

    private ReadableCanMailbox networkDoorClosedLeft;
    private DoorClosedCanPayloadTranslator mDoorClosedLeft;

    private ReadableCanMailbox networkDoorClosedRight;
    private DoorClosedCanPayloadTranslator mDoorClosedRight;

    private ReadableCanMailbox networkAtFloor;
    private AtFloorCanPayloadTranslator mAtFloor;

    private ReadableCanMailbox networkDesiredFloor;
    private DesiredFloorCanPayloadTranslator mDesiredFloor;


    //these variables keep track of which instance this is.
    private final Hallway hallway;
    private final Direction direction;
    private final int floor;
    private SimTime period;

    //enumerate states
    private enum State {
        STATE_LIGHT_OFF,
        STATE_LIGHT_ON,
    }
    private State state = State.STATE_LIGHT_OFF;


    public HallButtonControl(int floor, Hallway hallway, Direction direction, SimTime period, boolean verbose) {
    	super("HallButtonControl" + ReplicationComputer.makeReplicationString(floor, hallway, direction), verbose);

        this.period = period;
        this.hallway = hallway;
        this.direction = direction;
        this.floor = floor;

        log("Created HallButtonControl with period = ", period);

        localHallCall = HallCallPayload.getReadablePayload(floor, hallway, direction);
        physicalInterface.registerTimeTriggered(localHallCall);

        localHallLight = HallLightPayload.getWriteablePayload(floor, hallway, direction);
        physicalInterface.sendTimeTriggered(localHallLight, period);

        networkHallCallOut = CanMailbox.getWriteableCanMailbox(MessageDictionary.HALL_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway, direction));
        mHallCall = new HallCallCanPayloadTranslator(networkHallCallOut, floor, hallway, direction);
        canInterface.sendTimeTriggered(networkHallCallOut, period);

        networkDoorClosedLeft = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway, Side.LEFT));
        mDoorClosedLeft = new DoorClosedCanPayloadTranslator(networkDoorClosedLeft, hallway, Side.LEFT);
        canInterface.registerTimeTriggered(networkDoorClosedLeft);

        networkDoorClosedRight = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway, Side.RIGHT));
        mDoorClosedRight = new DoorClosedCanPayloadTranslator(networkDoorClosedRight, hallway, Side.RIGHT);
        canInterface.registerTimeTriggered(networkDoorClosedRight);

        networkAtFloor = CanMailbox.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway));
        mAtFloor = new AtFloorCanPayloadTranslator(networkAtFloor, floor, hallway);
        canInterface.registerTimeTriggered(networkAtFloor);

        networkDesiredFloor = CanMailbox.getReadableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
        mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloor);
        canInterface.registerTimeTriggered(networkDesiredFloor);


        timer.start(period);
    }

    public void timerExpired(Object callbackData) {
        State newState = state;
        switch (state) {
            case STATE_LIGHT_OFF:
                localHallLight.set(false);
                mHallCall.set(false);

                //#transition '8.T.1'
                if (localHallCall.pressed() == true) {
                    newState = State.STATE_LIGHT_ON;
                } else {
                    newState = state;
                }
                break;
            case STATE_LIGHT_ON:
                localHallLight.set(true);
                mHallCall.set(true);

                //#transition '8.T.2'
                if (mDoorClosedLeft.getValue() == false && 
                	mDoorClosedRight.getValue() == false && 
                	mAtFloor.getValue() == true  
                	/*((mDesiredFloor.getFloor() > floor && 
                			direction == Direction.UP) || 
                			(mDesiredFloor.getFloor() < floor && 
                					direction == Direction.DOWN))*/)  {
                    newState = State.STATE_LIGHT_OFF;
                } else {
                    newState = state;
                }
                break;
            default:
                throw new RuntimeException("State " + state + " was not recognized.");
        }


        //update the state variable
        state = newState;

        //report the current state
        setState(STATE_KEY,newState.toString());

        timer.start(period);
    }    
}