/*
 * 18-649 Fall 2013
 * Group 22
 * David Chow davidcho
 * Brody Anderson bcanders
 * Yang Liu yangliu2
 * Jeffrey Lau jalau
 * CarPositionCanPayloadTranslator.java
 * @author: Brody Anderson (bcanders)
 */
package simulator.elevatorcontrol;

import java.util.BitSet;

import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.CanPayloadTranslator;

public class CarPositionCanPayloadTranslator extends CanPayloadTranslator {

	
	public CarPositionCanPayloadTranslator(WriteableCanMailbox p) {
        super(p, 8, MessageDictionary.CAR_POSITION_CAN_ID);
    }
    
    public CarPositionCanPayloadTranslator(ReadableCanMailbox p) {
        super(p, 8, MessageDictionary.CAR_POSITION_CAN_ID);
    }
	
	@Override
	public String payloadToString() {
		// TODO Auto-generated method stub
		return null;
	}
	
	/**
	 * Takes an int representing the floor that the car is on and
	 * sets the carPosition payload to it
	 * @param position
	 */
	public void set(int position) {
		setPosition(position);
	}
	
	public int getValue() {
		return getPosition();
	}
	
	/**
     * Set the position in bits 0-16 of the can payload
     * @param position
     */
    public void setPosition(int p) {
        BitSet b = getMessagePayload();
        addIntToBitset(b, p, 0, 32);
        setMessagePayload(b, getByteSize());
    }

    /**
     * 
     * @return the position value from the CAN payload.
     */
    public int getPosition() {
        int p = getIntFromBitset(getMessagePayload(), 0, 16);
        if (p > 0) {
        	return p;
        }
        else {
        	throw new RuntimeException("Unrecognized Hallway Value " + p);
        }
    }


}
