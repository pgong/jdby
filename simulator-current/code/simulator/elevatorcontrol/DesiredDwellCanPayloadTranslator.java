/*
 * 18-649 Fall 2013
 * Group 22
 * David Chow davidcho
 * Brody Anderson bcanders
 * Yang Liu yangliu2
 * Jeffrey Lau jalau
 * DesiredDwellCanPayloadTranslator.java
 * @author: Jeff Lau (jalau) and Brody Anderson (bcanders)
 */


package simulator.elevatorcontrol;

import java.util.BitSet;

import jSimPack.SimTime;
import jSimPack.SimTime.SimTimeUnit;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.CanPayloadTranslator;

public class DesiredDwellCanPayloadTranslator extends CanPayloadTranslator {

	private SimTime Dwell;
	
	/**
     * Constructor for WriteableCanMailbox.  You should always implement both a 
     * Writeable and Readable constructor so the same translator can be used for
     * both objects
     * @param payload
     */
    public DesiredDwellCanPayloadTranslator(WriteableCanMailbox payload, Hallway hallway) {
        super(payload, 8, MessageDictionary.DESIRED_DWELL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway));
    }

    /**
     * Constructor for ReadableCanMailbox.  You should always implement both a 
     * Writeable and Readable constructor so the same translator can be used for
     * both objects
     * @param payload
     */
    public DesiredDwellCanPayloadTranslator(ReadableCanMailbox payload, Hallway hallway) {
        super(payload, 8, MessageDictionary.DESIRED_DWELL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway));
    }
    
    /**
     * Utility method to add an double value to a bit set. This method
     * modifies the bits from <code>startLocation</code> to <code>startLocation
     * + bitSize</code> by setting them according to the given double value.
     * By calling this method several times with different startLocations,
     * multiple values can be stored in a single bitset.
     * 
     * We truncate the most significant 4 bytes of the double-converted-to-long-bits
     * to keep the message small and efficient.
     * 
     * From 0 to 20, doubleToLongBit ranges from:
     * 0x0f0000000000000000 to 0x0f4034000000000000
     * 
     * Thus, we can truncate to 0x0f000000 and 0x0f403400
     * 
     * @param b
     *        BitSet to modify.
     * @param value
     *        double value to set. Negative values will not be preserved
     * @param startLocation
     *        the index in the bit set that corresponds to the least significant bit of the value.
     *        This value is zero-indexed.
     * @param bitSize
     *        the number of bits used to represent the integer. Values larger
     *        than 64 will generate an error.
     */
    public static void addDoubleToBitset(BitSet b, double value, int startLocation,
        int bitSize)
    {
        if (bitSize > 32)
        {
            throw new IllegalArgumentException("bitSize too large");
        }
        if (bitSize <= 0)
        {
            throw new IllegalArgumentException("bitSize must be positive");
        }
        if (bitSize < 32)
        {
            // check min/max according to elevator speeds
        	// we set max to some arbitrary large number but still within
        	//"reason" of a moving elevator.
            double max = 20.0;
            double min = 0.0;
            if (value > max)
            {
                throw new IllegalArgumentException("Value " + value
                        + " is too large");
            }
            if (value < min)
            {
                throw new IllegalArgumentException("Value " + value
                        + " is negative!");
            }
        }
        int mask = 0x1;
        long value_to_store = Double.doubleToLongBits(value);
        value_to_store = value_to_store >> 32;
        int bitOffset = startLocation;
        for (int i = 0; i < bitSize; i++)
        {
            b.set(bitOffset, ((int)value_to_store & mask) == mask);
            mask = mask << 1;
            bitOffset++;
        }
    }

    /**
     * Recovers a double value from the specified bit range. This method is
     * designed to be used in conjunction with addDoubleToBitset. With the
     * same concepts of truncated long bit values.
     * 
     * @param b
     *        The BitSet to read
     * @param startLocation
     *        The location of the lsb of the value. This value is zero-indexed.
     * @param bitSize
     *        The number of bits to read.
     * @return The recovered (positive) double value.
     */
    public static double getDoubleFromBitset(BitSet b, int startLocation, int bitSize)
    {
        if (bitSize > 32)
        {
            throw new RuntimeException("bitSize too large");
        }
        if (bitSize <= 0)
        {
            throw new RuntimeException("bitSize must be positive");
        }
        long value = 0;
        int mask = 0x1;
        int bitOffset = startLocation;
        for (int i = 0; i < bitSize; i++)
        {
            if (b.get(bitOffset))
            {
                value = value | mask;
            }
            mask = mask << 1;
            bitOffset++;
        }
        if (bitSize < 32 && b.get(bitOffset - 1))
        {
            // sign extend the result is the top bit was set
            for (int i = bitSize; i < 32; i++)
            {
                value = value | mask;
                mask = mask << 1;
            }
        }
        value = value << 32;
        return Double.longBitsToDouble(value);
    }
    /**
     * This method is required for setting values by reflection in the
     * MessageInjector.  The order of parameters in .mf files should match the
     * signature of this method.
     * All translators must have a set() method with the signature that contains
     * all the parameter values.
     *
     * @param dwell
     */
    public void set(SimTime dwell) {
    	this.Dwell = dwell;
    	double temp_dwell = Dwell.getFrac(SimTimeUnit.SECOND);
    	BitSet b = getMessagePayload();
        addDoubleToBitset(b, temp_dwell, 0, 32);
        setMessagePayload(b, getByteSize());
    }
    

    /**
     * Returns an SimTime value of dwell.
     * @return
     */
    public SimTime getValue() {
    	double temp_dwell = getDoubleFromBitset(getMessagePayload(), 0, 32);
    	return new SimTime(temp_dwell, SimTimeUnit.SECOND);
    }
	
	@Override
	public String payloadToString() {
		return "Dwell = " + Dwell.toString();
	}

}
