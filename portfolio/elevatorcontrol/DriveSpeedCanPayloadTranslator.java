/*
 * 18-649 Fall 2013
 * Group 22
 * David Chow davidcho
 * Brody Anderson bcanders
 * Yang Liu yangliu2
 * Jeffrey Lau jalau
 * DriveSpeedCanPayloadTranslator.java
 * @author: Jeff Lau (jalau)
 */
package simulator.elevatorcontrol;

import java.util.BitSet;

import simulator.framework.Direction;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.CanPayloadTranslator;


/**
 * 
 * @author Jeff Lau
 *
 */
public class DriveSpeedCanPayloadTranslator extends CanPayloadTranslator{

    public DriveSpeedCanPayloadTranslator(WriteableCanMailbox p) {
        super(p, 6, MessageDictionary.DRIVE_SPEED_CAN_ID);
    }
    
    public DriveSpeedCanPayloadTranslator(ReadableCanMailbox p) {
        super(p, 6, MessageDictionary.DRIVE_SPEED_CAN_ID);
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
     * @param speed
     * @param dir
     */
    public void set(double speed, Direction dir) {
        setSpeed(speed);
        setDirection(dir);
    }
    
    public void setSpeed(double speed) {
        BitSet b = getMessagePayload();
        addDoubleToBitset(b, speed*1000, 0, 32);
        setMessagePayload(b, getByteSize());
    }

    public double getSpeed() {
        return getDoubleFromBitset(getMessagePayload(), 0, 32)/1000;
    }

    public void setDirection(Direction dir) {
        BitSet b = getMessagePayload();
        addIntToBitset(b, dir.ordinal(), 32, 16);
        setMessagePayload(b, getByteSize());
    }

    public Direction getDirection() {
        int val = getIntFromBitset(getMessagePayload(), 32, 16);
        for (Direction d : Direction.values()) {
            if (d.ordinal() == val) {
                return d;
            }
        }
        throw new RuntimeException("Unrecognized Direction Value " + val);
    }

    @Override
    public String payloadToString() {
        return "DriveSpeed:  speed=" + getSpeed() + " direction=" + getDirection();
    }
}
