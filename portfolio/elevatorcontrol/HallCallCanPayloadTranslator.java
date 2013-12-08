/*
 * 18-649 Fall 2013
 * Group 22
 * David Chow davidcho
 * Brody Anderson bcanders
 * Yang Liu yangliu2
 * Jeffrey Lau jalau
 * HallCallCanPayloadTranslator.java
 * @author: Jeff Lau (jalau)
 */

package simulator.elevatorcontrol;

import simulator.elevatorcontrol.MessageDictionary;
import simulator.framework.Hallway;
import simulator.framework.Direction;
import simulator.framework.ReplicationComputer;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;

public class HallCallCanPayloadTranslator extends BooleanCanTranslator {

    /**
     * CAN translator for messages from HallCalls
     * @param payload CAN payload object whose message is interpreted by this translator
     * @param floor replication index
     * @param hallway replication index
     * @param direction replication index
     */
    public HallCallCanPayloadTranslator(WriteableCanMailbox payload, int floor, Hallway hallway, Direction direction) {
        super(payload, MessageDictionary.HALL_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway, direction),
        		"HallCall" + ReplicationComputer.makeReplicationString(floor, hallway, direction));
    }

    /**
     * CAN translator for messages from HallCalls
     * @param payload CAN payload object whose message is interpreted by this translator
     * @param floor replication index
     * @param hallway replication index
     * @param direction replication index
     */
    public HallCallCanPayloadTranslator(ReadableCanMailbox payload, int floor, Hallway hallway, Direction direction) {
        super(payload, MessageDictionary.HALL_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway, direction),
        		"HallCall" + ReplicationComputer.makeReplicationString(floor, hallway, direction));
    }

}
