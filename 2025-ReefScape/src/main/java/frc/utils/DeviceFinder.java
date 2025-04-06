package frc.utils;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;

import edu.wpi.first.hal.can.CANJNI;

public class DeviceFinder {

	/** helper routine to get last received message for a given ID */
	private long checkMessage(int fullId, int mask, int deviceID) {
		try {
			ByteBuffer targetID = ByteBuffer.allocateDirect(4);
			ByteBuffer timeStamp = ByteBuffer.allocateDirect(4);
			targetID.clear();
			targetID.order(ByteOrder.LITTLE_ENDIAN);
			targetID.asIntBuffer().put(0,fullId|deviceID);

			timeStamp.clear();
			timeStamp.order(ByteOrder.LITTLE_ENDIAN);
			timeStamp.asIntBuffer().put(0,0x00000000);
			
			CANJNI.FRCNetCommCANSessionMuxReceiveMessage(targetID.asIntBuffer(), 0b00011111111111111111100000011111, timeStamp);

			long retval = timeStamp.getInt();
			retval &= 0xFFFFFFFF; /* undo sign-extension */ 
			return retval;
		} catch (Exception e) {
			return -1;
		}
	}
	public long getManufacturer(int deviceID){
		ByteBuffer targetID = ByteBuffer.allocateDirect(4);
		ByteBuffer timeStamp = ByteBuffer.allocateDirect(4);
		targetID.clear();
		targetID.order(ByteOrder.LITTLE_ENDIAN);
		targetID.asIntBuffer().put(0, deviceID);

		timeStamp.clear();
		timeStamp.order(ByteOrder.LITTLE_ENDIAN);
		timeStamp.asIntBuffer().put(0,0x00000000);
		
		try {
			CANJNI.FRCNetCommCANSessionMuxReceiveMessage(targetID.asIntBuffer(), 0b00000000000000000000000000011111, timeStamp);
		} catch (Exception e) {
			return 0;
		}
		return (targetID.getInt() & 0b00000111111110000000000000000) >> 16;
	}
	/**
	 *   polls to determine if attached device is manufactured by Cross The Road (CTR).
	 *   This is meant to be used once initially (and not periodically) since 
	 *   this steals cached messages from the robot API.
	 *   Note: This only checks the manufacturer of the given device ID which is only
	 * 		 part of the arbitrated ID. Muliple devices with the same "CAN ID" (device ID)
	 *       can cause issues here
	 * @param CANID
	 * @return
	 */
	public boolean isCTR(int CANID){
		return getManufacturer(CANID) == 4;
	}
	/**
	 *   polls to determine if attached device is manufactured by REVRobotics
	 *   This is meant to be used once initially (and not periodically) since 
	 *   this steals cached messages from the robot API.
	 * 
	 * 	 Note: This only checks the manufacturer of the given device ID which is only
	 * 		 part of the arbitrated ID. Muliple devices with the same "CAN ID" (device ID)
	 *       can cause issues here
	 * @param CANID
	 * @return
	 */
	public boolean isREV(int CANID){
		return getManufacturer(CANID) == 5;
	}

	/** polls for received framing to determine if a device is present.
	 *   This is meant to be used once initially (and not periodically) since 
	 *   this steals cached messages from the robot API.
	 * @return ArrayList of strings holding the names of devices we've found.
	 */
	public ArrayList<String> Find() {
		ArrayList<String> retval = new ArrayList<String>();

		/* get timestamp0 for each device */
		long pdp0_timeStamp0; // only look for PDP at '0'
		long [] pcm_timeStamp0 = new long[63];
		long [] srx_timeStamp0 = new long[63];
		long [] sparkmax_timeStamp0 = new long[63];
		long [] pidgeon1_timeStamp0 = new long[63];
		
		pdp0_timeStamp0 = checkMessage(0x08041400,0b00011111111111111111111111111111,0);
		for(int i=0;i<63;++i) {
			pcm_timeStamp0[i] = checkMessage(0x09041400, 0b00011111111111111111111111111111, i);
			srx_timeStamp0[i] = checkMessage(0x02041400, 0b00011111111111111111111111111111, i);
			pidgeon1_timeStamp0[i] = checkMessage(21<<24 | 4<<16 | 3<<10 | 3<<6,0b00011111111111111111100000011111,i);
		}

		/* wait 200ms */
		try {
			Thread.sleep(200);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

		/* get timestamp1 for each device */
		long pdp0_timeStamp1 = -1; // only look for PDP at '0'
		long [] pcm_timeStamp1 = new long[63];
		long [] srx_timeStamp1 = new long[63];
		long [] sparkmax_timeStamp1 = new long[63];
		long [] pidgeon1_timeStamp1 = new long[63];
		
		pdp0_timeStamp0 = checkMessage(0x08041400,0b00011111111111111111111111111111,0);
		for(int i=0;i<63;++i) {
			pcm_timeStamp1[i] = checkMessage(0x09041400, 0b00011111111111111111111111111111, i);
			srx_timeStamp1[i] = checkMessage(0x02041400, 0b00011111111111111111111111111111, i);
			pidgeon1_timeStamp1[i] = checkMessage(21<<24 | 4<<16 | 3<<10 | 3<<6,0b00011111111111111111100000011111,i);
		}

		/* compare, if timestamp0 is good and timestamp1 is good, and they are different, device is healthy */
		if( pdp0_timeStamp0>=0 && pdp0_timeStamp1>=0 && pdp0_timeStamp0!=pdp0_timeStamp1)
			retval.add("PDP 0");

		for(int i=0;i<63;++i) {
			if( pcm_timeStamp0[i]>=0 && pcm_timeStamp1[i]>=0 && pcm_timeStamp0[i]!=pcm_timeStamp1[i])
				retval.add("PCM " + i);
				if( srx_timeStamp0[i]>=0 && srx_timeStamp1[i]>=0 && srx_timeStamp0[i]!=srx_timeStamp1[i])
				retval.add("SRX " + i);
				if( sparkmax_timeStamp0[i]>=0 && sparkmax_timeStamp1[i]>=0 && sparkmax_timeStamp0[i]<sparkmax_timeStamp1[i])
				retval.add("SPARKMAX " + i);
				if( pidgeon1_timeStamp0[i]>=0 && pidgeon1_timeStamp1[i]>=0 && pidgeon1_timeStamp0[i]<pidgeon1_timeStamp1[i])
				retval.add("PIDGEON " + i);
		}
		return retval;
	}
}
