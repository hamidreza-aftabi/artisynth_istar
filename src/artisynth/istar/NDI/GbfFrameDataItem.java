package artisynth.istar.NDI;

/**
 * This class encapsulates frame data for a specific tool type as it is read from BX2
 */
public class GbfFrameDataItem
{
    //! Indicates what type of frame gathered the data. See the related enum to interpret this value
   byte frameType;

   //! Each frame is given an sequence number, which clients can usually ignore
   byte frameSequenceIndex;

   //! Same as TransformStatus, but only codes that apply to the frame as a whole
   short frameStatus;

   //! The frame number that identifies when the data was collected
   int frameNumber;

   //! The "seconds" part of the timestamp
   int timespec_s;

   //! The "nanoseconds" part of the timestamp
   int timespec_ns;
   
   //! This container holds other GbfComponents containing the data
   GbfContainer frameData;

   public GbfFrameDataItem (BufferedReader reader) {
      frameType = reader.get_byte();
      frameSequenceIndex = reader.get_byte();
      frameStatus = reader.get_uint16();
      frameNumber = reader.get_uint32();
      timespec_s = reader.get_uint32();
      timespec_ns = reader.get_uint32();
      frameData = new GbfContainer(reader);
   }

   /**
    * Returns a string representation of the data for debugging purposes.
    */
   public String toString() {
      StringBuilder sb = new StringBuilder();
      sb.append ("-----GbfFrameDataItem \n");
      sb.append ("frameType=" + Utils.byteToHexString(frameType));
      sb.append ("("+ToolData.FrameType.findValue(frameType)+")\n");
      sb.append ("frameSequenceIndex=" + Utils.byteToHexString(frameSequenceIndex)+"\n");
      sb.append ("frameStatus=" + Utils.shortToHexString(frameStatus) + "\n");
      sb.append ("frameNumber=" + Utils.intToHexString(frameNumber) + "\n");
      sb.append ("timestamp=" + Utils.intToHexString(timespec_s));
      sb.append ("," + Utils.intToHexString(timespec_ns) + "\n");
      sb.append (frameData.toString() + "\n");
      return sb.toString();
   }

};
