package artisynth.istar.NDI;

/**
 * This class stores the 6D quaternion transformation indicating the device's
 * orientation.  Be sure to check the transform status. When missing, the
 * transform parameters are invalid and should be set to BAD_FLOAT.
 */
public class Transform {

   public static final double BAD_FLOAT = -3.697314E28;

   static final double MAX_NEGATIVE = -3.0E28;

   //! The handle that uniquely identifies the tool

   short toolHandle;

   //! The TransformStatus as a two byte integer. See the related enum for its interpretation.
   short status;

   //! The quaternion parameters in camera coordinates [mm]
   double q0, qx, qy, qz;

   //! The transformation parameters in camera coordinates [mm]
   double tx,ty,tz;

   //! The RMS error in the measurement [mm]
   double error;

   public static enum Status {
      Enabled, // 0x00
      Reserved01, // 0x01
      Reserved02, // 0x02
      PartiallyOutOfVolume, // 0x03
      Reserved04, // 0x04
      Reserved05, // 0x05
      Reserved06, // 0x06
      Reserved07, // 0x07
      Reserved08, // 0x08
      OutOfVolume, // 0x09
      Reserved0A, // 0x0A
      Reserved0B, // 0x0B
      Reserved0C, // 0x0C
      TooFewMarkers, // 0x0D
      Inteference, // 0x0E
      Reserved0F, // 0x0F
      Reserved10, // 0x10
      BadTransformFit, // 0x11
      DataBufferLimit, // 0x12
      AlgorithmLimit, // 0x13
      FellBehind, // 0x14
      OutOfSynch, // 0x15
      ProcessingError, // 0x16
      Reserved17, // 0x17
      Reserved18, // 0x18
      Reserved19, // 0x19
      Reserved1A, // 0x1A
      Reserved1B, // 0x1B
      Reserved1C, // 0x1C
      Reserved1D, // 0x1D
      Reserved1E, // 0x1E
      ToolMissing, // 0x1F
      TrackingNotEnabled, // 0x20
      ToolUnplugged; // 0x21

      public static Status findValue (int num) {
         if (num < values().length) {
            return values()[num];
         }
         else {
            return null;
         }
      }
   };

   public Transform() {
      toolHandle = 0x0000;
      status = 0x0100;
      q0 = BAD_FLOAT;
      qx = BAD_FLOAT;
      qy = BAD_FLOAT;
      qz = BAD_FLOAT;
      tx = BAD_FLOAT;
      ty = BAD_FLOAT;
      tz = BAD_FLOAT;
      error = BAD_FLOAT;
   }

   public Transform (Transform trans) {
      set (trans);
   }

   public void set (Transform trans) {
      toolHandle = trans.toolHandle;
      status = trans.status;
      q0 = trans.q0;
      qx = trans.qx;
      qy = trans.qy;
      qz = trans.qz;
      tx = trans.tx;
      ty = trans.ty;
      tz = trans.tz;
      error = trans.error;
   }
   
   /**
    * The index of the face being tracked on a multi-face tool.
    */
   byte getFaceNumber() {
      return (byte)((status & 0xE000) >>> 12);
   }

   /**
    * Returns the error code stored in the least significant 8 bits of the status.
    */
   byte getErrorCode() {
      return (byte)(status & 0x00FF);
   }

   /*
    * Returns true if status bit 8 is high, indicating the tool is missing.
    */
   boolean isMissing() {
      return (status & 0x0100) != 0x0000;
   }

};
