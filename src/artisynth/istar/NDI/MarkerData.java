package artisynth.istar.NDI;

import java.util.*;

public class MarkerData {

   byte status;

   int markerIndex;

   double x, y, z;

   public MarkerData() {
      markerIndex = 0;
      status = (byte)Status.Missing.ordinal();
      x = Transform.BAD_FLOAT;
      y = Transform.BAD_FLOAT;
      z = Transform.BAD_FLOAT;
   }

   public enum Status {
      OK, // 0x00
      Missing, // 0x01
      Reserved02, // 0x02
      Reserved03, // 0x03
      Reserved04, // 0x04
      OutOfVolume, // 0x05
      PossiblePhantom, // 0x06
      Saturated, // 0x07
      SaturatedOutOfVolume; // 0x08

      public static Status findValue (int num) {
         if (num < values().length) {
            return values()[num];
         }
         else {
            return null;
         }
      }
   };
}
