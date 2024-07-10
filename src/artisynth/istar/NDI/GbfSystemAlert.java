package artisynth.istar.NDI;

import java.util.*;

/**
 * This class encapsulates system alerts as they are read from BX2
 */
public class GbfSystemAlert extends GbfComponent {

   ArrayList<SystemAlert> data;

   public GbfSystemAlert (BufferedReader reader, int itemCount) {
      data = new ArrayList<>();
      for (int j = 0; j < itemCount; j++) {
         SystemAlert alert = new SystemAlert();
         alert.conditionType = reader.get_byte();
         reader.get_byte(); // reserved
         alert.conditionCode = reader.get_uint16();
         data.add(alert);
      }
   }

   public String toString() {
      StringBuilder sb = new StringBuilder();
      sb.append ("----GbfSystemAlert \n");
      sb.append (super.toString());
      for ( int i = 0; i < data.size(); i++) {
         sb.append ("--Alert: conditionType=");
         sb.append (Utils.byteToHexString(data.get(i).conditionType));
         sb.append (", conditionCode=");
         sb.append (Utils.shortToHexString((short)data.get(i).conditionCode));
         sb.append ("\n");
      }
      return sb.toString();     
   }
   
}
