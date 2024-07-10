package artisynth.istar.NDI;

import java.util.*;
import maspack.util.*;

/**
 * This class encapsulates button data as it is read from BX2
 */
public class GbfButton1D extends GbfComponent {

   //! The tool the data is associated with
   short toolHandle;

   //! An 8-bit integer for each button representing its state
   DynamicByteArray data;

   public GbfButton1D (BufferedReader reader, int itemCount) {
      data = new DynamicByteArray();
      if (itemCount > 0) {
         toolHandle = reader.get_uint16();
         short numberOfButtons = reader.get_uint16();
         for (int j = 0; j < numberOfButtons; j++) {
            data.add (reader.get_byte());
         }
      }
   }

   /**
    * Returns a string representation of the data for debugging purposes.
    */
   public String toString() {
      StringBuilder sb = new StringBuilder();
      sb.append ("----GbfButton1D \n");
      sb.append (super.toString());
      if (data.size() > 0) {
         sb.append (
            "toolHandleReference="+Utils.shortToHexString(toolHandle)+"\n");
         sb.append ("numberOfButtons="+data.size()+"\n");
         for ( int i = 0; i < data.size(); i++) {
            sb.append (Utils.byteToHexString(data.get(i)) + " ");
         }
         sb.append ("\n");
      }
      return sb.toString();
   }

};
