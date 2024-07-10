package artisynth.istar.NDI;

import java.util.*;
import maspack.util.*;

/**
 * This class encapsulates 3D marker data as it is read from BX2
 */
public class GbfData3D extends GbfComponent {

   //! Tool handles
   DynamicIntArray toolHandles;

   //! MarkerData associated with the tool whose handle is stored at the same array index
   ArrayList<ArrayList<MarkerData>> markers;

   public GbfData3D (BufferedReader reader, int numberOfTools) {
      toolHandles = new DynamicIntArray();
      markers = new ArrayList<>();

      short toolHandle = 0x00;
      short numberOf3Ds = 0x00;
      for (int i = 0; i < numberOfTools; i++) {
         // Read the data
         toolHandle = reader.get_uint16();
         numberOf3Ds = reader.get_uint16();

         // Put the toolHandle into its own vector
         toolHandles.add(toolHandle);

         // Create a corresponding vector with the 3Ds
         ArrayList<MarkerData> list3Ds = new ArrayList<>();
         for ( int k = 0; k < numberOf3Ds; k++) {
            MarkerData pos = new MarkerData();
            pos.status = reader.get_byte();
            reader.get_byte(); // reserved
            pos.markerIndex = reader.get_uint16();
            if (pos.status == MarkerData.Status.Missing.ordinal()) {
               pos.x = Transform.BAD_FLOAT;
               pos.y = Transform.BAD_FLOAT;
               pos.z = Transform.BAD_FLOAT;
            }
            else {
               pos.x = reader.get_double();
               pos.y = reader.get_double();
               pos.z = reader.get_double();
            }
            list3Ds.add(pos);
         }
         markers.add(list3Ds);
      }      
   }

   public String toString() {
      StringBuilder sb = new StringBuilder();
      sb.append ("----GbfData3D \n");
      sb.append (super.toString());

      // For each tool, print the tool handle and all of its associated 3Ds
      for (int toolIndex = 0; toolIndex < toolHandles.size(); toolIndex++) {
         short handle = (short)toolHandles.get(toolIndex);
         ArrayList<MarkerData> mkrdata = markers.get (toolIndex);
         sb.append ("toolHandleReference="+Utils.shortToHexString(handle)+"\n");
         sb.append ("numberOf3Ds=" + mkrdata.size() + "\n");
         for ( int i = 0; i < mkrdata.size(); i++) {
            MarkerData data = mkrdata.get(i);
            sb.append ("--Data3D: status=" + Utils.byteToHexString(data.status));
            sb.append (" (" + MarkerData.Status.findValue (data.status) + ")");
            sb.append (", markerIndex=" + data.markerIndex);
            sb.append (", [x y z] = [");
            NumberFormat fmt = new NumberFormat ("%10.4f");
            sb.append (fmt.format(data.x)+",");
            sb.append (fmt.format(data.y)+",");
            sb.append (fmt.format(data.z)+"]\n");
         }
      }
      return sb.toString();
   }
}
