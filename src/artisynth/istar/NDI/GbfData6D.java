package artisynth.istar.NDI;

import java.util.*;
import maspack.util.*;

/**
 * This class encapsulates 6D transformation data as its read from BX2
 */
public class GbfData6D extends GbfComponent {

   public ArrayList<Transform> toolTransforms;

   public GbfData6D (BufferedReader reader, int numberOfTools) {
      toolTransforms = new ArrayList<>();

      for ( int i = 0; i < numberOfTools; i++) {
         Transform data6D = new Transform();
         data6D.toolHandle = reader.get_uint16();
         data6D.status = reader.get_uint16();

         if (data6D.isMissing()) {
            data6D.q0 = Transform.BAD_FLOAT;
            data6D.qx = Transform.BAD_FLOAT;
            data6D.qy = Transform.BAD_FLOAT;
            data6D.qz = Transform.BAD_FLOAT;
            data6D.tx = Transform.BAD_FLOAT;
            data6D.ty = Transform.BAD_FLOAT;
            data6D.tz = Transform.BAD_FLOAT;
            data6D.error = Transform.BAD_FLOAT;
         }
         else {
            data6D.q0 = reader.get_double();
            data6D.qx = reader.get_double();
            data6D.qy = reader.get_double();
            data6D.qz = reader.get_double();
            data6D.tx = reader.get_double();
            data6D.ty = reader.get_double();
            data6D.tz = reader.get_double();
            data6D.error = reader.get_double();
         }
         toolTransforms.add(data6D);
      }
   }

   public String toString() {
      StringBuilder sb = new StringBuilder();
      sb.append ("-----GbfData6D \n");
      sb.append (super.toString());
      for ( int i = 0; i < toolTransforms.size(); i++) {
         Transform trans = toolTransforms.get(i);
         sb.append ("toolHandle="+Utils.shortToHexString(trans.toolHandle)+"\n");
         sb.append ("status="+Utils.shortToHexString(trans.status));
         if (trans.isMissing()) {
            sb.append (" - MISSING ");
         }
         sb.append (", error="+Utils.byteToHexString(trans.getErrorCode()));
         sb.append (" ("+ Transform.Status.findValue(trans.getErrorCode()) + ")\n");
         NumberFormat fmt = new NumberFormat ("%10.4f");
         sb.append ("transform=[q0, qx, qy, qz, tx, ty, tz, error ] = [");
         sb.append (fmt.format (trans.q0) + ",");
         sb.append (fmt.format (trans.qx) + ",");
         sb.append (fmt.format (trans.qy) + ",");
         sb.append (fmt.format (trans.qz) + ",");
         sb.append (fmt.format (trans.tx) + ",");
         sb.append (fmt.format (trans.ty) + ",");
         sb.append (fmt.format (trans.tz) + ",");
         sb.append (fmt.format (trans.error) + "]\n");
      }     
      return sb.toString();
   }
}
