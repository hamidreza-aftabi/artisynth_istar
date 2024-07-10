package artisynth.istar.NDI;

import java.util.*;

/**
 * This class encapsulates system alerts as they are read from BX2
 */
public class GbfComponent {
   short componentType;
   int componentSize;
   short itemOption;
   int itemCount;

   static enum Type {
      Reserved00, // 0x00
      Frame, // 0x0001
      Data6D, // 0x0002
      Data3D, // 0x0003
      Button1D, // 0x0004
      Data2D, // 0x0005
      Reserved06, // 0x06
      Reserved07, // 0x07
      Reserved08, // 0x08
      Reserved09, // 0x09
      Reserved0a, // 0x0a
      Reserved0b, // 0x0b
      Reserved0c, // 0x0c
      Reserved0d, // 0x0d
      Reserved0e, // 0x0e
      Reserved0f, // 0x0f
      Reserved10, // 0x10
      UV, // 0x0011
      SystemAlert; // 0x0012

      public static Type findValue (int num) {
         if (num < values().length) {
            return values()[num];
         }
         else {
            return null;
         }
      }
   };

   static GbfComponent buildComponent(BufferedReader reader) {
      	// Read the component header information
      GbfComponent component = null;
      short componentType = reader.get_uint16();
      int componentSize = reader.get_uint32();
      short itemOption = reader.get_uint16();
      int itemCount = reader.get_uint32();

      // Read data into the appropriate class based on the componentType
      Type type = Type.findValue (componentType);
      switch (type) {
         case Frame: {
            component = new GbfFrame(reader, itemCount);
            break;
         }
         case Data6D: {
            component = new GbfData6D(reader, itemCount);
            break;
         }
         case Data3D: {
            component = new GbfData3D (reader, itemCount);
            break;
         }
         case Button1D: {
            component = new GbfButton1D (reader, itemCount);
            break;
         }
         case SystemAlert: {
            component = new GbfSystemAlert (reader, itemCount);
            break;
         }
         default: {
            type = null;
         }
      }
      if (type == null) {
         // TODO: Not implement yet - GbfComponentTypes: Data2D, UV

	// Skip unrecognized items the size minus the 12 byte header we just read
	
	reader.skipBytes(componentSize - 12);
        component = new GbfComponent();
      }

      // Write the header information that we read earlier
      component.componentType = componentType;
      component.componentSize = componentSize;
      component.itemOption = itemOption;
      component.itemCount = itemCount;
      
      return component;
   }

   public String toString() {
      StringBuilder sb = new StringBuilder();
      sb.append ("componentType=" + Utils.shortToHexString(componentType));
      sb.append ("("+Type.findValue(componentType)+")\n");
      sb.append ("componentSize=" + Utils.intToHexString(componentSize)+"\n");
      sb.append ("itemOption=" + Utils.shortToHexString(itemOption)+"\n");
      sb.append ("itemCount=" + Utils.intToHexString(itemCount)+"\n");
      return sb.toString();
   }

}
