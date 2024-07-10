package artisynth.istar.NDI;

import java.util.*;

/**
 * The most basic GBF object stores only a version and the number of items to read.
 */
public class GbfContainer
{
   // Public member data
   short gbfVersion;
   short componentCount;
   ArrayList<GbfComponent> components;

   /**
    * Parses data from a BX2 reply that has been read into the buffer.
    *
    * @param reader The BufferedReader which has already read an entire reply.
    */
   GbfContainer (BufferedReader reader) {
      components = new ArrayList<>();
      // Read the GBF header info so we know how to parse the data
      gbfVersion = reader.get_uint16();
      componentCount = reader.get_uint16();

      // Read each item in the container
      for (int i = 0; i < componentCount; i++ ) {
         // Push the component to the std::vector and read the next one...
         components.add (GbfComponent.buildComponent(reader));
      }     
   }

   /**
    * Frees memory for any GbfComponents that were created.
    */
   void dispose() {
   }

   /**
    * Returns a string representation of the data for debugging purposes.
    */
   public String toString() {
      StringBuilder sb = new StringBuilder();

      sb.append ("----GbfContainer \n");
      sb.append ("gbfVersion=" + Utils.shortToHexString(gbfVersion) + "\n");
      sb.append ("componentCount="+Utils.shortToHexString(componentCount) +"\n");
      for ( int i = 0; i < components.size(); i++) {
         sb.append (components.get(i).toString());
      }
      return sb.toString();
   }
};

