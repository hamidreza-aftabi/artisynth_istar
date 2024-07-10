package artisynth.istar.Prisman.undo;

import java.util.List;
import artisynth.core.mechmodels.*;
import artisynth.core.gui.editorManager.Command;

public class TransformCommand implements Command {
   private String myName;
   private MechModel myModel;
   private List<FixedMeshBody> temp;
   private int length;
   private List<FixedMeshBody> fibulasegmented;

   public TransformCommand (String name, MechModel model,
   List<FixedMeshBody> fibulasegments, List<FixedMeshBody> temps,
   int numberOfSegments) {
      myName = name;
      myModel = model;
      fibulasegmented = fibulasegments;
      temp = temps;
      length = numberOfSegments;
   }

   public void execute () {
      for (int i = 0; i < length; i++) {
         if (!fibulasegmented.isEmpty ()) {
            myModel.addMeshBody (fibulasegmented.get (i));
         }
//            myModel.addMeshBody (temp.get (2 * i));
//            myModel.addMeshBody (temp.get (2 * i + 1));
      }

   }

   public void undo () {
      for (int i = 0; i < length; i++) {
         if (!fibulasegmented.isEmpty ()) {
            myModel.removeMeshBody (fibulasegmented.get (i));
         }
            myModel.removeMeshBody (temp.get (2 * i));
            myModel.removeMeshBody (temp.get (2 * i + 1));
      }
   }

   public String getName () {
      return myName;
   }
}
