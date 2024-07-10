package artisynth.istar.Prisman.undo;
import java.util.ArrayList;

import artisynth.core.gui.editorManager.Command;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.MechModel;



public class AddImplantsCommand implements Command {
      private String myName;
      private MechModel myModel;
      ArrayList<FixedMeshBody> myImplantBodies;

      public AddImplantsCommand(String name, ArrayList<FixedMeshBody> implantBodies, MechModel model) {
         myName = name;
         myImplantBodies = implantBodies;
         myModel = model;
      }

      public void execute () {
         myModel.addMeshBody (myImplantBodies.get(myImplantBodies.size () - 1 ));
         
      }

      public void undo () {
         myModel.removeMeshBody (myImplantBodies.get(myImplantBodies.size () -1 ));
         
      }

      public String getName () {
         return myName;
      }
}
