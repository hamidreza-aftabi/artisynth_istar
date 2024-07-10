package artisynth.istar.Prisman.undo;
import java.util.ArrayList;
import java.util.function.Consumer;

import artisynth.core.gui.editorManager.Command;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.istar.Prisman.ReconstructionModel;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;



public class DentalImplantCommand implements Command {
      private String myName = "Dental Implant Command";
      private MechModel myModel;
      private MechModel mechModel;
      private ReconstructionModel root;

      public DentalImplantCommand(ReconstructionModel root, MechModel mechModel) {
         this.root = root;
         this.mechModel = mechModel;
      }

      public void execute () {
         ArrayList<MeshComponent> implants = new ArrayList<>();
         for (MeshComponent mesh : mechModel.meshBodies ()) {
            if (mesh.getName ()!= null && mesh.getName ().contains ("Dental Implant")) {
               implants.add (mesh);
            }
         }
         PolygonalMesh implantMesh = MeshFactory.createCylinder (2, 50, 10);
         implantMesh.setName ("Implant"+implants.size ()+1);
         FixedMeshBody implantMeshBody = new FixedMeshBody(implantMesh);
         implantMeshBody.setName ("ImplantBody"+implants.size ()+1);
         mechModel.addMeshBody(implantMeshBody);
      }

      public void undo () {
         ArrayList<MeshComponent> implants = new ArrayList<>();
         for (MeshComponent mesh : mechModel.meshBodies ()) {
            if (mesh.getName ()!= null && mesh.getName ().contains ("Dental Implant")) {
               implants.add (mesh);
            }
         }
         mechModel.removeMeshBody (implants.get (implants.size ()-1));
      }

      public String getName () {
         return myName;
      }
}
