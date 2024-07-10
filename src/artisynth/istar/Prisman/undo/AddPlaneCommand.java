package artisynth.istar.Prisman.undo;

import java.awt.Color;
import java.util.ArrayList;

import artisynth.core.driver.Main;
import artisynth.core.gui.editorManager.Command;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.core.mechmodels.RigidBody;
import artisynth.istar.Assist.Assist;
import artisynth.istar.Prisman.ReconstructionModel;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;

public class AddPlaneCommand implements Command {
   String myName = "Add Plane";
   Assist Assist = new Assist ();
   MechModel mechModel = null;
   ReconstructionModel root = null;

   public AddPlaneCommand (ReconstructionModel root, MechModel mechModel) {
      this.mechModel = mechModel;
      this.root = root;
   }

   public void execute () {
      ArrayList<FixedMeshBody> planeBodies = new ArrayList<FixedMeshBody>();
      for (MeshComponent meshbody: mechModel.meshBodies ()) {
         if (meshbody.getName ()!=null && meshbody.getName ().contains ("plane")) {
            planeBodies.add ((FixedMeshBody)meshbody);
         }
      }
      PolygonalMesh plane = MeshFactory.createPlane (200, 200,10,10);
      FixedMeshBody planeBody = new FixedMeshBody(plane);
      planeBody.setName ("plane"+(planeBodies.size ()+1));
      planeBody.setPosition (Assist.GetRigidBody (mechModel, "Maxilla").getPosition ());
      Main.getMain ().rerender ();
      mechModel.addMeshBody (planeBody);
      
      Main.getMain ().setSelectionMode (Main.SelectionMode.Transrotate);
      for (MeshComponent meshbody: mechModel.meshBodies ()) {
         if (meshbody.isSelected ()) {
            meshbody.setSelected (false);
         }
      }
      for (RigidBody meshbody: mechModel.rigidBodies ()) {
         if(meshbody.isSelected ()) {
            meshbody.setSelected (false);
         }
      }
      planeBody.setSelected (true);
      Main.getMain ().rerender ();
      
   }

   public void undo () {
      ArrayList<FixedMeshBody> planeBodies = new ArrayList<FixedMeshBody>();
      for (MeshComponent meshbody: mechModel.meshBodies ()) {
         if (meshbody.getName ()!=null && meshbody.getName ().contains ("plane")) {
            planeBodies.add ((FixedMeshBody)meshbody);
         }
      }
      mechModel.removeMeshBody (Assist.GetMeshBody (mechModel, "plane"+planeBodies.size()));
   }

   public String getName () {
      return myName;
   }
}