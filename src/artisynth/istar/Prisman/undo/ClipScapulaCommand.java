package artisynth.istar.Prisman.undo;

import artisynth.core.mechmodels.*;
import artisynth.istar.Assist.Assist;
import artisynth.istar.Prisman.ReconstructionModel;
import maspack.geometry.*;

import java.util.ArrayList;
import java.util.List;

import artisynth.core.gui.editorManager.Command;

//edit for n number planes
public class ClipScapulaCommand implements Command {
   MechModel mechModel = null;
   ReconstructionModel root = null;
   Assist Assist = new Assist ();

  public ClipScapulaCommand (ReconstructionModel root,
     MechModel mechModel) {
     this.mechModel = mechModel;
     this.root = root;

  }

  public void execute () {
     PolygonalMesh ScapulaSegment = root.Intersection.getCSGMesh ();
     RigidBody ScapulaSegmentBody = new RigidBody("DonorSegment");
     ScapulaSegmentBody.setSurfaceMesh (ScapulaSegment);
     mechModel.addRigidBody (ScapulaSegmentBody);
     Assist.GetMeshBody (mechModel, "Donor").getRenderProps ().setVisible (false);
     root.Intersection.setRenderCSGMesh (false);
     root.removeController (root.Intersection);
     PolygonalMesh recon = MeshFactory.getUnion (ScapulaSegment, Assist.GetMesh (mechModel, "ClippedMaxilla"));
     RigidBody Reconstruction = new RigidBody("ReconstructedMaxilla");
     Reconstruction.setSurfaceMesh (recon);
     mechModel.addRigidBody (Reconstruction);
     PolygonalMesh UnresectedDonorMesh = MeshFactory.getSubtraction (Assist.GetMesh (mechModel, "Donor"), Assist.GetMesh (mechModel, "Intersect"));
     RigidBody UnresectedDonorBody = new RigidBody("UnresectedDonor");
     UnresectedDonorBody.setSurfaceMesh (UnresectedDonorMesh);
     mechModel.addRigidBody(UnresectedDonorBody);
     UnresectedDonorBody.getRenderProps().setVisible (false);
     Reconstruction.getRenderProps ().setVisible (false);
     Assist
     .attachVisiblityToPanel (
        Reconstruction, root.getControlPanels ().get ("Visibility"));
     Assist
     .attachVisiblityToPanel (
        UnresectedDonorBody, root.getControlPanels ().get ("Visibility"));
     Assist
     .attachVisiblityToPanel (
        ScapulaSegmentBody, root.getControlPanels ().get ("Visibility"));

  }

  public void undo () {
     mechModel.removeRigidBody (Assist.GetRigidBody (mechModel, "ReconstructedMaxilla"));
     mechModel.removeRigidBody (Assist.GetRigidBody (mechModel, "UnresectedDonor"));
     mechModel.removeRigidBody (Assist.GetRigidBody (mechModel, "DonorSegment"));
     Assist.GetMeshBody (mechModel, "Donor").getRenderProps ().setVisible (true);
     root.Intersection.setRenderCSGMesh (true);
     root.addController (root.Intersection);

  }

  public String getName () {
    return "Clip Scapula";
  }
}
