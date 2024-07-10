 package artisynth.istar.Atabak;

import java.util.ArrayList;

import artisynth.core.gui.editorManager.Command;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.core.mechmodels.RigidBody;
import maspack.collision.SurfaceMeshIntersector;
import maspack.geometry.PolygonalMesh;

public class ClipMandCommand implements Command {
   String myName = "ClipCommand";
   Assist Assist = new Assist ();
   MechModel mechModel = null;
   ReconstructionModel root = null;
   PolygonalMesh BackupMandible = new PolygonalMesh ();

   public ClipMandCommand (ReconstructionModel root, MechModel mechModel) {
      this.mechModel = mechModel;
      this.root = root;
      this.BackupMandible = Assist.GetMesh (mechModel, "Mandible").copy ();
   }

   public void execute () {
      // Check if the operation has already completed
      mechModel.removeRigidBody (Assist.GetRigidBody (mechModel, "BackupMandible"));
      if (Assist.GetRigidBody (mechModel, "Resection") != null) {
         System.out.println ("The mandible has already been cut");
      }
      else {
         if(Assist.GetRigidBody (mechModel, "BackupMandible")==null) {
            RigidBody BackupMandibleBody = new RigidBody("BackupMandible");
            BackupMandibleBody.setSurfaceMesh (BackupMandible);
            BackupMandibleBody.getRenderProps ().setVisible (false);
            mechModel.addRigidBody (BackupMandibleBody);
         }
         SurfaceMeshIntersector intersector =
            new maspack.collision.SurfaceMeshIntersector ();
         ArrayList<FixedMeshBody> PlaneList = new ArrayList<FixedMeshBody> ();
         RigidBody ResectionBody = new RigidBody ();
         // Generate a copy of the mandible body
         try {
            ResectionBody
               .setSurfaceMesh (Assist.GetMesh (mechModel, "Mandible").copy ());
            ResectionBody.setName ("Resection");
         }
         catch (Exception e) {
            System.out.println ("Could not clone mandible");
         }
         // Sort out all the planes in the model
         for (MeshComponent mesh : mechModel.meshBodies ()) {
            if (mesh.getName()!=null && mesh.getName ().contains ("Plane") && !mesh.getName().contains("Scapula")) {
               PlaneList.add ((FixedMeshBody)mesh);
            }
         }
         mechModel.addRigidBody (ResectionBody);

         // Find the mesh within the plane area using intersector
         for (FixedMeshBody plane : PlaneList) {
            PolygonalMesh resectionMesh =
               intersector
                  .findDifference01 (
                     ResectionBody.getSurfaceMesh (),
                     (PolygonalMesh)plane.getMesh ());
            root.rerender ();
            ResectionBody.setSurfaceMesh (resectionMesh);
         }
         // Filter out small intersection meshes
         if (ResectionBody
            .getSurfaceMesh ().partitionIntoConnectedMeshes () != null) {
            PolygonalMesh[] MeshList =
               ResectionBody.getSurfaceMesh ().partitionIntoConnectedMeshes ();
            MeshList[0].removeDisconnectedVertices ();
            MeshList[0].removeDisconnectedFaces ();
            ResectionBody.setSurfaceMesh (MeshList[0]);
         }
         // Attach the resection to the visibilty panel
         Assist
            .attachVisiblityToPanel (
               ResectionBody, root.getControlPanels ().get ("Visibility"));

         // Generate the unresected mandible by flipping all planes and taking
         // the difference and then flip back
         ArrayList<PolygonalMesh> MeshParts = new ArrayList<PolygonalMesh> ();
         for (FixedMeshBody plane : PlaneList) {
            ((PolygonalMesh)plane.getMesh ()).flip ();
            PolygonalMesh Testmesh =
               intersector
                  .findDifference01 (
                     Assist.GetMesh (mechModel, "Mandible"),
                     (PolygonalMesh)plane.getMesh ());
            MeshParts.add (Testmesh);
            ((PolygonalMesh)plane.getMesh ()).flip ();
            plane.getRenderProps ().setVisible (false);
         }
         
        
         // Union all of the Unresected mandible meshes into a single one and
         // set surface mesh
         PolygonalMesh UnResected = new PolygonalMesh ();
         for (PolygonalMesh meshpart : MeshParts) {
            UnResected = intersector.findUnion (UnResected, meshpart);
         }
         Assist
            .GetRigidBody (mechModel, "Mandible").setSurfaceMesh (UnResected);

         // Change the Visibility of mandible resection for logical progression
         Assist
            .GetRigidBody (mechModel, "Resection").getRenderProps ()
            .setVisible (false);
//         RigidBody Planee = new RigidBody ();
//         mechModel.addRigidBody (Planee);
//         Planee.setName ("Planee");
//         for (FixedMeshBody plane : PlaneList) {  
//            //((PolygonalMesh)plane.getMesh ()).flip ();
//            Planee
//               .setSurfaceMesh ((PolygonalMesh)plane.getMesh ());
//            Planee.getRenderProps ().setVisible (false);
//         }
      }
   }

   public void undo () {
      mechModel.removeRigidBody (Assist.GetRigidBody (mechModel, "Resection"));
      Assist
         .GetRigidBody (mechModel, "Mandible").setSurfaceMesh (BackupMandible);
      for (MeshComponent mesh : mechModel.meshBodies ()) {
         if (mesh.getName ().contains ("Plane")) {
            mesh.getRenderProps ().setVisible (true);
         }
      }
   }

   public String getName () {
      return myName;
   }
}
