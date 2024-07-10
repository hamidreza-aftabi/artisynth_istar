package artisynth.istar.MaxillaRecon;

import java.awt.event.ActionEvent;
import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import javax.swing.AbstractAction;

import artisynth.core.driver.Main;
import artisynth.istar.Assist.Assist;
import artisynth.core.gui.editorManager.UndoManager;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.util.ArtisynthPath;
import artisynth.istar.Prisman.HelperMathFunctions;
import artisynth.istar.Prisman.HelperMeshFunctions;
import artisynth.istar.Prisman.ReconstructionModel;
import artisynth.istar.Prisman.undo.CreateDonorGuide;
import artisynth.istar.Prisman.undo.FibGuideCommand;
import maspack.collision.SurfaceMeshIntersector;
import maspack.geometry.BVIntersector;
import maspack.geometry.MeshFactory;
import maspack.geometry.OBB;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.geometry.io.GenericMeshWriter;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.Plane;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;

public class createDonorGuideClicked extends AbstractAction {
   ReconstructionModel root = null;
   MechModel mechModel = null;

   public createDonorGuideClicked (ReconstructionModel root,
   MechModel mechModel) {
      putValue (NAME, "Create Donor Guide");
      this.root = root;
      this.mechModel = mechModel;
   }
   @Override
   public void actionPerformed (ActionEvent evt) {
      CreateDonorGuide cmd = new CreateDonorGuide(root, mechModel, "Maxilla");
      Main main = Main.getMain ();
      UndoManager undo = main.getUndoManager ();
      undo.execute (cmd);
   }
}