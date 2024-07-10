 package artisynth.istar.Atabak;

import java.awt.event.ActionEvent;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

import javax.swing.AbstractAction;

import artisynth.core.driver.Main;
import artisynth.core.gui.editorManager.UndoManager;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.RenderableComponentList;
import maspack.collision.SurfaceMeshIntersector;
import maspack.geometry.BVIntersector;
import maspack.geometry.MeshFactory;
import maspack.geometry.OBB;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.interpolation.NumericList;
import maspack.interpolation.NumericListKnot;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.Plane;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;

public class TransformButtonClicked extends AbstractAction {
   /**
    * 
    */
   private static final long serialVersionUID = -4357198968454315337L;
   /**
    * 
    */
   ReconstructionModel root = null;
   Assist Assist = new Assist ();
   MechModel mechModel = null;
   PolygonalMesh[] nonResectionFibulaMeshes;
   PolygonalMesh[] resectionFibulaMeshes;
   ImprovedFormattedTextField DonorDistanceProx;
   ImprovedFormattedTextField DonorDistanceDis;
   ImprovedFormattedTextField multiplier;
   double SAW_BLD_THICK;

   public TransformButtonClicked (ReconstructionModel root, MechModel mechModel,
   ImprovedFormattedTextField DonorDistanceProx,
   ImprovedFormattedTextField DonorDistanceDis,
   ImprovedFormattedTextField multiplier, double SAW_BLD_THICK) {
      putValue (NAME, "Transform");
      this.root = root;
      this.mechModel = mechModel;
      this.DonorDistanceDis = DonorDistanceDis;
      this.DonorDistanceProx = DonorDistanceProx;
      this.multiplier = multiplier;
      this.SAW_BLD_THICK = SAW_BLD_THICK;
   }

   @Override
   public void actionPerformed (ActionEvent evt) {
      Transform cmd = new Transform (root, mechModel,
         DonorDistanceProx,
         DonorDistanceDis,
         multiplier, SAW_BLD_THICK);
      Main main = Main.getMain ();
      UndoManager undo = main.getUndoManager ();
      undo.execute (cmd);
   }
}
