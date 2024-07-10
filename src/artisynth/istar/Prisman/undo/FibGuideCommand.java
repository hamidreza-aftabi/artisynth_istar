package artisynth.istar.Prisman.undo;

import java.util.List;
import artisynth.core.mechmodels.*;
import artisynth.core.gui.editorManager.Command;

public class FibGuideCommand implements Command {
   private String myName;
   private MechModel myModel;
   private List<FixedMeshBody> large;
   private int length;
   private List<FixedMeshBody> small;
   private FixedMeshBody base;
   private FixedMeshBody cylinder;
   private FixedMeshBody guide;
   private FixedMeshBody RFDHP;
   private FixedMeshBody r;
   private boolean d;

   public FibGuideCommand (String name, MechModel model, FixedMeshBody cbody,
   FixedMeshBody baseBody, int numberOfSegments,
   List<FixedMeshBody> largeBodies, List<FixedMeshBody> smallBodies,
   FixedMeshBody FibulaGuideBody, FixedMeshBody RFDH,
   FixedMeshBody reconstruction, boolean b) {
      myName = name;
      myModel = model;
      cylinder = cbody;
      base = baseBody;
      length = numberOfSegments;
      large = largeBodies;
      small = smallBodies;
      guide = FibulaGuideBody;
      RFDHP = RFDH;
      r = reconstruction;
      d = b;
   }

   public void execute () {

      myModel.addMeshBody (cylinder);
      cylinder.getRenderProps ().setVisible (false);
      myModel.addMeshBody (base);
      base.getRenderProps ().setVisible (false);
      myModel.addMeshBody (guide);
      if (!d) {
         r.getRenderProps ().setVisible (false);
         myModel.addMeshBody (RFDHP);
      }
      for (int i = 0; i < length; i++) {
         myModel.addMeshBody (large.get (2 * i));
         myModel.addMeshBody (large.get (2 * i + 1));
         myModel.addMeshBody (small.get (2 * i));
         myModel.addMeshBody (small.get (2 * i + 1));
         large.get (2 * i).getRenderProps ().setVisible (false);
         large.get (2 * i + 1).getRenderProps ().setVisible (false);
         small.get (2 * i).getRenderProps ().setVisible (false);
         small.get (2 * i + 1).getRenderProps ().setVisible (false);
      }

   }

   public void undo () {

      for (int i = 0; i < length; i++) {
         myModel.removeMeshBody (large.get (2 * i));
         myModel.removeMeshBody (large.get (2 * i + 1));
         myModel.removeMeshBody (small.get (2 * i));
         myModel.removeMeshBody (small.get (2 * i + 1));
      }
      myModel.removeMeshBody (guide);
      if (!d) {
         myModel.removeMeshBody (RFDHP);
         r.getRenderProps ().setVisible (true);
      }

   }

   public String getName () {
      return myName;
   }
}
