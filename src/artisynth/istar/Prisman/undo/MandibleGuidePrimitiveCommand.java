package artisynth.istar.Prisman.undo;

import java.util.List;
import artisynth.core.mechmodels.*;
import artisynth.core.gui.editorManager.Command;

public class MandibleGuidePrimitiveCommand implements Command {
   private String myName;
   private FixedMeshBody[] OB;
   private FixedMeshBody[] IB;
   private FixedMeshBody CB;
   private List<FixedMeshBody> SB;
   private FixedMeshBody B1;
   private FixedMeshBody B2;
   private FixedMeshBody MG;
   private MechModel MM;
   private boolean c;
   private boolean h;

   public MandibleGuidePrimitiveCommand (String name, FixedMeshBody[] OutBody,
   FixedMeshBody[] InBody, FixedMeshBody connectorBody,
   List<FixedMeshBody> screwBody, FixedMeshBody base1, FixedMeshBody base2,
   FixedMeshBody MasterGuide, MechModel model, boolean connector,
   boolean oneholder) {
      myName = name;
      OB = OutBody;
      IB = InBody;
      CB = connectorBody;
      SB = screwBody;
      B1 = base1;
      B2 = base2;
      MG = MasterGuide;
      MM = model;
      c = connector;
      h = oneholder;
   }

   public void execute () {
      for (int i = 0; i < 2; i++) {
         MM.addMeshBody (OB[i]);
      }
      for (int i = 0; i < 2; i++) {
         MM.addMeshBody (IB[i]);
      }
      if (!c) {
         MM.addMeshBody (CB);
      }
      for (int i = 0; i < SB.size (); i++) {
         MM.addMeshBody (SB.get (i));
      }
      MM.addMeshBody (B1);
      if (!h) {
         MM.addMeshBody (B2);
      }
      MG.getRenderProps ().setVisible (false);
   }

   public void undo () {
      for (int i = 0; i < 2; i++) {
         MM.removeMeshBody (OB[i]);
      }
      for (int i = 0; i < 2; i++) {
         MM.removeMeshBody (IB[i]);
      }
      if (!c) {
         MM.removeMeshBody (CB);
      }
      for (int i = 0; i < SB.size (); i++) {
         MM.removeMeshBody (SB.get (i));
      }
      MM.removeMeshBody (B1);
      if (!h) {
         MM.removeMeshBody (B2);
      }
      MG.getRenderProps ().setVisible (true);
   }

   public String getName () {
      return myName;
   }
}
