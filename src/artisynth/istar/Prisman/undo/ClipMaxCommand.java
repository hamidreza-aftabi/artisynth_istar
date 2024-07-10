package artisynth.istar.Prisman.undo;

import artisynth.core.mechmodels.*;

import artisynth.core.gui.editorManager.Command;

public class ClipMaxCommand implements Command {
   private String myName;
   private FixedMeshBody mynonresect;
   private FixedMeshBody myresect;
   private FixedMeshBody myplane1;
   private FixedMeshBody myplane2;
   private FixedMeshBody myplane3;
   private MechModel myModel;
   private RigidBody maxillaBody;
   private FixedMeshBody mybox1, mybox2, mybox3, mybox4;
   private FixedMeshBody interbox;
 
   public ClipMaxCommand (String name, RigidBody maxilla,
      FixedMeshBody nonresection, FixedMeshBody resection, FixedMeshBody plane1,
      FixedMeshBody plane2, FixedMeshBody plane3, MechModel model, FixedMeshBody box1, FixedMeshBody box2, 
      FixedMeshBody box3, FixedMeshBody intersectedbox) {

      myName = name;
      myModel = model;
      mynonresect = nonresection;
      myresect = resection;
      myplane1 = plane1;
      myplane2 = plane2;
      maxillaBody = maxilla;
      myplane3 = plane3;
      mybox1 = box1;
      mybox2 = box2;
      mybox3 = box3;
      interbox = intersectedbox;
   }
 
   public void execute () {
      myModel.addMeshBody (mynonresect);
      myModel.addMeshBody (myresect);
      myModel.addMeshBody (mybox1);
      myModel.addMeshBody (mybox2);
      myModel.addMeshBody (mybox3);
      myModel.addMeshBody (interbox);
      maxillaBody.getRenderProps ().setVisible (false);
      myresect.getRenderProps ().setVisible (false);
      myplane1.getRenderProps ().setVisible (false);
      myplane2.getRenderProps ().setVisible (false);
      myplane3.getRenderProps ().setVisible (false);
      mybox1.getRenderProps ().setVisible (false);
      mybox2.getRenderProps ().setVisible (false);
      mybox3.getRenderProps ().setVisible (false);
      interbox.getRenderProps ().setVisible (false);
   }
 
   public void undo () {
      myModel.removeMeshBody (mynonresect);
      myModel.removeMeshBody (myresect);
      myModel.removeMeshBody (mybox1);
      myModel.removeMeshBody (mybox2);
      myModel.removeMeshBody (mybox3);
      myModel.removeMeshBody (interbox);
      maxillaBody.getRenderProps ().setVisible (true);
      myplane1.getRenderProps ().setVisible (true);
      myplane2.getRenderProps ().setVisible (true);
      myplane3.getRenderProps ().setVisible (true); 
   }
 
   public String getName () {
      return myName;
   }
}
