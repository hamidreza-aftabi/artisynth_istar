package artisynth.istar.Rui;

import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;

import artisynth.core.modelbase.ControllerBase;
import artisynth.core.modelbase.RenderableComponentBase;
import maspack.collision.ContactInfo;
import maspack.collision.IntersectionContour;
import maspack.collision.IntersectionPoint;
import maspack.collision.SurfaceMeshIntersector;
import maspack.collision.SurfaceMeshIntersector.CSG;
import maspack.collision.SurfaceMeshIntersector.RegionType;
import maspack.collision.SurfaceMeshIntersectorTest;
import maspack.geometry.PolygonalMesh;
import maspack.properties.HasProperties;
import maspack.properties.PropertyList;
import maspack.render.RenderList;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.render.Renderer.*;
import maspack.util.FunctionTimer;
import maspack.util.InternalErrorException;

public class Intersector extends RenderableComponentBase {

   private SurfaceMeshIntersector myIntersector;
   private boolean myContoursOnly = true;
   private boolean myRenderDiffMesh = false;
   private PolygonalMesh myMesh0;
   private PolygonalMesh myMesh1;
   private ContactInfo myContactInfo;
   public ArrayList<IntersectionContour> myContours;
   private PolygonalMesh myDiffMesh;
   private Color myColor;

   public boolean getRenderDiffMesh() {
      return myRenderDiffMesh;
   }
   
   public ArrayList<IntersectionContour> getContours(){
      myContours = myIntersector.findContours(myMesh0, myMesh1);
      return myContours;
   }

   public void setRenderDiffMesh (boolean enable) {
      if (enable != myRenderDiffMesh) {
         myRenderDiffMesh = enable;
      }
   }

   public Intersector (
      PolygonalMesh mesh0, PolygonalMesh mesh1, double maxPenetrationDistance, Color color) {
      super ();
      myMesh0 = mesh0;
      myMesh1 = mesh1;
      myIntersector = new SurfaceMeshIntersector();
      myContactInfo = null;
      myContours = null;
      setRenderProps (createRenderProps ());
      myColor =color;
   }

   public static PropertyList myProps = new PropertyList (
      Intersector.class, RenderableComponentBase.class);

   static {
      myProps.get("renderProps").setDefaultValue(defaultRenderProps(null));
      myProps.add (
         "renderDiffMesh", "controls whether diff mesh is rendered",
         false);
      myProps.add (
         "contoursOnly", 
         "compute intersection contours but not diff mesh", true);     
   }

   public PropertyList getAllPropertyInfo () {
      return myProps;
   }

   public PolygonalMesh getDiffMesh() {
      return myDiffMesh;
   }
   
   public boolean getContoursOnly() {
      return myContoursOnly;
   }
   
   public void setContoursOnly (boolean enable) {
      myContoursOnly = enable;
   }

   void updateDiffMesh() {
      PolygonalMesh csgMesh = null;
      try {
         csgMesh = myIntersector.findDifference01 (myMesh0, myMesh1);
      }
      catch (InternalErrorException e) {
         System.out.println (e);
         try {
            SurfaceMeshIntersectorTest.writeProblem (
               "diffop.obj", myMesh0, myMesh1, null, CSG.DIFFERENCE01);
         }
         catch (IOException ioe) {
            ioe.printStackTrace();
         }
      }
      myDiffMesh = csgMesh;
   }
   
   @Override
   public void prerender (RenderList list) {

      super.prerender (list);
      PolygonalMesh csgMesh = null;
      FunctionTimer timer = new FunctionTimer();
      if (myContoursOnly) {
         timer.start();
         myContours = myIntersector.findContours(myMesh0, myMesh1);
         timer.stop();
         if (myContours.size() == 0) {
            return;
         }
      }
      else {
         timer.start();
         try {
            myContactInfo = myIntersector.findContoursAndRegions (
               myMesh0, RegionType.OUTSIDE, myMesh1, RegionType.INSIDE);
            csgMesh = myIntersector.createCSGMesh (myContactInfo);
         }
         catch (InternalErrorException e) {
            System.out.println (e);
            try {
               SurfaceMeshIntersectorTest.writeProblem (
                  "csgop.obj", myMesh0, myMesh1, null, CSG.DIFFERENCE01);
            }
            catch (IOException ioe) {
               ioe.printStackTrace();
            }
         }
         timer.stop();
         if (myContactInfo == null) {
            myContours = null;
            return;
         }
         myContours = myContactInfo.getContours();
         if (myRenderDiffMesh && csgMesh != null) {
            csgMesh.prerender (myRenderProps);
            myDiffMesh = csgMesh;
         }
         else {
            myDiffMesh = null;
         }

      }
      //System.out.println ("time=" + timer.result(1));
   }
   
   @Override
   public void render (Renderer renderer, int flags) {
      ArrayList<IntersectionContour> contours = myContours;
      if (contours != null) {
         Shading savedShading = renderer.getShading();
         renderer.setShading (Shading.NONE);
         renderer.setLineWidth (3);
         renderer.setColor (myColor);
         for (IntersectionContour c : contours) {
            if (c.isClosed()) {
               renderer.beginDraw (DrawMode.LINE_LOOP);
            }
            else {
               renderer.beginDraw (DrawMode.LINE_STRIP);
            }
            for (IntersectionPoint p : c) {
               renderer.addVertex ((float)p.x, (float)p.y, (float)p.z);
            }
            renderer.endDraw();
         }           
         renderer.setShading (savedShading);
      }
      PolygonalMesh csgMesh = myDiffMesh;
      if (myRenderDiffMesh && csgMesh != null) {
         csgMesh.render (renderer, myRenderProps, flags);
      }
   }

   static protected RenderProps defaultRenderProps (HasProperties host) {
      RenderProps props = RenderProps.createRenderProps (host);
      props.setFaceStyle (FaceStyle.FRONT);
      return props;
   }

   @Override
   public RenderProps createRenderProps () {
      return defaultRenderProps (this);
   }

   @Override
   public boolean isSelectable () {
      return true;
   }
}
