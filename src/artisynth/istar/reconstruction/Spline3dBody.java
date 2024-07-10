package artisynth.istar.reconstruction;

import java.io.IOException;
import java.io.PrintWriter;
import java.util.Deque;
import java.util.LinkedList;

import artisynth.core.mechmodels.FrameState;
import artisynth.core.modelbase.CompositeComponent;
import artisynth.core.modelbase.HasCoordinateFrame;
import artisynth.core.modelbase.RenderableComponentBase;
import artisynth.core.util.ScanToken;
import maspack.interpolation.CubicHermiteSpline3d;
import maspack.interpolation.CubicHermiteSpline3d.Knot;
import maspack.matrix.AxisAngle;
import maspack.matrix.Point3d;
import maspack.matrix.Quaternion;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.properties.HasProperties;
import maspack.properties.PropertyList;
import maspack.render.RenderList;
import maspack.render.RenderObject;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.render.Renderer.LineStyle;
import maspack.render.Renderer.Shading;
import maspack.util.IntHolder;
import maspack.util.NumberFormat;
import maspack.util.ReaderTokenizer;

/**
 * Custom component that contains and renders a 3D cubic Hermite spline.
 */
public class Spline3dBody extends RenderableComponentBase
   implements HasCoordinateFrame {
   
   // use a FrameState to store the position even though we ignore velocity
   FrameState myState = new FrameState();
   private CubicHermiteSpline3d mySpline;
   private RigidTransform3d myRenderFrame = new RigidTransform3d();
   private RenderObject myRob;   

   public static int DEFAULT_NUM_RENDER_POINTS = 100;
   private int myNumRenderPoints = DEFAULT_NUM_RENDER_POINTS;

   public static PropertyList myProps =
      new PropertyList (Spline3dBody.class, RenderableComponentBase.class);

   static {
      myProps.get("renderProps").setDefaultValue(defaultRenderProps(null));
      myProps.add ("position", "position of the body coordinate frame",null,"NW");
      myProps.add (
         "orientation", "orientation of the body coordinate frame", null, "NW");
      myProps.add (
         "numRenderPoints",
         "num points used to render the spline", DEFAULT_NUM_RENDER_POINTS); 
   }

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }  

   public Spline3dBody() {
   }

   public Spline3dBody (CubicHermiteSpline3d spline) {
      this();
      setSpline (spline);
   }

   public Spline3dBody (String name, CubicHermiteSpline3d spline) {
      this();
      setName (name);
      setSpline (spline);
   }

   private void setSpline (CubicHermiteSpline3d spline) {
      mySpline = spline;
   }

   public CubicHermiteSpline3d getSpline() {
      return mySpline;
   }
   
   public int getNumRenderPoints() {
      return myNumRenderPoints;
   }

   public void setNumRenderPoints (int num) {
      myNumRenderPoints = num;
   }

   public void setPose (RigidTransform3d XFrameToWorld) {
      myState.setPose (XFrameToWorld);
   }

   public RigidTransform3d getPose() {
      return myState.getPose();
   }

   public void getPose (RigidTransform3d XFrameToWorld) {
      myState.getPose (XFrameToWorld);
   }

   public Point3d getPosition() {
      return new Point3d (myState.getPose().p);
   }

   public void setPosition (Point3d pos) {
      myState.setPosition (pos);
   }

   public AxisAngle getOrientation() {
      return myState.getAxisAngle();
   }

   public void setOrientation (AxisAngle axisAng) {
      RigidTransform3d X = new RigidTransform3d (myState.getPose());
      X.R.setAxisAngle (axisAng);
      setPose (X);
   }

   public Quaternion getRotation() {
      return myState.getRotation();
   }

   public void setRotation (Quaternion q) {
      myState.setRotation (q);
   }
   
   /* --- begin Renderable implementation --- */
   
   protected static RenderProps defaultRenderProps (HasProperties host) {
      RenderProps props = RenderProps.createPointLineProps (host);
      return props;
   }

   public RenderProps createRenderProps() {
      return defaultRenderProps (this);
   }

   public void render (Renderer renderer, int flags) {
      RenderObject rob = myRob;
      if (rob != null) {
         RenderProps props = getRenderProps();
         renderer.pushModelMatrix();
         renderer.mulModelMatrix (myRenderFrame);

         float savedLineWidth = renderer.getLineWidth();
         Shading savedShadeModel = renderer.getShading();
         LineStyle lineStyle = props.getLineStyle();
         Shading savedShading = renderer.getShading();

         if (lineStyle == LineStyle.LINE) {
            renderer.setShading (Shading.NONE);
         }
         else {
            renderer.setShading (props.getShading());
         }
         renderer.setLineColoring (props, isSelected());
         switch (lineStyle) {
            case LINE: {
               int width = props.getLineWidth();
               if (width > 0) {
                  renderer.drawLines (rob, LineStyle.LINE, width);
               }
               break;
            }
            case SPINDLE:
            case SOLID_ARROW:
            case CYLINDER: {
               double rad = props.getLineRadius();
               if (rad > 0) {
                  renderer.drawLines (rob, props.getLineStyle(), rad);
               }
               break;
            }
         }
         renderer.setShading (savedShading);
         renderer.setLineWidth (savedLineWidth);
         renderer.setShading (savedShadeModel);
         
         renderer.popModelMatrix();         
      }
      
      // Knot knot0 = mySpline.getFirstKnot();
      // Knot knotL = mySpline.getLastKnot();

      // double s0 = knot0.getS0();
      // double sL = knotL.getS0();
      // int nump = myNumRenderPoints;

      // renderer.setColor (myRenderProps.getLineColorF(), isSelected());
      // renderer.beginDraw (DrawMode.LINE_STRIP);
      // RigidTransform3d TCW = myState.getPose();
      // for (int i=0; i<nump; i++) {
      //    Point3d pnt = new Point3d(mySpline.eval (s0 + i*(sL-s0)/(nump-1)));
      //    pnt.transform (TCW);
      //    renderer.addVertex (pnt);                  
      // }
      // renderer.endDraw();     
      
   }

   public void prerender (RenderList list) {
      myRob = buildRenderObject (myRenderProps);
      myRenderFrame.set (getPose());
   }

   protected RenderObject buildRenderObject (RenderProps props) {

      RenderObject r = new RenderObject();
      r.createLineGroup();

      double s0 = mySpline.getS0();
      double sL = mySpline.getSLast();
      int nump = myNumRenderPoints;
      int vidx = 0;
      IntHolder lastIdx = new IntHolder();
      for (int i=0; i<nump; i++) {
         Vector3d pos = mySpline.eval (s0 + i*(sL-s0)/(nump-1));
         r.addPosition((float)pos.x, (float)pos.y, (float)pos.z);
         r.addVertex (i);
         if (i > 0) {
            r.addLine (i-1, i);
         }
      }
      return r;
   }  

   @Override
   public void getSelection(LinkedList<Object> list, int qid) {
      list.addLast(this);
   }

   public void updateBounds (Vector3d pmin, Vector3d pmax) {
      for (Knot knot : mySpline) {
         knot.getA0().updateBounds (pmin, pmax);
      }
   }

   // I/O methods

   protected boolean scanItem (ReaderTokenizer rtok, Deque<ScanToken> tokens)
      throws IOException {
      rtok.nextToken();
      if (scanAttributeName (rtok, "spline")) {
         mySpline = new CubicHermiteSpline3d();
         mySpline.scan (rtok, null);
         return true;
      }
      else if (scanAttributeName (rtok, "position")) {
         Point3d pos = new Point3d();
         pos.scan (rtok);
         setPosition (pos);
         return true;
      }
      else if (scanAttributeName (rtok, "rotation")) {
         Quaternion q = new Quaternion();
         q.scan (rtok);
         setRotation (q);
         return true;
      }
      rtok.pushBack();
      return super.scanItem (rtok, tokens);
   }

   protected void writeItems (
      PrintWriter pw, NumberFormat fmt, CompositeComponent ancestor) 
      throws IOException {
      super.writeItems (pw, fmt, ancestor);
      pw.println ("position=[ " + myState.getPosition().toString (fmt) + "]");
      pw.println ("rotation=[ " + myState.getRotation().toString (fmt) + "]");
      if (mySpline != null) {
         pw.print ("spline=");
         mySpline.write (pw, fmt, null);
      }
   }
}
