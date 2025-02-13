package artisynth.istar.TMJModel.JawTMJ;

import java.awt.Color;
import java.awt.GridBagConstraints;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;

import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSeparator;
import javax.swing.SwingConstants;

import com.github.sardine.model.Set;

import artisynth.core.femmodels.FemCutPlane;
import artisynth.core.femmodels.FemElement;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemFactory;
import artisynth.core.femmodels.FemModel;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.HexElement;
import artisynth.core.femmodels.IntegrationPoint3d;
import artisynth.core.fields.ScalarNodalField;
import artisynth.core.femmodels.FemModel.Ranging;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.gui.ControlPanel;
import artisynth.core.inverse.TrackingController;
import artisynth.core.materials.LinearElasticContact;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.BodyConnector;
import artisynth.core.mechmodels.CollisionBehavior;
import artisynth.core.mechmodels.CollisionManager;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.mechmodels.PlanarConnector;
import artisynth.core.mechmodels.PointAttachable;
import artisynth.core.mechmodels.PointList;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.CollisionBehavior.Method;
import artisynth.core.mechmodels.CollisionManager.ColliderType;
import artisynth.core.mechmodels.MechSystemSolver.PosStabilization;
import artisynth.core.modelbase.ComponentUtils;
import artisynth.core.modelbase.MonitorBase;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.renderables.ColorBar;
import artisynth.core.util.ArtisynthIO;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.util.ScalarRange;
import artisynth.core.workspace.RootModel;
import maspack.geometry.BVFeatureQuery;
import maspack.geometry.Face;
import maspack.geometry.MeshBase;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.AxisAngle;
import maspack.matrix.Matrix3d;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.SymmetricMatrix3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;
import maspack.properties.PropertyMode;
import maspack.render.RenderList;
import maspack.render.RenderProps;
import maspack.render.Renderer.LineStyle;
import maspack.render.color.ColorMapBase;
import maspack.render.color.RainbowColorMap;
import maspack.util.DoubleInterval;
import maspack.util.PathFinder;
import maspack.widgets.IntegerField;


public class JawFemDemoOptimize extends RootModel implements ActionListener {
  
   
   JawModelFEM myJawModel; 
   TrackingController myTrackingController;
   ScalarNodalField integField;
   
   boolean Element_Visibility = false;
   
   boolean myShowDonorStress = false;
   
   boolean cutPlane = false;
   
   String donorMesh = "donor_opt0_remeshed.obj";
   String plateFile = "plate_opt.art";
     
   
   public static double DENSITY_TO_mmKS = 1e-9; // convert density from MKS tp mmKS
   public static double PRESSURE_TO_mmKS = 1e-3; // convert pressure from MKS tp mmKS

   public static double CancellousBoneDensity = 100.0 * DENSITY_TO_mmKS;
   public static double CancellousBoneE = 1.3*1e9 * PRESSURE_TO_mmKS;
   public static double CancellousBoneNu = 0.3;

   public static double myTitaniumDensity = 4420.0 * DENSITY_TO_mmKS;
   public static double myTitaniumE = 100*1e9 * PRESSURE_TO_mmKS;
   public static double myTitaniumNu = 0.3;
   
   public static double corticalBoneDensity = 2000.0 * DENSITY_TO_mmKS;
   public static double corticalBoneYoungModulus =  13.7*1e9 * PRESSURE_TO_mmKS;
   public static double corticalBonePoissonRatio = 0.3;
 
   public static double corticalAppositionDensity = 0.002;
   public static double cancellousAppositionDensity = 0.0015;

   
   // for interaction between the donor check Optimization paper for prosthesis
   double  DEFAULT_E =0.03 * 1e9 * PRESSURE_TO_mmKS;
   double  DEFAULT_Thickness = .2; // mm   
   double  DEFAULT_Damping = 10; 
   double  DEFAULT_Nu = 0.3;
   
   
   double corticalThickness = 2.5;
   
   double t=0.75; 

   protected static double unitConversion = 1000;

 
   
   
   FemModel3d myDonor0;
   FemModel3d myPlate;
 
   
   
   RigidBody myMandibleRight;
   RigidBody myMandibleLeft;
   RigidBody myDonor0Mesh;

   
   PolygonalMesh donorMeshSurface;
   PolygonalMesh surfaceLeft;
   PolygonalMesh surfaceRight;

   
 
   private static Color PALE_BLUE = new Color (0.6f, 0.6f, 1.0f);
   private static Color GOLD = new Color (1f, 0.8f, 0.1f);

   
   String myGeoDir = PathFinder.getSourceRelativePath (
      JawFemDemoOptimize.class, "geometry/");
   
   
   ArrayList<String> MuscleAbbreviation = new ArrayList<String>();

 

   protected String workingDirname = "data/";
   String probesFilename ;

   HashMap<String,String> condyleMusclesLeft = new HashMap<String,String>();
   HashMap<String,String> condyleMusclesRight = new HashMap<String,String>();

   HashMap<String,String> ramusMusclesLeft = new HashMap<String,String>();
   HashMap<String,String> ramusMusclesRight = new HashMap<String,String>();

   HashMap<String,String> bodyMusclesLeft = new HashMap<String,String>();
   HashMap<String,String> bodyMusclesRight = new HashMap<String,String>();

   HashMap<String,String> hemisymphysisMusclesLeft = new HashMap<String,String>();
   HashMap<String,String> hemisymphysisMusclesRight = new HashMap<String,String>();
   
   
   HashSet<FemNode3d> nodesOnSurfaceLeft =   new HashSet<FemNode3d>();
   HashSet<FemElement3d> elemsNearSurfaceLeft =  new HashSet<FemElement3d>();
   
   HashSet<FemNode3d> nodesOnSurfaceRight =   new HashSet<FemNode3d>();
   HashSet<FemElement3d> elemsNearSurfaceRight =  new HashSet<FemElement3d>();
   HashSet<FemElement3d> elementsCloseToSurface = new HashSet<FemElement3d>();
   
   JFrame frame;
   JPanel panel; 
   JSeparator seperator1;
   JCheckBox cb1,cb2,cb3,cb4,cb5,cb6,cb7,cb8;      
   GridBagConstraints gc;
   JLabel label;
   JButton button;
   
   
   public static PropertyList myProps = new PropertyList (JawFemDemoOptimize.class, RootModel.class);
  
   
   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   
   static {
      myProps.addReadOnly ("maxNodalStress", "Max Nodal Stress");
      myProps.addReadOnly ("safetyLeft", "Safety Factor Left");
      myProps.addReadOnly ("safetyRight", "Safety Factor Right");
      myProps.addReadOnly ("percMechanicalStimLeftBuiltin", "Max Mechanical Stimulus");
      myProps.addReadOnly ("percMechanicalStimRightBuiltin", "Max Mechanical Stimulus");

    }
   

 public double getMaxNodalStress() {
      
      return computeMaxNodalStress();
     
   }
   

 
 public double getPercMechanicalStimLeftBuiltin() {
    
    return computePercStressStrainDonor0LeftBuiltin();
   
 }

 
 public double getPercMechanicalStimRightBuiltin() {
    
    return computePercStressStrainDonor0RightBuiltin();
   
 }
 
 public double getSafetyLeft() {
    
    return computeSafetyLeft ();
 }
 
 
 public double getSafetyRight() {
    
    return computeSafetyRight ();
 }
 
 

    /*
   @Override
   public void prerender ( RenderList list ) {
     super.prerender (list );
     ColorBar cbar = ( ColorBar)( renderables ().get("colorBar"));
     cbar.setColorMap (integField. getColorMap ());
     cbar.updateLabels (integField.getValueRange ().getLowerBound (), integField.getValueRange().getUpperBound());
   }
   */


   public JawFemDemoOptimize () {
      
   }

   
   public JawFemDemoOptimize (String name){
      super(null);
   }
   
   
   @Override
   public void build (String[] args) throws IOException {
      super.build (args);
      
 
     
      setWorkingDir();
      myJawModel = new JawModelFEM("jawmodel");
      addModel (myJawModel);
      getRoot (this).setMaxStepSize (0.001);
      
    
      myJawModel.setStabilization (
         PosStabilization.GlobalStiffness); // more accurate stabilization
      
      
 
      //addClosingForce ();
      addOpening();
      
      addFemDonorPlate();
            

      myDonor0.setComputeNodalStress (true);
      myDonor0.setComputeNodalStrain (true);
      myDonor0.setComputeStrainEnergy (true);
      myDonor0.setComputeNodalEnergyDensity (true);

   
      if (myShowDonorStress) {
         // set donor FEM models to display stress on their surfaces
         myDonor0.setSurfaceRendering (SurfaceRender.EnergyDensity);
         myDonor0.setStressPlotRanging (Ranging.Fixed);
         myDonor0.setStressPlotRange (new DoubleInterval(0, 0.08));
        
         // allow stress ranges to be controlled in the control panel
         ControlPanel panel1 = new ControlPanel("options");

         panel1.addWidget ("stressRanging0", myDonor0, "stressPlotRanging");
         panel1.addWidget ("stressRange0", myDonor0, "stressPlotRange");
         
      }
      
      
      /*
      integField = new ScalarNodalField (myDonor0);
      myDonor0.addField(integField);
       */

         /*
        for (FemNode3d node :myDonor0.getNodes ()) {
            
            SymmetricMatrix3d stressTensor = node.getStress();
            SymmetricMatrix3d strainTensor = node.getStrain();

            double dotProduct = 0;

            for (int row = 0; row < 3; row++) {
                for (int col = 0; col < 3; col++) {
                    dotProduct += stressTensor.get(row, col) * strainTensor.get(row, col);
                }
            } 

            integField.setValue (node, node.getVonMisesStress ());
                      
         }
         */
      

     if (cutPlane == true) {
        
     
         FemCutPlane cutplane = new FemCutPlane (
         new RigidTransform3d (-32.7614, -69.3082, -98.8487, 0, 0 ,Math . toRadians (90) ));
         
         myDonor0. addCutPlane (cutplane);
             
         //integField.setVisualization (ScalarNodalField.Visualization.SURFACE);
         //integField.addRenderMeshComp (cutplane);
         
         cutplane. setSurfaceRendering (SurfaceRender.EnergyDensity);
         cutplane.setStressPlotRanging (Ranging.Fixed);
         cutplane.setStressPlotRange (new DoubleInterval(0, .0792));
         cutplane. setAxisLength (25);
         
         //integField.setRenderRange (new ScalarRange (ScalarRange.Updating.FIXED));
         //RenderProps.setLineWidth (cutplane , 2);
         //integField.setRenderRange (new ScalarRange (0, .04));    
         //RainbowColorMap rainbowMap = new RainbowColorMap();
         //field.setColorMap(hueColorMap);
         
         
         
         ColorBar cbar = new ColorBar ();
         cbar. setName("colorBar");
         cbar. setNumberFormat ("%.3f"); // 2 decimal places
         cbar. populateLabels (0.0 , .0792 , 10); // Start with range [0,1], 10 ticks
         cbar. setLocation (-100, 0.1, 20, 0.8) ;
         addRenderable (cbar);
         cbar. setColorMap (cutplane. getColorMap ());
        
             
         
         /*
         ControlPanel panel = new ControlPanel ();
         panel.addWidget (integField , " visualization ");
         panel.addWidget (integField , " renderRange ");
         panel.addWidget (integField , " colorMap ");
         addControlPanel ( panel);
         */
        
         /*
         addMonitor(new StressUpdateMonitor(myDonor0, integField));
         */
         
     }
      
      for (double i=0.01; i<=2*t; i=i+0.01 ){
         addWayPoint (i);
      }
      addBreakPoint (t);    
      
      loadProbes("probe.art");
     
      
      loadBoluses();
            
     

      
      condyleMusclesLeft.put("lip","Left Inferior Lateral Pterygoid");
      condyleMusclesLeft.put("lsp","Left Superior Lateral Pterygoid");
      
      condyleMusclesRight.put("rip","Right Inferior Lateral Pterygoid");
      condyleMusclesRight.put("rsp","Right Superior Lateral Pterygoid");


      ramusMusclesLeft.put("lpt", "Left Posterior Temporal");
      ramusMusclesLeft.put("lmt", "Left Middle Temporal");
      ramusMusclesLeft.put("lat", "Left Anterior Temporal");
      ramusMusclesLeft.put("ldm", "Left Deep Masseter");
      ramusMusclesLeft.put("lsm", "Left Superficial Masseter");
      ramusMusclesLeft.put("lmp", "Left Medial Pterygoid");
      
      ramusMusclesRight.put("rpt", "Right Posterior Temporal");
      ramusMusclesRight.put("rmt", "Right Middle Temporal");
      ramusMusclesRight.put("rat", "Right Anterior Temporal");
      ramusMusclesRight.put("rdm", "Right Deep Masseter");
      ramusMusclesRight.put("rsm", "Right Superficial Masseter");
      ramusMusclesRight.put("rmp", "Right Medial Pterygoid");
      
      
      bodyMusclesLeft.put("lpm","Left Posterior Mylohyoid");
      bodyMusclesLeft.put("lam","Left Mylohyoid");

      bodyMusclesRight.put("ram","Right Mylohyoid");
      bodyMusclesRight.put("rpm","Right Posterior Mylohyoid");


      hemisymphysisMusclesLeft.put("lad", "Left Anterior Digastric" );
      hemisymphysisMusclesLeft.put("lgh", "Left Geniohyoid" );        
                      
      hemisymphysisMusclesRight.put("rad", "Right Anterior Digastric" );
      hemisymphysisMusclesRight.put("rgh", "Right Geniohyoid" );      

      
      frame = new JFrame();
      panel = new JPanel();
      gc = new GridBagConstraints();
  
      gc.anchor = GridBagConstraints.EAST;
      gc.fill = GridBagConstraints.NONE;
  
      seperator1 = new JSeparator();
  
      cb1 = new JCheckBox("Left Condyle Defect (Left C)");
      cb2 = new JCheckBox("Right Condyle Defect (Right C)");

      cb3 = new JCheckBox("Left Ramus Defect (Left R)");
      cb4 = new JCheckBox("Right Ramus Defect (Right R)");
  
      cb5 = new JCheckBox("Left Body Defect (Left B)");
      cb6 = new JCheckBox("Right Body Defect (Right B)");
  
      cb7 = new JCheckBox("Left HemiSymphysis Defect (Left SH)");
      cb8 = new JCheckBox("Right HemiSymphysis Defect (Right SH)");
  
      button = new JButton("Initialize/Reset");
  
  
      cb1.addActionListener(this);
      cb2.addActionListener(this);
      cb3.addActionListener(this);
      cb4.addActionListener(this);
      cb5.addActionListener(this);
      cb6.addActionListener(this);
      cb7.addActionListener(this);
      cb8.addActionListener(this);

      button.addActionListener(this);
  
      gc.gridx = 0;
      gc.gridy = 1;
      panel.add(cb1,gc);
  
      gc.gridx = 0;
      gc.gridy = 2;
      panel.add(cb2,gc);
  
      gc.gridx = 0;
      gc.gridy = 3;
      panel.add(cb3,gc);
  
      gc.gridx = 0;
      gc.gridy = 4;
      panel.add(cb4,gc);
  
      gc.gridx = 0;
      gc.gridy = 5;
      panel.add(cb5,gc);
  
      gc.gridx = 0;
      gc.gridy = 6;
      panel.add(cb6,gc);
  
      gc.gridx = 0;
      gc.gridy = 7;
      panel.add(cb7,gc);
  
      gc.gridx = 0;
      gc.gridy = 8;
      panel.add(cb8,gc);
  
      seperator1.setOrientation(SwingConstants.HORIZONTAL);
      gc.gridx = 0;
      gc.gridy = 9;
      panel.add(seperator1,gc);
  
      gc.gridx = 0;
      gc.gridy = 10;
      panel.add(button,gc);
  
     panel.setLayout(new GridLayout(0,1));

     frame.setTitle("Urken's Defect Classification (Forward)");
     frame.setSize(330, 500);

     frame.add(panel);
     frame.setVisible(false);
     
     
     
     
     
     surfaceLeft = myMandibleLeft.getSurfaceMesh();

     //Finding Surface Nodes and elements 
     
     for ( FemNode3d node : myDonor0. getNodes ()) {
      
        double d = surfaceLeft. distanceToPoint (node. getPosition ());

         if (d < .1) {
            nodesOnSurfaceLeft.add (node);  
            RenderProps.setPointColor (node, Color.PINK);
         }
  
     }
     
     
     for (FemElement3d element : myDonor0.getElements()) {
         for (FemNode3d node : element.getNodes()) {
            if (nodesOnSurfaceLeft.contains ((FemNode3d)node)) {
                elemsNearSurfaceLeft.add (element);
                RenderProps.setLineColor (element, Color.green);
                RenderProps.setVisible (element, Element_Visibility);
                
            }
              
         }
     }
     
     
     
     surfaceRight = myMandibleRight.getSurfaceMesh();

     
     for ( FemNode3d node : myDonor0. getNodes ()) {
      
     double d = surfaceRight. distanceToPoint (node. getPosition ());

         if (d < .1) {
            nodesOnSurfaceRight.add (node);  
            RenderProps.setPointColor (node, Color.PINK);
         }
  
     }
     
     
     for (FemElement3d element : myDonor0.getElements()) {
         for (FemNode3d node : element.getNodes()) {
            if (nodesOnSurfaceRight.contains ((FemNode3d)node)) {
                elemsNearSurfaceRight.add (element);
                RenderProps.setLineColor (element, Color.MAGENTA);
                RenderProps.setVisible (element, Element_Visibility);

            }
              
         }
     }

     
     
     
    myDonor0Mesh = (RigidBody)myJawModel.findComponent (
        "rigidBodies/donor_mesh0");
    RenderProps.setVisible (myDonor0Mesh, false);

   
    //Get the vertices from the surface mesh of the donor mesh rigid body
    donorMeshSurface = myDonor0Mesh.getSurfaceMesh();
    double distanceThreshold = corticalThickness;  // Set your distance threshold
   
   //Set to store elements close to the surface
   elementsCloseToSurface = new HashSet<>();
   
   //Iterate over all elements in the FEM model
   for (FemElement3d element : myDonor0.getElements()) {
     // Compute the centroid of the element
     Point3d centroid = new Point3d();
     element.computeCentroid(centroid);
   
     // Initialize a variable to track the minimum distance
     double minDistance = Double.MAX_VALUE;
   
     // Iterate over all vertices in the donor mesh surface
     for (Vertex3d vertex : donorMeshSurface.getVertices()) {
         Point3d vertexPosition = vertex.getWorldPoint(); // Get the world position of the vertex
   double distance = vertexPosition.distance(centroid); // Calculate distance to the centroid
   
   // Keep track of the minimum distance found
         if (distance < minDistance) {
             minDistance = distance;
         }
     }
   
     // If the closest distance is within the threshold, add the element and color it
     if (minDistance < distanceThreshold) {
         elementsCloseToSurface.add(element);
         RenderProps.setLineColor(element, Color.YELLOW);  // Color the element green
         RenderProps.setVisible (element, Element_Visibility);

         LinearMaterial corticalBoneMaterial = new LinearMaterial(corticalBoneYoungModulus, corticalBonePoissonRatio);
         element.setMaterial(corticalBoneMaterial);
         element.setDensity(corticalBoneDensity);
     }
     else {
        LinearMaterial corticalBoneMaterial = new LinearMaterial(CancellousBoneE, CancellousBoneNu);
        element.setMaterial(corticalBoneMaterial);
        element.setDensity(CancellousBoneDensity);
     }
   }
   
   //Debug: print the number of close elements found
   System.out.println("Number of elements close to the surface: " + elementsCloseToSurface.size());
   
 
  }
   
   
   
   
   
   
   
   public double computeSafetyLeft() {

      // Define yield strengths for cortical and cancellous bone
      double corticalYieldStrength = 100.0; // Lower bound for cortical bone yield strength in MPa
      double cancellousYieldStrength = 5.0;  // Lower bound for cancellous bone yield strength in MPa

      // Variables to track the maximum MAP stress for cortical and cancellous regions
      double maxCorticalStress = 0.0001;
      double maxCancellousStress = 0.0001;

      // Iterate through the elements near the surface on the left
      for (FemElement3d elem : elemsNearSurfaceLeft) {
          for (FemNode3d node : elem.getNodes()) {
              double nodeStress = Math.abs(node.getMAPStress()) / 1000; // Convert to MPa if necessary

              // Check if the element is part of elementsCloseToSurface (cortical)
              if (elementsCloseToSurface.contains(elem)) {
                  // Update maximum cortical stress if this is higher
                  if (nodeStress > maxCorticalStress) {
                      maxCorticalStress = nodeStress;
                  }
              } else {
                  // Update maximum cancellous stress if this is higher
                  if (nodeStress > maxCancellousStress) {
                      maxCancellousStress = nodeStress;
                  }
              }
          }
      }

      // Calculate the safety factors
      double safetyFactorCortical = corticalYieldStrength / maxCorticalStress;
      double safetyFactorCancellous = cancellousYieldStrength / maxCancellousStress;

      // Return the minimum safety factor
      double minSafetyFactor = Math.min(safetyFactorCortical, safetyFactorCancellous);

      //System.out.println("Left Max Cortical Stress = " + maxCorticalStress + " MPa, Left Safety Factor = " + safetyFactorCortical);
      //System.out.println("Left Max Cancellous Stress = " + maxCancellousStress + " MPa, Left Safety Factor = " + safetyFactorCancellous);
      //System.out.println("Left Minimum Safety Factor = " + minSafetyFactor);

      return minSafetyFactor;
  }

   
   
   
   
   public double computeSafetyRight() {

      // Define yield strengths for cortical and cancellous bone
      double corticalYieldStrength = 100.0; // Lower bound for cortical bone yield strength in MPa
      double cancellousYieldStrength = 5.0;  // Lower bound for cancellous bone yield strength in MPa

      // Variables to track the maximum MAP stress for cortical and cancellous regions
      double maxCorticalStress = 0.0001;
      double maxCancellousStress = 0.0001;

      // Iterate through the elements near the surface on the right
      for (FemElement3d elem : elemsNearSurfaceRight) {
          for (FemNode3d node : elem.getNodes()) {
              double nodeStress = Math.abs(node.getMAPStress()) / 1000; // Convert to MPa if necessary

              // Check if the element is part of elementsCloseToSurface (cortical)
              if (elementsCloseToSurface.contains(elem)) {
                  // Update maximum cortical stress if this is higher
                  if (nodeStress > maxCorticalStress) {
                      maxCorticalStress = nodeStress;
                  }
              } else {
                  // Update maximum cancellous stress if this is higher
                  if (nodeStress > maxCancellousStress) {
                      maxCancellousStress = nodeStress;
                  }
              }
          }
      }

      // Calculate the safety factors
      double safetyFactorCortical = corticalYieldStrength / maxCorticalStress;
      double safetyFactorCancellous = cancellousYieldStrength / maxCancellousStress;

      // Return the minimum safety factor
      double minSafetyFactor = Math.min(safetyFactorCortical, safetyFactorCancellous);

      //System.out.println("Right Max Cortical Stress = " + maxCorticalStress + " MPa, Right Safety Factor = " + safetyFactorCortical);
      //System.out.println("Right Max Cancellous Stress = " + maxCancellousStress + " MPa, Right Safety Factor = " + safetyFactorCancellous);
      //System.out.println("Right Minimum Safety Factor = " + minSafetyFactor);

      return minSafetyFactor;
  }

   
   
   public double computeMaxNodalStress() {
      
   // Assuming 'femModel' is your FEM model
      FemModel3d femModel = myDonor0 ; // Initialize your FEM model here

      
   // Get the list of all nodes in the model
      PointList<FemNode3d> nodePointList = femModel.getNodes();

      // Create a list to store stress values of nodes
      List<Double> stressValues = new ArrayList<>();

      // Map to store nodes with their respective stress values
      Map<FemNode3d, Double> nodeStressMap = new HashMap<>();

      // Loop through nodes and get stress values
      for (FemNode3d node : nodePointList) {
          // Assuming getStress() returns the stress value for the node
          double stress = node.getMAPStress (); // Replace with actual method to get node stress
          stressValues.add(stress);
          nodeStressMap.put(node, stress);
      }

      // Sort stress values in descending order
      Collections.sort(stressValues, Collections.reverseOrder());

      // Calculate the number of top 1% nodes
      int topNodesCount = (int) Math.ceil(0.01 * stressValues.size());

      // Get the stress value threshold for the top 1% nodes
      double thresholdStress = stressValues.get(topNodesCount - 1);

      // Calculate the sum and count of top 1% stress values
      double sumStress = 0.0;
      int count = 0;

      for (Map.Entry<FemNode3d, Double> entry : nodeStressMap.entrySet()) {
          if (entry.getValue() >= thresholdStress) {
              sumStress += entry.getValue();
              count++;
          }
      }

      // Calculate the average stress for the top 1% nodes
      double averageStress = sumStress / count;
      
      return averageStress;
      
      
      
      

   }

   
   
   
   
   public double computePercStressStrainDonor0LeftBuiltin() {
      // Enable necessary computations
      myDonor0.setComputeNodalStress(true);
      myDonor0.setComputeNodalStrain(true);
      myDonor0.setComputeStrainEnergy(true);
      myDonor0.setComputeNodalEnergyDensity(true);

      double appCounter = 0;

      // Define densities for cortical and cancellous bone


      // Strain Energy Density and Mechanical Stimulus Computations
      for (FemElement3d element : elemsNearSurfaceLeft) {
          double elementStrainEnergyDensity = element.getStrainEnergy() / element.getVolume();

          boolean isNearSurface = elementsCloseToSurface.contains(element);
          double density = isNearSurface ? corticalAppositionDensity : cancellousAppositionDensity;

          if (elementStrainEnergyDensity / (1000 * density) > 0.0396) {
              appCounter = appCounter + 1;
            //  RenderProps.setLineColor (element, Color.red);
            //  RenderProps.setVisible (element, true);
              
          }
          
          else {
             
            if (isNearSurface) {
            // RenderProps.setLineColor (element, Color.blue);
            }
            else {
             //  RenderProps.setLineColor (element, Color.green);
            }
            }
          
      }

      System.out.println("left elemnt number in appose = " + appCounter );
      System.out.println("left elemnt number total suface = " + elemsNearSurfaceLeft.size() );

      return appCounter / elemsNearSurfaceLeft.size();
  }

   
   
   
   
   public double computePercStressStrainDonor0RightBuiltin() {
      // Enable necessary computations
      myDonor0.setComputeNodalStress(true);
      myDonor0.setComputeNodalStrain(true);
      myDonor0.setComputeStrainEnergy(true);
      myDonor0.setComputeNodalEnergyDensity(true);

      double appCounter = 0;

      // Strain Energy Density and Mechanical Stimulus Computations
      for (FemElement3d element : elemsNearSurfaceRight) {
          double elementStrainEnergyDensity = element.getStrainEnergy() / element.getVolume();

          boolean isNearSurface = elementsCloseToSurface.contains(element);
          double density = isNearSurface ? corticalAppositionDensity : cancellousAppositionDensity;

          if (elementStrainEnergyDensity / (1000 * density) > 0.0396) {
              appCounter = appCounter + 1;
          }
      }


      System.out.println("right elemnt number in appose = " + appCounter );
      System.out.println("right elemnt number total suface = " + elemsNearSurfaceRight.size() );
      
      return appCounter / elemsNearSurfaceRight.size();
  }

   

   

   public FemModel3d createFemModel (
      MechModel mech, String name, String meshName) {

      FemModel3d fem = new FemModel3d (name);

      
      PolygonalMesh surface = loadMesh (meshName);
      FemFactory.createFromMesh (fem, surface, /*tetgen quality=*/1.5);

      fem.setMassDamping (1.0);
      fem.setStiffnessDamping (0);

      
      RenderProps.setSphericalPoints (fem, 0.2, Color.BLUE);

      mech.addModel (fem);
      return fem;
   }

   
   

   private PolygonalMesh loadMesh (String meshName) {
      PolygonalMesh mesh = null;
      String meshPath = myGeoDir + meshName;
      try {
         mesh = new PolygonalMesh (meshPath);
      }
      catch (IOException e) {
         System.out.println ("Can't open or load "+meshPath);
      }
      mesh.transform (myJawModel.amiraTranformation);
      return mesh;
   }


   
   public void attachFemToBody (
      MechModel mech, FemModel3d fem, PointAttachable body, int[] nodeNums) {

      for (int num : nodeNums) {
         mech.attachPoint (fem.getNodeByNumber(num), body);
      }
   }

  
   
   public void attachFemToBody (
      MechModel mech, FemModel3d fem, PointAttachable body, double dist) {
      
      PolygonalMesh surface = null;
      if (body instanceof RigidBody) {
         surface = ((RigidBody)body).getSurfaceMesh();
      }
      else if (body instanceof FemModel3d) {
         surface = ((FemModel3d)body).getSurfaceMesh();
      }
      else {
         throw new IllegalArgumentException (
            "body is neither a rigid body nor an FEM model");
      }
      for (FemNode3d n : fem.getNodes()) {
         if (fem.isSurfaceNode (n)) {
            double d = surface.distanceToPoint (n.getPosition());
            if (d < dist) {
               mech.attachPoint (n, body);
               // set the attached points to render as red spheres
               RenderProps.setSphericalPoints (n, 0.5, Color.RED);
            }
         }
      }
   }

   
   
   
   public void addFemDonorPlate() {
     
      
      myDonor0 = createFemModel (
         myJawModel, "donor0", donorMesh);
            
      String platePath = myGeoDir + plateFile;
   
      try {
         // read the FEM using the loadComponent utility
         myPlate = ComponentUtils.loadComponent (
            platePath, null, FemModel3d.class);
         // set the material properties to correspond to titanium 
         myPlate.setName ("plate");
         myPlate.setDensity (myTitaniumDensity);
         myPlate.setMaterial (new LinearMaterial (myTitaniumE, myTitaniumNu));
         myPlate.setMassDamping (10.0);
         myPlate.setStiffnessDamping (0.0);
         
         // set render properties for the plate
         RenderProps.setFaceColor (myPlate, GOLD);
         RenderProps.setPointRadius (myPlate, 0.5);         
         Vector3d translation = new Vector3d(7.0, -40.957,  -53.909);
         
         myPlate.transformPose (new RigidTransform3d (translation, AxisAngle.IDENTITY));
         myPlate.transformPose (myJawModel.amiraTranformation);
      }
      catch (IOException e) {
         System.out.println ("Can't open or load "+platePath);
         e.printStackTrace(); 
      }
      myJawModel.addModel (myPlate);
      
           
      myMandibleRight = (RigidBody)myJawModel.findComponent (
      "rigidBodies/jaw_resected");
      
      myMandibleLeft = (RigidBody)myJawModel.findComponent (
      "rigidBodies/jaw");

   
      
      
      int[] leftAttachNodes = {0,1,4,5};
      
      attachFemToBody (myJawModel, myPlate, myMandibleLeft, leftAttachNodes);
      
      
      int numNodes = myPlate.numNodes (); 
      
      int[] rightAttachNodes = {numNodes-3, numNodes-4, numNodes-7, numNodes-8  };
     
      attachFemToBody (myJawModel, myPlate, myMandibleRight, rightAttachNodes);

      
      
      attachPlateToDonorSegments (myJawModel);

      
      setDonorSegmentInteractions (myJawModel);
 
   }
   

   private void attachPlateToDonorSegments (MechModel mech) {
         // attach plate to donor segments using rigid bodies representing screws

        double attachTol = 0.05;
        //int hexElem =9;
        
        
        RigidBody screw0 = (RigidBody)myJawModel.findComponent ("rigidBodies/screw0");

        
        int  hexElem0  = findClosestHexElementNumber(myPlate,screw0);

       
         attachElemToSegmentforRigid (
             mech, screw0,  (HexElement)myPlate.getElementByNumber(hexElem0),
             myDonor0, attachTol);
         
         
         screw0.setDensity (myTitaniumDensity);      
        
   }
   
   

   public  int findClosestHexElementNumber( FemModel3d myPlate, RigidBody screw) {
      double minDistance = Double.MAX_VALUE;
      HexElement closestElement = null;
      Point3d screwPosition = screw.getPosition();  // Get the screw's position directly

      // Iterate through all element numbers in the plate
      for (int i = 0; i < myPlate.numElements(); i++) {
          HexElement elem = (HexElement) myPlate.getElement(i);

          // Calculate centroid of the element
          Point3d centroid = new Point3d();
          elem.computeCentroid(centroid);
          
          // Calculate distance to the screw
          double distance = centroid.distance(screwPosition);
          
          // Update the closest element if necessary
          if (distance < minDistance) {
              minDistance = distance;
              closestElement = elem;
          }
      }

      // If a closest element is found, return its number
      if (closestElement != null) {
          return closestElement.getNumber();
      } else {
          // Return -1 or any other indicator if no element is found
          return -1;
      }
  }


   

   private void setDonorSegmentInteractions (MechModel mech) {
      
     
      CollisionBehavior behav7 = new CollisionBehavior (true, 0);
      CollisionBehavior behav8 = new CollisionBehavior (true, 0);

      
      behav7.setMethod(Method.VERTEX_PENETRATION);
      behav8.setMethod(Method.VERTEX_PENETRATION);

     
      LinearElasticContact EFContact = new LinearElasticContact (DEFAULT_E, DEFAULT_Nu, DEFAULT_Damping, DEFAULT_Thickness);
      behav7.setForceBehavior (EFContact);
      behav8.setForceBehavior (EFContact);


      CollisionManager cm = myJawModel.getCollisionManager();
      cm.setColliderType (ColliderType.AJL_CONTOUR);

      
      
      myJawModel.setCollisionBehavior (myJawModel.rigidBodies().get ("jaw_resected"), (FemModel3d) myJawModel.models().get ("donor0"), behav7);
      myJawModel.setCollisionBehavior (myJawModel.rigidBodies().get ("jaw"), (FemModel3d) myJawModel.models().get ("donor0"), behav8);

;
      behav7.setName ("donor_mandible1");
      behav8.setName ("donor_mandible2");

     
   }

   
 
   private void attachElemToSegmentforRigid (
      MechModel mech, RigidBody screw, HexElement hex, FemModel3d donorFem, double attachTol) {

      PolygonalMesh smesh = screw.getSurfaceMesh();
      int nattach = 0;
      for (FemNode3d n : donorFem.getNodes()) {
         if (smesh.distanceToPoint (n.getPosition()) <= attachTol) {
            mech.attachPoint (n, screw);
            nattach++;
         }
      }
      System.out.println ("screw attached attached with" + nattach + " points");
      // also attach the screw to the hex element
      mech.attachFrame (screw, hex);
   }

   
   ///////// FOOD BOLUS
   
   public boolean bolusesLoaded = false;

   ArrayList<FoodBolus> myFoodBoluses = new ArrayList<FoodBolus>();

   protected double bolusDiameter = 8; // mm

   protected double bolusMaxResistance = 110; // N

   protected double bolusStiffness = bolusMaxResistance / (bolusDiameter);

   protected double bolusDamping = 0.01;

   
   public void loadBoluses() {
      if (bolusesLoaded) return;
      createBoluses();
      for (FoodBolus fb : myFoodBoluses) {
         myJawModel.addForceEffector(fb);
         // System.out.println(fb.getName() + " P = "
         // + fb.getPlane().toString("%8.2f"));
         if (fb.getName().equals("leftbolus")) fb.setActive(true);
         else
            fb.setActive(false);
      }
      bolusesLoaded = true;
      PlanarConnector LBITE = (PlanarConnector) myJawModel.bodyConnectors().get("LBITE");
      LBITE.setEnabled (true);
      RenderProps.setVisible (LBITE, true);
   }
   
   

   public void createBoluses() {
      // TODO: create bolus using occlusal plane angle
      //Point3d rightbitePos = myJawModel.frameMarkers().get("rbite").getLocation ();
      Point3d leftbitePos = myJawModel.frameMarkers().get("lbite")
            .getLocation();
      //createFoodBolus("rightbolus", rightbitePos, (PlanarConnector) myJawModel.bodyConnectors().get("RBITE"));
      createFoodBolus("leftbolus", leftbitePos, (PlanarConnector) myJawModel
            .bodyConnectors().get("LBITE"));
      updateBoluses();
   }
   
   public void updateBoluses() {
      System.out.println("bolus dirs updated");
      if (myFoodBoluses.size() >= 2) {
         //updateBolusDirection("RBITE", myFoodBoluses.get(0));
         updateBolusDirection("LBITE", myFoodBoluses.get(1));
      }
   }
   
   
   
   public void updateBolusDirection(String constraintName, FoodBolus bolus) {
      PlanarConnector bite = (PlanarConnector) myJawModel.bodyConnectors()
            .get(constraintName);
      if (bite != null && bolus != null) {
         bolus.setPlane(bite);
         // RigidTransform3d XPB = bite.getXDB();
         // // System.out.println(constraintName + " X =\n" +
         // XPB.toString("%8.2f"));
         // bolus.setPlane( getPlaneFromX (XPB));
         // // System.out.println(bolus.getName() + "plane =\n" +
         // bolus.myPlane.toString("%8.2f"));
      }
   }
   
   
   public void createFoodBolus(String bolusName, Point3d location,
      PlanarConnector plane) {
   FoodBolus fb = new FoodBolus(bolusName, plane, bolusDiameter,
         bolusMaxResistance, bolusDamping);

   RenderProps bolusPtProps = new RenderProps(myJawModel.getRenderProps());
   bolusPtProps.setPointRadius(0.0);
   bolusPtProps.setPointColor(Color.BLACK);

   RigidBody jaw = myJawModel.rigidBodies().get("jaw");
   FrameMarker bolusContactPt = new FrameMarker();
   myJawModel.addFrameMarker(bolusContactPt, jaw, location);
   bolusContactPt.setName(bolusName + "ContactPoint");
   bolusContactPt.setRenderProps(new RenderProps(bolusPtProps));

   fb.setCollidingPoint(bolusContactPt);
   myFoodBoluses.add(fb);
}

   
   
   @Override
   public void actionPerformed(ActionEvent event) {
                   
           
           checkBoxJob(condyleMusclesLeft, cb1);
           checkBoxJob(condyleMusclesRight, cb2);
           
           checkBoxJob(ramusMusclesLeft, cb3);
           checkBoxJob(ramusMusclesRight, cb4);

           checkBoxJob(bodyMusclesLeft, cb5);
           checkBoxJob(bodyMusclesRight, cb6);
           
           checkBoxJob(hemisymphysisMusclesLeft, cb7);
           checkBoxJob(hemisymphysisMusclesRight, cb8);
           
           
           if (event.getSource() == button) {
                   
                   
                   disableCorrMuscles(bodyMusclesLeft);
                   disableCorrMuscles(bodyMusclesRight);
                   disableCorrMuscles(condyleMusclesLeft);
                   disableCorrMuscles(condyleMusclesRight);
                   disableCorrMuscles(ramusMusclesLeft);
                   disableCorrMuscles(ramusMusclesRight);
                   disableCorrMuscles(hemisymphysisMusclesLeft);
                   disableCorrMuscles(hemisymphysisMusclesRight);
                   
                   
                   enableCorrMuscles(bodyMusclesLeft);
                   enableCorrMuscles(bodyMusclesRight);
                   enableCorrMuscles(condyleMusclesLeft);
                   enableCorrMuscles(condyleMusclesRight);
                   enableCorrMuscles(ramusMusclesLeft);
                   enableCorrMuscles(ramusMusclesRight);
                   enableCorrMuscles(hemisymphysisMusclesLeft);
                   enableCorrMuscles(hemisymphysisMusclesRight);

                   //myJawModel.assembleBilateralExcitors();
                   //myJawModel.assembleMuscleGroups();
                   //loadProbes("adapted11_l.art");
                   cb1.setSelected(false);
                   cb2.setSelected(false);
                   cb3.setSelected(false);
                   cb4.setSelected(false);
                   cb5.setSelected(false);
                   cb6.setSelected(false);
                   cb7.setSelected(false);
                   cb8.setSelected(false);
           }
                          
   }
   
   
   
   
   public void checkBoxJob(HashMap<String,String> corrMuscles, JCheckBox cb) {
           
           if (cb.isSelected()) {
                   disableCorrMuscles(corrMuscles);
                   //loadProbes("adapted11_l.art");
                   System.out.print("-slected");
                  
           } 
                     
   }

   
   
   public void enableCorrMuscles(HashMap<String,String> corrMucle) {
                   
      for (Muscle muscle : myJawModel.myAttachedMuscles){
         muscle.setExcitationColor (Color.RED);
         muscle.setMaxColoredExcitation (1);
         myJawModel.addAxialSpring (muscle);
        
      }
       
   }
   
   
   
   public void disableCorrMuscles(HashMap<String,String> corrMucle) {
           
           for (String name: corrMucle.keySet()) {
                 AxialSpring as = myJawModel.axialSprings().get (name);
                 myJawModel.removeAxialSpring(as);      
           }
           
   }

    

   public void addClosingForce() throws IOException{      
      for (BodyConnector p : myJawModel.bodyConnectors ()){     
         if (p.getName ().equals ("BiteICP")==false){
            p.setEnabled (false);
            p.getRenderProps ().setVisible (false);
         }  
   }
      ((PlanarConnector)myJawModel.bodyConnectors ().get ("BiteICP")).setUnilateral (false);
      MuscleExciter mex=myJawModel.getMuscleExciters ().get ("bi_close");     
      NumericInputProbe probe = new NumericInputProbe (mex, "excitation",ArtisynthPath.getSrcRelativePath (JawModelFEM.class, "/data/input_activation.txt"));
      probe.setStartStopTimes (0, 1);
      probe.setName ("Closing Muscle Activation");
      addInputProbe (probe);
   }
   
   
   
   public void addOpening() throws IOException{
      for (BodyConnector p : myJawModel.bodyConnectors ()){         
            if (p.getName ().equals ("BiteICP")==false){
               p.setEnabled (false);
               p.getRenderProps ().setVisible (false);
            }        
      }           
      MuscleExciter mex=myJawModel.getMuscleExciters ().get ("bi_open");      
      NumericInputProbe probe = new NumericInputProbe (mex, "excitation",ArtisynthPath.getSrcRelativePath (JawModelFEM.class, "/data/input_activation.txt"));
      probe.setStartStopTimes (0, 0.5);
      probe.setName ("Opening Muscle Activation");
      addInputProbe (probe);
   }
      
   public void setWorkingDir() {
      if (workingDirname == null) return;
      // set default working directory to repository location
      File workingDir = new File (
      ArtisynthPath.getSrcRelativePath(JawFemDemoOptimize.class, workingDirname));
      ArtisynthPath.setWorkingDir(workingDir);        
   }
  
   public void loadProbes(String probesFilename) {
      String probeFileFullPath = (ArtisynthPath.getSrcRelativePath(JawModelFEM.class,"data/"+probesFilename));

      System.out.println("Loading Probes from File: " + probeFileFullPath);
       
      try {
          scanProbes(ArtisynthIO.newReaderTokenizer(probeFileFullPath));
       } catch (Exception e) {
          System.out.println("Error reading probe file");
          e.printStackTrace();
       }
    }
   
   
 
}
