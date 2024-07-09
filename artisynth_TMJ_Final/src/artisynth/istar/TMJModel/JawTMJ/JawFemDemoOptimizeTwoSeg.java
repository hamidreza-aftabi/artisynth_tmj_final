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

import artisynth.core.femmodels.FemCutPlane;
import artisynth.core.femmodels.FemElement;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemFactory;
import artisynth.core.femmodels.FemModel;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.HexElement;
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
import maspack.geometry.Face;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.AxisAngle;
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


public class JawFemDemoOptimizeTwoSeg extends RootModel implements ActionListener {
  
   
   JawModelFEM myJawModel; 
   TrackingController myTrackingController;
   ScalarNodalField integField;
   
   boolean myShowDonorStress = false;

  
   
   boolean myUseScrews = true;
  
   
   double DENSITY_TO_mmKS = 1e-9; // convert density from MKS tp mmKS
   double PRESSURE_TO_mmKS = 1e-3; // convert pressure from MKS tp mmKS

   double myBoneDensity = 1900.0 * DENSITY_TO_mmKS;
   double myBoneE = 13.7*1e9 * PRESSURE_TO_mmKS;
   double myTitaniumDensity = 4420.0 * DENSITY_TO_mmKS;
   double myTitaniumE = 100*1e9 * PRESSURE_TO_mmKS;
   double myTitaniumNu = 0.3;
   double myBoneNu = 0.3;
   
   double t=0.75; 

   protected static double unitConversion = 1000;

   double  DEFAULT_E =0.03 * 1e9 * PRESSURE_TO_mmKS;
   double  DEFAULT_Thickness = .2; // mm   
   double  DEFAULT_Damping = 10; 
   double  DEFAULT_Nu = 0.3;
   
   
   
   FemModel3d myDonor0;
   FemModel3d myDonor1;
   FemModel3d myPlate;
 
   
   
   RigidBody myMandibleRight;
   RigidBody myMandibleLeft;


   private static Color PALE_BLUE = new Color (0.6f, 0.6f, 1.0f);
   private static Color GOLD = new Color (1f, 0.8f, 0.1f);

   String myGeoDir = PathFinder.getSourceRelativePath (
      JawFemDemoOptimizeTwoSeg.class, "geometry/");
   
   
   ArrayList<String> MuscleAbbreviation = new ArrayList<String>();
   ArrayList<Integer> RightSurfaceElemnets = new ArrayList<Integer>();
   ArrayList<Integer> LeftSurfaceElemnets = new ArrayList<Integer>();

 

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
   
   
   JFrame frame;
   JPanel panel; 
   JSeparator seperator1;
   JCheckBox cb1,cb2,cb3,cb4,cb5,cb6,cb7,cb8;      
   GridBagConstraints gc;
   JLabel label;
   JButton button;
   
   
   protected static PropertyList myProps = new PropertyList (JawFemDemoOptimizeTwoSeg.class, RootModel.class);
  
   
   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   
   static {
      myProps.addReadOnly ("maxNodalStress", "Max Nodal Stress");
      myProps.addReadOnly ("maxMechanicalStimRightBuiltin", "Max Mechanical Stimulus");
      myProps.addReadOnly ("maxMechanicalStimLeftBuiltin", "Max Mechanical Stimulus");
      myProps.addReadOnly ("percMechanicalStimLeftBuiltin", "Max Mechanical Stimulus");
      myProps.addReadOnly ("percMechanicalStimRightBuiltin", "Max Mechanical Stimulus");

    }
   

 public double getMaxNodalStress() {
      
      return computeMaxNodalStress();
     
   }
   
  public double getMaxMechanicalStimRightBuiltin() {
      
      return computeStressStrainDonor0RightBuiltin();
     
   }
   

   
 public double getMaxMechanicalStimLeftBuiltin() {
      
      return computeStressStrainDonor0LeftBuiltin();
     
   }
 
 
 public double getPercMechanicalStimLeftBuiltin() {
    
    return computePercStressStrainDonor0LeftBuiltin();
   
 }

 
 public double getPercMechanicalStimRightBuiltin() {
    
    return computePercStressStrainDonor0RightBuiltin();
   
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


   public JawFemDemoOptimizeTwoSeg () {
      
   }

   
   public JawFemDemoOptimizeTwoSeg (String name){
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
            
      
      //computeStressStrainDonor0Right();
      
      //computeStressStrainDonor0Left();

      myDonor0.setComputeNodalStress (true);
      myDonor0.setComputeNodalStrain (true);
      myDonor0.setComputeStrainEnergy (true);
      myDonor0.setComputeNodalEnergyDensity (true);
      
      
      myDonor1.setComputeNodalStress (true);
      myDonor1.setComputeNodalStrain (true);
      myDonor1.setComputeStrainEnergy (true);
      myDonor1.setComputeNodalEnergyDensity (true);
      
      //myDonor0.setSurfaceRendering (SurfaceRender.EnergyDensity);
      //myDonor0.setStressPlotRanging (Ranging.Fixed);
      //myDonor0.setStressPlotRange (new DoubleInterval(0, .08));
   
      if (myShowDonorStress) {
         // set donor FEM models to display stress on their surfaces
         myDonor0.setSurfaceRendering (SurfaceRender.Stress);
         myDonor0.setStressPlotRanging (Ranging.Fixed);
         myDonor0.setStressPlotRange (new DoubleInterval(0, 200000));
        
         // allow stress ranges to be controlled in the control panel
         ControlPanel panel0 = new ControlPanel("options");

         panel0.addWidget ("stressRanging0", myDonor0, "stressPlotRanging");
         panel0.addWidget ("stressRange0", myDonor0, "stressPlotRange");
         
         
         
         myDonor1.setSurfaceRendering (SurfaceRender.Stress);
         myDonor1.setStressPlotRanging (Ranging.Fixed);
         myDonor1.setStressPlotRange (new DoubleInterval(0, 200000));
        
         // allow stress ranges to be controlled in the control panel
         ControlPanel panel1 = new ControlPanel("options");

         panel1.addWidget ("stressRanging1", myDonor1, "stressPlotRanging");
         panel1.addWidget ("stressRange1", myDonor1, "stressPlotRange");
         
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
      

      FemCutPlane cutplane0 = new FemCutPlane (
      new RigidTransform3d (-32.7614, -69.3082, -98.8487, 0, 0 ,Math . toRadians (90) ));
      
      myDonor0. addCutPlane (cutplane0);
          
      //integField.setVisualization (ScalarNodalField.Visualization.SURFACE);
      //integField.addRenderMeshComp (cutplane);
      cutplane0. setSurfaceRendering (SurfaceRender.EnergyDensity);
      cutplane0.setStressPlotRanging (Ranging.Fixed);
      cutplane0.setStressPlotRange (new DoubleInterval(0, .08));
      cutplane0. setAxisLength (25);
      //integField.setRenderRange (new ScalarRange (ScalarRange.Updating.FIXED));
      //RenderProps.setLineWidth (cutplane , 2);
      //integField.setRenderRange (new ScalarRange (0, .04));    
      //RainbowColorMap rainbowMap = new RainbowColorMap();
      //field.setColorMap(hueColorMap);
      
      
      
      ColorBar cbar = new ColorBar ();
      cbar. setName("colorBar");
      cbar. setNumberFormat ("%.3f"); // 2 decimal places
      cbar. populateLabels (0.0 , .08 , 10); // Start with range [0,1], 10 ticks
      cbar. setLocation (-100, 0.1, 20, 0.8) ;
      addRenderable (cbar);
      cbar. setColorMap (cutplane0. getColorMap ());
     
   
      
      
      
      FemCutPlane cutplane1 = new FemCutPlane (
      new RigidTransform3d (-32.7614, -69.3082, -98.8487, 0, 0 ,Math . toRadians (90) ));
      
      myDonor1. addCutPlane (cutplane1);
          
      //integField.setVisualization (ScalarNodalField.Visualization.SURFACE);
      //integField.addRenderMeshComp (cutplane);
      cutplane1. setSurfaceRendering (SurfaceRender.EnergyDensity);
      cutplane1.setStressPlotRanging (Ranging.Fixed);
      cutplane1.setStressPlotRange (new DoubleInterval(0, .08));
      cutplane1. setAxisLength (25);
      //integField.setRenderRange (new ScalarRange (ScalarRange.Updating.FIXED));
      //RenderProps.setLineWidth (cutplane , 2);
      //integField.setRenderRange (new ScalarRange (0, .04));    
      //RainbowColorMap rainbowMap = new RainbowColorMap();
      //field.setColorMap(hueColorMap);
         
         
    
      
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
      
      for (double i=0.01; i<=2*t; i=i+0.01 ){
         addWayPoint (i);
      }
      addBreakPoint (t);    
      
      loadProbes("probe.art");
     
      //addControlPanel();
      
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
     //frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
     frame.add(panel);
     frame.setVisible(false);
     
     

  
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
      
      HashSet<FemNode3d> nodesOnSurfaceLeft =   new HashSet<FemNode3d>();
      HashSet<FemElement3d> elemsNearSurfaceLeft =  new HashSet<FemElement3d>();
      
      myDonor0.setComputeNodalStress (true);
      myDonor0.setComputeNodalStrain (true);
      myDonor0.setComputeStrainEnergy (true);
      myDonor0.setComputeNodalEnergyDensity (true);
     
      
      
      PolygonalMesh surfaceLeft = myMandibleLeft.getSurfaceMesh();

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
                 RenderProps.setLineColor (element, Color.MAGENTA);
             }
               
          }
      }
      
      double appCounter = 0;
      
      //Strain Energy Density and Mechanical Stimulus Computations
      
      for (FemElement3d element : elemsNearSurfaceLeft) {
        
         double elementStrainEnergyDensity = 0; // Element-specific strain energy density
     
         elementStrainEnergyDensity = element.getStrainEnergy ()/element.getVolume ();
         
         
         if (elementStrainEnergyDensity/(1000 * 0.002) > 0.0396) {
            appCounter = appCounter + 1;
         }

      }

      
      return  appCounter/(elemsNearSurfaceLeft.size ());
      
   }
   
   
   
   
   public double computePercStressStrainDonor0RightBuiltin() {
      
      HashSet<FemNode3d> nodesOnSurfaceRight =   new HashSet<FemNode3d>();
      HashSet<FemElement3d> elemsNearSurfaceRight =  new HashSet<FemElement3d>();
      

      myDonor0.setComputeNodalStress (true);
      myDonor0.setComputeNodalStrain (true);
      myDonor0.setComputeStrainEnergy (true);
      myDonor0.setComputeNodalEnergyDensity (true);
     
      
      
      PolygonalMesh surfaceRight = myMandibleRight.getSurfaceMesh();

      //Finding Surface Nodes and elements 
      
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
             }
               
          }
      }
      
      double appCounter = 0;
      
      //Strain Energy Density and Mechanical Stimulus Computations
      
      for (FemElement3d element : elemsNearSurfaceRight) {
        
         double elementStrainEnergyDensity = 0; // Element-specific strain energy density
     
         elementStrainEnergyDensity = element.getStrainEnergy ()/element.getVolume ();
         
         
         if (elementStrainEnergyDensity/(1000 * 0.002) > 0.0396) {
            appCounter = appCounter + 1;
         }

      }

      
      return  appCounter/(elemsNearSurfaceRight.size ());
      
   }
   
   
   
   
   
   
   
public double computeStressStrainDonor0LeftBuiltin(){
   

   HashSet<FemNode3d> nodesOnSurfaceLeft =   new HashSet<FemNode3d>();
   HashSet<FemElement3d> elemsNearSurfaceLeft =  new HashSet<FemElement3d>();
      
      myDonor0.setComputeNodalStress (true);
      myDonor0.setComputeNodalStrain (true);
      myDonor0.setComputeStrainEnergy (true);
      myDonor0.setComputeNodalEnergyDensity (true);
      
      
      double totalStrainEnergyDensity = 0; // Initialize total strain energy density
      double MaxStrainEnergyDensity =0;
      
      
      PolygonalMesh surfaceLeft = myMandibleLeft.getSurfaceMesh();

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
                 RenderProps.setLineColor (element, Color.MAGENTA);
             }
               
          }
      }
      
      
      //Strain Energy Density and Mechanical Stimulus Computations
      
      for (FemElement3d element : elemsNearSurfaceLeft) {
        
         double elementStrainEnergyDensity = 0; // Element-specific strain energy density
     
         elementStrainEnergyDensity = element.getStrainEnergy ()/element.getVolume ();
         
         totalStrainEnergyDensity += elementStrainEnergyDensity;
         
         if (elementStrainEnergyDensity > MaxStrainEnergyDensity) {
            MaxStrainEnergyDensity = elementStrainEnergyDensity;
         }

      }

      
      return  MaxStrainEnergyDensity/(1000 * 0.002); //Mechanical Stimulus (mj/g): divided by unit Conversion Times the Density for 

      //return  MaxStrainEnergyDensity/(1000); //Strain Energy Density (mj/mm^3): Divided by unit Conversion
      
}




public double computeStressStrainDonor0Left(){
   
   
   HashSet<FemNode3d> nodesOnSurfaceLeft =   new HashSet<FemNode3d>();
   HashSet<FemElement3d> elemsNearSurfaceLeft =  new HashSet<FemElement3d>();
      
      myDonor0.setComputeNodalStress (true);
      myDonor0.setComputeNodalStrain (true);
      
      
      double totalStrainEnergyDensity = 0; // Initialize total strain energy density
      double MaxStrainEnergyDensity =0;
      

      PolygonalMesh surfaceLeft = myMandibleLeft.getSurfaceMesh();

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
                 RenderProps.setLineColor (element, Color.MAGENTA);
             }
               
          }
      }
      
      
      //Strain Energy Density and Mechanical Stimulus Computations
      
      for (FemElement3d element : elemsNearSurfaceLeft) {
        
         double elementStrainEnergyDensity = 0; // Element-specific strain energy density
         

         for (FemNode3d node :element.getNodes ()) {
            
            SymmetricMatrix3d stressTensor = node.getStress();
            SymmetricMatrix3d strainTensor = node.getStrain();

            double dotProduct = 0;

            
            for (int row = 0; row < 3; row++) {
                for (int col = 0; col < 3; col++) {
                    dotProduct += stressTensor.get(row, col) * strainTensor.get(row, col);
                }
            } 
            
            elementStrainEnergyDensity += dotProduct / 2;
          
         }
         
         elementStrainEnergyDensity /= 4;
         
         totalStrainEnergyDensity += elementStrainEnergyDensity;
         
         if (elementStrainEnergyDensity > MaxStrainEnergyDensity) {
            MaxStrainEnergyDensity = elementStrainEnergyDensity;
         }

      }

      
      return  MaxStrainEnergyDensity/(1000 * 0.0002) ; //Mechanical Stimulus (mj/g): divided by unit Conversion Times the Density for 

      //return  MaxStrainEnergyDensity/(1000); //Strain Energy Density (mj/mm^3): Divided by unit Conversion
      
}





public double computeStressStrainDonor0RightBuiltin(){
   
   HashSet<FemNode3d> nodesOnSurfaceRight =   new HashSet<FemNode3d>();
   HashSet<FemElement3d> elemsNearSurfaceRight =  new HashSet<FemElement3d>();

   
   myDonor0.setComputeNodalStress (true);
   myDonor0.setComputeNodalStrain (true);
   myDonor0.setComputeStrainEnergy (true);
   myDonor0.setComputeNodalEnergyDensity (true);
   
   double totalStrainEnergyDensity = 0; // Initialize total strain energy density
   double MaxStrainEnergyDensity =0;
   
   
   PolygonalMesh surfaceRight = myMandibleRight.getSurfaceMesh();

   //Finding Surface Nodes and elements 
   
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
          }
            
       }
   }
   

   
   //Strain Energy Density and Mechanical Stimulus Computations
   

   double elementStrainEnergyDensity = 0; // Element-specific strain energy density

   
   for (FemElement3d element : elemsNearSurfaceRight) {
     
      elementStrainEnergyDensity = element.getStrainEnergy ()/element .getVolume ();
      
      totalStrainEnergyDensity += elementStrainEnergyDensity;
      
      if (elementStrainEnergyDensity > MaxStrainEnergyDensity) {
         MaxStrainEnergyDensity = elementStrainEnergyDensity;
      }

   }
   
   return  MaxStrainEnergyDensity/(1000 * 0.002) ; //Mechanical Stimulus (mj/g): divided by unit Conversion Times the Density for 

   //return  MaxStrainEnergyDensity/(1000); //Strain Energy Density (mJ/mm^3): Divided by unit Conversion
}





   
   public double computeStressStrainDonor0Right(){
      HashSet<FemNode3d> nodesOnSurfaceRight =   new HashSet<FemNode3d>();
      HashSet<FemElement3d> elemsNearSurfaceRight =  new HashSet<FemElement3d>();
      

      
      myDonor0.setComputeNodalStress (true);
      myDonor0.setComputeNodalStrain (true);
           
      double totalStrainEnergyDensity = 0; // Initialize total strain energy density
      double MaxStrainEnergyDensity =0;
      
      
      PolygonalMesh surfaceRight = myMandibleRight.getSurfaceMesh();

      //Finding Surface Nodes and elements 
      
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
             }
               
          }
      }

      
      //Strain Energy Density and Mechanical Stimulus Computations
      
      for (FemElement3d element : elemsNearSurfaceRight) {
        
         double elementStrainEnergyDensity = 0; // Element-specific strain energy density

         for (FemNode3d node :element.getNodes ()) {
            
            SymmetricMatrix3d stressTensor = node.getStress();
            SymmetricMatrix3d strainTensor = node.getStrain();

            double dotProduct = 0;

            
            for (int row = 0; row < 3; row++) {
                for (int col = 0; col < 3; col++) {
                    dotProduct += stressTensor.get(row, col) * strainTensor.get(row, col);
                }
            } 
            
            elementStrainEnergyDensity += dotProduct / 2;
          
         }
         
         elementStrainEnergyDensity /= 4;
         
         totalStrainEnergyDensity += elementStrainEnergyDensity;
         
         if (elementStrainEnergyDensity > MaxStrainEnergyDensity) {
            MaxStrainEnergyDensity = elementStrainEnergyDensity;
         }

      }
      
      return  MaxStrainEnergyDensity/(1000 * 0.002) ; //Mechanical Stimulus (mj/g): divided by unit Conversion Times the Density for 

      //return  MaxStrainEnergyDensity/(1000); //Strain Energy Density (mJ/mm^3): Divided by unit Conversion
   }
   
 
   
   /**
    * Create a FEM model from a triangular surface mesh using Tetgen.
    *
    * @param mech MechModel to add the FEM model to
    * @param name name of the FEM model
    * @param meshName name of the mesh file in the geometry folder
    * @param density of the FEM
    * @param E Young's modulus for the FEM material
    * @param nu Possion's ratio for the FEM material
    */
   public FemModel3d createFemModel (
      MechModel mech, String name, String meshName,
      double density, double E, double nu) {

      // create the fem and set its material properties
      FemModel3d fem = new FemModel3d (name);
      fem.setDensity (density);
      fem.setMaterial (new LinearMaterial (E, nu));
      

      // load the triangular surface mesh and then call createFromMesh,
      // which uses tetgen to create a tetrahedral volumetric mesh:
      PolygonalMesh surface = loadMesh (meshName);
      FemFactory.createFromMesh (fem, surface, /*tetgen quality=*/1.5);

      // damping parameters are important for stabilty
      fem.setMassDamping (1.0);
      fem.setStiffnessDamping (0);

      // enable computation of nodal stresses. Do this so that stresses will be
      // computed even if they are not being rendered.
      //fem.setComputeNodalStress (true);

      // turn on surface rendering and set surface color to light blue
      
      //RenderProps.setFaceColor (fem, PALE_BLUE);
      //fem.setSurfaceRendering (FemModel.SurfaceRender.Shaded);
      RenderProps.setSphericalPoints (fem, 0.2, Color.BLUE);

      mech.addModel (fem);
      return fem;
   }

   

   
   /**
    * Load a polygonal mesh with the given name from the geometry folder.
    */
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

   
   /**
    * Attach an FEM model to another body (either an FEM or a rigid body)
    * by attaching a subset of its nodes to that body.
    *
    * @param mech MechModel containing all the components
    * @param fem FEM model to be connected
    * @param body body to attach the FEM to. Can be a rigid body
    * or another FEM.
    * @param nodeNums numbers of the FEM nodes which should be attached
    * to the body
    */
   public void attachFemToBody (
      MechModel mech, FemModel3d fem, PointAttachable body, int[] nodeNums) {

      for (int num : nodeNums) {
         mech.attachPoint (fem.getNodeByNumber(num), body);
      }
   }

  
   
   /**
    * Attach an FEM model to another body (either an FEM or a rigid body) by
    * attaching all surface nodes that are within a certain distance of the
    * body's surface mesh.
    *
    * @param mech MechModel containing all the components
    * @param fem FEM model to be connected
    * @param body body to attach the FEM to. Can be a rigid body
    * or another FEM.
    * @param dist distance to the body surface for attaching nodes
    */
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
     
     //donor
     //  myDonor0 = createFemModel (
     //     myJawModel, "donor0", "resected_donor_transformed_remeshed3.obj", myBoneDensity, myBoneE, myBoneNu);
      
     //myDonor0 = createFemModel (
     //    myJawModel, "donor0", "case4_donor_inf_remeshed_transformed_boolean.obj", myBoneDensity, myBoneE, myBoneNu);
      
      myDonor0 = createFemModel (
         myJawModel, "donor0", "donor_opt0_remeshed.obj", myBoneDensity, myBoneE, myBoneNu);
      
      myDonor1 = createFemModel (
         myJawModel, "donor1", "donor_opt1_remeshed.obj", myBoneDensity, myBoneE, myBoneNu);
      
      //plate
      
      //String platePath = myGeoDir + "plate_final_case4_inf.art";
      String platePath = myGeoDir + "plate_opt.art";
   
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
      
      //attach the plate to the left and right mandible segments. We use
      // explicitly defined nodes to do this, since the plate may be some
      // distance from the segments.
           
      myMandibleRight = (RigidBody)myJawModel.findComponent (
      "rigidBodies/jaw_resected");
      
      myMandibleLeft = (RigidBody)myJawModel.findComponent (
      "rigidBodies/jaw");

      
      
      /*
      int[] leftAttachNodes = {78,57,56,77,76,55};
      
      attachFemToBody (myJawModel, myPlate, myMandibleLeft, leftAttachNodes);
      
      int[] rightAttachNodes = {69,48,70,49,71,50};
     
      attachFemToBody (myJawModel, myPlate, myMandibleRight, rightAttachNodes);
      
      */
      
      
      int[] leftAttachNodes = {0,1,4,5};
      
      attachFemToBody (myJawModel, myPlate, myMandibleLeft, leftAttachNodes);
      
      
      int numNodes = myPlate.numNodes (); 
      
      int[] rightAttachNodes = {numNodes-3, numNodes-4, numNodes-7, numNodes-8  };
     
      attachFemToBody (myJawModel, myPlate, myMandibleRight, rightAttachNodes);

      
      
      attachPlateToDonorSegments (myJawModel);

      
      setDonorSegmentInteractions (myJawModel);
 
   }
   
   
   
   
   /**
    * Helper method to attach the plate to the donor segments.
    */
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

         
         RigidBody screw1 = (RigidBody)myJawModel.findComponent ("rigidBodies/screw1");
         
         int  hexElem1  = findClosestHexElementNumber(myPlate,screw1);
       
        
          attachElemToSegmentforRigid (
              mech, screw1,  (HexElement)myPlate.getElementByNumber(hexElem1),
              myDonor1, attachTol);
          
          screw1.setDensity (myTitaniumDensity);


      
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


   
   /**
    * Helper method to set the interactions between donor segments and the
    * mandible segments.
    */
   private void setDonorSegmentInteractions (MechModel mech) {
      
      
      LinearElasticContact EFContact = new LinearElasticContact (DEFAULT_E, DEFAULT_Nu, DEFAULT_Damping, DEFAULT_Thickness);

      CollisionBehavior behav7 = new CollisionBehavior (true, 0);
      CollisionBehavior behav8 = new CollisionBehavior (true, 0);
      CollisionBehavior behav9 = new CollisionBehavior (true, 0);

      
      behav7.setMethod(Method.VERTEX_PENETRATION);
      behav8.setMethod(Method.VERTEX_PENETRATION);
      behav9.setMethod(Method.VERTEX_PENETRATION);

     
      behav7.setForceBehavior (EFContact);
      behav8.setForceBehavior (EFContact);
      behav9.setForceBehavior (EFContact);

      CollisionManager cm = myJawModel.getCollisionManager();
      
      // use AJL collisions so we can render pressure maps:
      cm.setColliderType (ColliderType.AJL_CONTOUR);

      myJawModel.setCollisionBehavior (myJawModel.rigidBodies().get ("jaw_resected"), (FemModel3d) myJawModel.models().get ("donor0"), behav7);
      myJawModel.setCollisionBehavior (myJawModel.rigidBodies().get ("jaw"), (FemModel3d) myJawModel.models().get ("donor0"), behav8);
      myJawModel.setCollisionBehavior ((FemModel3d) myJawModel.models().get ("donor0"), (FemModel3d) myJawModel.models().get ("donor1"), behav9);

   
      behav7.setName ("donor_mandible1");
      behav8.setName ("donor_mandible2");
      behav9.setName ("donor_donor");

     
   }

   /**
    * Attach a hex element of plate FEM to one of the donor segment FEMs using
    * a rigid body representation of a screw. The hex element and nearby nodes
    * of the donor FEM at then all connected to the screw.
    *
    * @param mech MechModel containing all the components
    * @param hex hex element of the plate FEM
    * @param donorFem FEM model of the donor segment
    * @param screwLen length of the cylinder representing the screw
    * @param attachTol distance tolerance for attaching donor FEM
    * nodes to the screw
    */
   
 
   private void attachElemToSegmentforRigid (
      MechModel mech, RigidBody screw, HexElement hex, FemModel3d donorFem, double attachTol) {

      /*
      // compute centroid of the hex element
      Point3d cent = new Point3d();
      hex.computeCentroid (cent);

      // compute normal pointing toward the donor FEM. From the construction of
      // plate FEM, we know that this is given by the outward facing normal of
      // the quad face given by the first four hex nodes.
      Vector3d nrm = new Vector3d();
      FemNode3d[] nodes = hex.getNodes();
      Face.computeNormal (
         nrm, nodes[0].getPosition(), nodes[1].getPosition(),
         nodes[2].getPosition(), nodes[3].getPosition());

      // represent the screw as a cylinder with radius 1/10 of it length.
      RigidBody screw = RigidBody.createCylinder (
         null, screwLen/10, screwLen, myTitaniumDensity, 10);
      // Set the pose of the screw so that it lies along the normal starting at
      // the hex centroid.
      RigidTransform3d TSW = new RigidTransform3d ();
      TSW.p.set (cent);
      TSW.R.setZDirection (nrm);
      TSW.mulXyz (0, 0, screwLen/2);
      screw.setPose (TSW);
       */

      //mech.addRigidBody (screw); // add to the MechModel

      // attach to the screw all donor FEM nodes that are within attachTol of
      // its surface
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

   protected double bolusMaxResistance = 130; // N

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
      ArtisynthPath.getSrcRelativePath(JawFemDemoOptimizeTwoSeg.class, workingDirname));
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
   
   
  public void addControlPanel(){
  
     ControlPanel panel;
     panel = new ControlPanel("Parameter Tuning","LiveUpdate");
     panel.addLabel ("Ligaments");
     panel.addWidget (myJawModel, "StmSlack");
     panel.addWidget (myJawModel, "SphmSlack");
     panel.addWidget ("tm_R", this, "models/jawmodel/multiPointSprings/tm_R:restLength");
     panel.addWidget ("tm_L", this, "models/jawmodel/multiPointSprings/tm_L:restLength");
     panel.addWidget (new JSeparator());
     panel.addLabel ("Elastic Foundation Contact");
     panel.addWidget (myJawModel, "EFYoung");
     panel.addWidget (myJawModel, "EFThickness");
     panel.addWidget (myJawModel, "EFDamping");
     panel.addWidget (myJawModel, "EFNu");
     panel.addWidget (new JSeparator());
     panel.addLabel ("Capsule Render Properties");
     panel.addWidget ("sapsule_r", this, "models/jawmodel/models/capsule_r:renderProps.visible");
     panel.addWidget ("sapsule_l", this, "models/jawmodel/models/capsule_l:renderProps.visible");
     addControlPanel (panel);
     panel.pack ();
     
    
  }
 
}
