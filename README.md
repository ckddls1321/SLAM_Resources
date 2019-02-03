# SLAM_Resources
Personal page of SLAM Resources to follow up current SLAM trends and papers.

Inspired by [Event-based vision resources](https://github.com/uzh-rpg/event-based_vision_resources)

Also reference pages are listed on [Pages collect resources for SLAM](#slamlist)

**What I cannot create, I do not understand.** - richard feynman

**Do the simplest thing that could possibly work** 

## Table of Contents:
- [Algorithms](#algorithms)
- [Sensor Model](#models)
- [Datasets and Simulators](#datasets)
- [Calibration](#calibration)
- [Evaluation](#evaluation)
- [Workshops&Tutorials](#workshops)
- [Survey](#survey)
- [Papers](#papers)
- [Deep Learning Related SLAM](#deepslam)
- [Semantic SLAM - Object level SLAM](#semanticslam)
- [Books](#books)
- [Pages : collect resources for SLAM](#slamlist)
- [Toolkit](#toolkit)
- [Videos, Lectures](#lecture)
- [Visualization](#visualization)
___
<br>

<a name="algorithms"></a>
## Algorithms
### Initialization
- #### Homography
- #### Fundemental
- #### SFM
- #### Visual-Inertial Alignment 
### Tracking
- #### Data Association : How to Define Data Selection, Match, Define Error 
  - ##### Direct Dense
  - ##### Direct Sparse
  - ##### Feature (Sparse)
    - ###### Corner Selection
    - ###### Descriptors
  - ##### Feature Match
- #### Motion Prior
  - ##### Constant Velocity Model
  - ##### Decaying Velocity Model
  - ##### [IMU Propagation](#imu)
  - ##### Using Prev Pose
- #### Pose Estimation : How to minimize Error
  - ##### PnP : Perspective N Points
  - ##### Motion Only BA : **Coarse**-Fine 
  - ##### Local BA : Coarse-**Fine**
    - ###### Sliding Window : Continous N Frame window
    - ###### Topological : Releated Keyframes 

### Mapping
- #### Map Type
- #### Map Generation

### Global Consistency 
- #### Relocalization
- #### Pose Graph Optimization : Loop Closure 
- #### Place Recognition 

### Probabilistic Graphical Models 
- #### Factor Graph 

<a name="models"></a>
## Sensor Models 
### Camera Models & Undistorttion Models
- [Image_Undistorter](https://github.com/ethz-asl/image_undistort)
- [Camera Models](https://github.com/gaowenliang/-camera_model) - modified version of CamOdoCal 
- ROS Image Proc => Wiki Documentation of ROS [image pipeline](http://wiki.ros.org/image_pipeline/CameraInfo) 
<a name="imu"></a>
### IMU Models
- #### Noise Model
- #### IMU Propagation
- #### IMU Preintegration

<a name="calibration"></a>
## Calibration
### Geometric Calibration : Reprojection Error
- [GML: C++ Calibration Toolbox](https://graphics.cs.msu.ru/en/node/909)
- [ROS camera calibration](http://wiki.ros.org/camera_calibration)
- [Camera Calibration Toolbox for Matlab](http://www.vision.caltech.edu/bouguetj/calib_doc/)
- [CamOdoCal](https://github.com/hengli/camodocal)
- [OCamCalib: Omni-Camera Calibration](https://sites.google.com/site/scarabotix/ocamcalib-toolbox)
### Photometric Calibration
- [TUM, Online Photometric calibration](https://github.com/tum-vision/online_photometric_calibration)
### Visual-Inertial Calibration : Reprojection Error + Extrinsic of Camera-IMU
- [Kalibr](https://github.com/ethz-asl/kalibr)
- [Vicalib](https://github.com/arpg/vicalib)
- [Inervis Toolbox-Matlab](http://home.deec.uc.pt/~jlobo/InerVis_WebIndex/InerVis_Toolbox.html)
### Visual-Lidar Calibration : 
### Lidar-IMU Calibration 
### IMU Calibration - Not sure... 
- [IMUSensorModels-Data_Analysis_Tools](https://github.com/hanley6/IMUSensorModels)
- [Kalibr_allan](https://github.com/rpng/kalibr_allan)
- [NaveGO: an open-source MATLAB/GNU Octave toolbox for processing INS and performing IMU analysis](https://github.com/rodralez/NaveGo)
- [imu_utils : ROS package tool to analyze the IMU performance](https://github.com/gaowenliang/imu_utils)

<a name="survey"></a>
## Survey or Tutorial papers for slam users
- [Past, Present, and Future of Simultaneous Localization and Mapping: Toward the Robust-Perception Age](https://arxiv.org/pdf/1606.05830.pdf) 
- [Keyframe-based monocular SLAM: design, survey, and future directions](https://arxiv.org/pdf/1607.00470.pdf)
- [Local Invariant Feature Detectors: A Survey](http://homes.esat.kuleuven.be/~tuytelaa/FT_survey_interestpoints08.pdf)
- [Visual Odometry Part I: The First 30 Years and Fundamentals](https://www.ifi.uzh.ch/dam/jcr:5759a719-55db-4930-8051-4cc534f812b1/VO_Part_I_Scaramuzza.pdf)
- [Visual odometry: Part II: Matching, robustness, optimization, and applications](http://www.zora.uzh.ch/71030/1/Fraundorfer_Scaramuzza_Visual_odometry.pdf)
- [Visual simultaneous localization and mapping : a survey](https://www.researchgate.net/publication/234081012_Visual_Simultaneous_Localization_and_Mapping_A_Survey)
- [Simultaneous Localization and mapping : a survey of current trends in Autonomous Driving](https://hal.archives-ouvertes.fr/hal-01615897/file/2017-simultaneous_localization_and_mapping_a_survey_of_current_trends_in_autonomous_driving.pdf)
- [Visual SLAM Algorithms : a survey from 2010 to 2016](https://ipsjcva.springeropen.com/articles/10.1186/s41074-017-0027-2) 

<a name="papers"></a>
## Papers : ordered by year but not strictly ordered, not fully collected.
- [Visual Odometry, Nister, CVPR 04](https://www.computer.org/csdl/proceedings/cvpr/2004/2158/01/01315094.pdf) 
- [Scalable monocular SLAM, E. Eade,T. Drummond, CVPR 06](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.141.7753&rep=rep1&type=pdf)
- [Parallel Tracking and Mapping(PTAM) for Small AR Workspaces, Georg Klein, David Murray, ISMAR 07](https://www.robots.ox.ac.uk/~vgg/rg/papers/klein_murray__2007__ptam.pdf)
- [MonoSLAM, AJ Davison, Reid, Molton, Stasse, PAMI 07](https://www.doc.ic.ac.uk/~ajd/Publications/davison_etal_pami2007.pdf) 
- [DTAM: Dense Tracking and Mapping in Real-Time, RA Newcombe, Steven J. Lovegrove, AJ Davison, ICCV 11](https://www.robots.ox.ac.uk/~vgg/rg/papers/newcombe_davison__2011__dtam.pdf)
- Dense Visual SLAM for RGB-D Camera
- [Semi-Dense Visual Odometry, J. Engel, J. Sturm, AJ Davision, ICCV 13](https://vision.in.tum.de/_media/spezial/bib/engel2013iccv.pdf)
- [SVO: Fast Semi-Direct Monocular Visual Odometry, C Forster, M. Pizzoli, D. Scarammuzza, ICRA 14](https://www.ifi.uzh.ch/dam/jcr:e9b12a61-5dc8-48d2-a5f6-bd8ab49d1986/ICRA14_Forster.pdf) 
- [LSD-SLAM: Large-Scale Direct Monocular SLAM, J. Engel, T.Schoeps, AJ Davision, ECCV 14](https://vision.in.tum.de/_media/spezial/bib/engel14eccv.pdf)
- [REMODE, M. Pizzoli, C. Forster, D. Scrammuza, ICRA 14](http://rpg.ifi.uzh.ch/docs/ICRA14_Pizzoli.pdf) 
- Dense Visual-Inertial Odometry for Tracking of Aggressive Motions
- [ORB_SLAM, R. Mur-Artal, J. Montiel,  JD Tardós, IEEE TRO 15](https://arxiv.org/pdf/1502.00956)
- [OKVIS, S. Leutenegger, S. Lynen, M. Bosse, R. Siegwart, P.Furgale, IJRR 15](http://www.roboticsproceedings.org/rss09/p37.pdf)
- [DPPTAM, Concha, Alejo and Civera, Javier, IROS 15](http://webdiis.unizar.es/~jcivera/papers/concha_civera_iros15.pdf)
- [SOFT2 : Stereo odometry based on careful feature selection and tracking, I Cvišić, I Petrović, ECCV 15](http://www.cvlibs.net/projects/autonomous_vision_survey/literature/Cvisic2015ECMR.pdf)
- [EVO: A Geometric Approach to Event-Based 6-DOF Parallal Tracking and Mapping in Real-time, H. Rebecq, T. Horstschaefer, G. Gallego, D. Scaramuzza, IEEE RA-L 16](http://rpg.ifi.uzh.ch/docs/RAL16_EVO.pdf)
- [On-Manifold Preintegration for Real-Time VIO, C. Forster, L. Carlone, F. Dellaert, D. Scaramuzza, IEEE RA-L 17](http://rpg.ifi.uzh.ch/docs/TRO16_forster.pdf) 
- [ORB_SLAM2, R Mur-Artal, JD Tardós, IEEE TRO 17](https://arxiv.org/pdf/1610.06475.pdf)
- [Direct Sparse Odometry, J. Engel, V. Kltun, AJ Davison, PAMI 17](https://vision.in.tum.de/research/vslam/dso)
- [Real-time VIO for Event Cameras using Keyframe-based Nonlinear Optimization, H.Rebecq, T. Horstschaefer, D. Scaramuzza, BMVC 17](http://rpg.ifi.uzh.ch/docs/BMVC17_Rebecq.pdf)    
- ElasticFusion: Dense SLAM Without A Pose Graph
- Dense RGB-D-Inertial SLAM with Map Deformations 
- [SVO for Monocular and Multi-Camera Systems, C. Forster, Z. Zhang, M. Gassner, M. Werlberger, D. Scaramuzza, IEEE TRO 17](http://rpg.ifi.uzh.ch/docs/TRO16_Forster-SVO.pdf)
- [VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator, T. Qin, Tong and Li, Peiliang, Shen, Shaojie, IEEE TRO 18](https://arxiv.org/pdf/1708.03852) 
- [Ultimate SLAM?
Combining Events, Images, and IMU for Robust
Visual SLAM in HDR and High Speed Scenarios, T. Rosinol Vidal, H.Rebecq, T. Horstschaefer, D. Scaramuzza, IEEE RA-L 18](https://arxiv.org/pdf/1709.06310.pdf)
- [Event-based, 6-DOF Camera Tracking from Photometric Depth Maps, Gallego, Jon E. A. Lund, E. Mueggler, H.Rebecq, T. Delbruck, D. Scaramuzza, PAMI 18](http://rpg.ifi.uzh.ch/docs/PAMI17_Gallego.pdf)
- [Loosely-Coupled Semi-Direct Monocular SLAM, Seong Hun Lee and Javier Civera, IEEE Robotics and Automation Letters]
(https://arxiv.org/pdf/1807.10073.pdf)

<a name="deepslam"></a>
## Deep SLAM : Depth Estimation, Pose Estimation, Feature Matching, Backend etc... What ever use Deep Neural Network
- [DeepVO: A Deep Learning approach for Monocular Visual Odometry, Vikram Mohanty, Shubh Agrawal, Shaswat Datta, Arna Ghosh, Vishnu D. Sharma, Debashish Chakravarty](https://arxiv.org/pdf/1611.06069.pdf)
- [CNN-SLAM: Real-time dense monocular SLAM with learned depth prediction, CVPR, 2017, Keisuke Tateno, Federico Tombari, Iro Laina, Nassir Navab](http://openaccess.thecvf.com/content_cvpr_2017/papers/Tateno_CNN-SLAM_Real-Time_Dense_CVPR_2017_paper.pdf)
- [Deep Virtual Stereo Odometry: Leveraging Deep Depth Prediction for Monocular Direct Sparse Odometry, Nan Yang, Rui Wang, J¨org St¨uckler, Daniel Cremers](http://openaccess.thecvf.com/content_ECCV_2018/papers/Nan_Yang_Deep_Virtual_Stereo_ECCV_2018_paper.pdf)
- [UnDeepVO: Monocular Visual Odometry through Unsupervised Deep Learning](https://arxiv.org/pdf/1709.06841.pdf)
- [SfMLearner++: Learning Monocular Depth & Ego-Motion using Meaningful Geometric Constraints, Vignesh Prasad, Brojeshwar Bhowmick](https://arxiv.org/pdf/1812.08370.pdf)
- [CNN-SVO: Improving the Mapping in Semi-Direct Visual OdometryUsing Single-Image Depth Prediction, Shing Yan Loo, Ali Jahan, Amiri, Syamsiah Mashohor, Sai Hong Tang and Hong Zhang1](https://arxiv.org/pdf/1810.01011.pdf)
- [Learning monocular visual odometry with dense 3D mapping from dense 3D flow, Cheng Zhao, Li Sun, Pulak Purkait, Tom Duckett and Rustam Stolkin1](https://arxiv.org/pdf/1803.02286.pdf)
- [Learning to Prevent Monocular SLAM Failure using Reinforcement Learning, Vignesh Prasad, Karmesh Yadav, Rohitashva Singh Saurabh, Swapnil Daga, Nahas Pareekutty, K. Madhava Krishna. Balaraman Ravindran, Brojeshwar Bhowmick](https://arxiv.org/pdf/1812.09647.pdf)
- CodeSLAM - Learning a Compact, Optimisable Representation for Dense Visual SLAM, Michael Bloesch, Jan Czarnowski, Ronald Clark, Stefan Leutenegger, Andrew J. Davison.
- LS-Net: Learning to Solve Nonlinear Least Squares for Monocular Stereo. ECCV, 2018, Ronald Clark, Michael Bloesch, Jan Czarnowski, Stefan Leutenegger, Andrew J. Davison. 
- [DeepTAM: Deep Tracking and Mapping, Huizhong Zhou, Benjamin Ummenhofer, Thomas Brox](https://arxiv.org/pdf/1808.01900.pdf)
- [Deep Auxiliary Learning for Visual Localization and Odometry, Abhinav Valada, Noha Radwan, Wolfram Burgard](http://ais.informatik.uni-freiburg.de/publications/papers/valada18icra.pdf)
- [Mask-SLAM: Robust feature-based monocular SLAM by masking using semantic segmentation, CVPR 2018, Masaya Kaneko Kazuya Iwami Toru Ogawa Toshihiko Yamasaki Kiyoharu Aiza](http://openaccess.thecvf.com/content_cvpr_2018_workshops/papers/w9/Kaneko_Mask-SLAM_Robust_Feature-Based_CVPR_2018_paper.pdf)
- [MagicVO: End-to-End Monocular Visual Odometry through Deep Bi-directional Recurrent Convolutional Neural Network, Jian Jiao, Jichao Jiao, Yaokai Mo, Weilun Liu, Zhongliang Deng](https://arxiv.org/ftp/arxiv/papers/1811/1811.10964.pdf)
- [Global Pose Estimation with an Attention-based Recurrent Network](https://arxiv.org/pdf/1802.06857.pdf)
- [Geometric Consistency for Self-Supervised End-to-End Visual Odometry, CVPR 2018, Ganesh Iyer, J. Krishna Murthy, Gunshi Gupta1, K. Madhava Krishna1, Liam Paull](http://openaccess.thecvf.com/content_cvpr_2018_workshops/papers/w9/Iyer_Geometric_Consistency_for_CVPR_2018_paper.pdf)
- [DepthNet: A Recurrent Neural Network Architecture for Monocular Depth Prediction, CVPR 2018, Arun CS Kumar Suchendra M. Bhandarkar, Mukta Prasad](http://openaccess.thecvf.com/content_cvpr_2018_workshops/papers/w9/Kumar_DepthNet_A_Recurrent_CVPR_2018_paper.pdf)

- DeepFusion: Real-Time Dense 3D Reconstruction for Monocular SLAM using Single-View Depth and Gradient Predictions. ICRA, 2019, Tristan Laidlow, Jan Czarnowski, Stefan Leutenegger. 
- KO-Fusion: Dense Visual SLAM with Tightly-Coupled Kinematic and Odometric Tracking. ICRA, 2019, Charlie Houseago, Michael Bloesch, Stefan Leutenegger. 
- DF-SLAM: A Deep-Learning Enhanced Visual SLAM System based on Deep Local Features, Rong Kang, Xueming Li, Yang Liu, Xiao Liu, Jieqi Shi 


<a name="semanticslam"></a>
## Semantic SLAM, Object-level, Using Semantic Information
- [Probabilistic Data Association for Semantic SLAM, Sean L. Bowman Nikolay Atanasov Kostas Daniilidis George J. Pappas](https://www.cis.upenn.edu/~kostas/mypub.dir/bowman17icra.pdf)
- Fusion++: Volumetric Object-Level SLAM. 3DV, 2018, John McCormac, Ronald Clark, Michael Bloesch, Stefan Leutenegger, Andrew J. Davison. 
- [DynSLAM: Simultaneous Localization and Mapping in Dynamic Environments,Ioan Andrei Brsan and Peidong Liu and Marc Pollefeys and Andreas Geiger](https://arxiv.org/pdf/1806.05620.pdf) 


<a name="evaluation"></a>
## Evaluation
- [Python package for evaluation of odometry and SLAM](https://github.com/MichaelGrupp/evo)
- [uzh-rpg : rpg_trajectory_evaluation](https://github.com/uzh-rpg/rpg_trajectory_evaluation), [papers](http://rpg.ifi.uzh.ch/docs/IROS18_Zhang.pdf)
- [TUM, useful tools for the RGBD benchmark](https://vision.in.tum.de/data/datasets/rgbd-dataset/tools) 
- [TUM, Matlab tools for evaluation](vision.in.tum.de/mono/evaluation_code_v2.zip), provided by [TUM, DSO : Direct Sparse Odometry](https://vision.in.tum.de/research/vslam/dso)

<a name="datasets"></a>
## Datasets and Simulators 
- [Awesome SLAM Dataset](https://sites.google.com/view/awesome-slam-datasets/) : 
    - [2018 TUM Visual Inertial Dataset](https://vision.in.tum.de/data/datasets/visual-inertial-dataset) : Stereo,IMU,Calibrated(+Photometric)
    - [2018 MVSEC : Multi Vehicle Stereo Event Dataset](https://daniilidis-group.github.io/mvsec/) : Stereo, Event, IMU
    - [2016 TUM Mono Dataset](https://vision.in.tum.de/data/datasets/mono-dataset) : Mono,IMU,Photometric Calibration
    - [2016 RPG Event Dataset](http://rpg.ifi.uzh.ch/davis_data.html) : Mono,Event,IMU
    - [2016 EuRoC Dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) : Stereo,IMU
    - [2015 TUM Omni Dataset](https://vision.in.tum.de/data/datasets/omni-lsdslam) : Mono,Omni,IMU
    - [2014 ICL-NUIM Dataset](https://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html) : Mono,RGB-D
    - [2014 MRPT-MALAGA Dataset](https://www.mrpt.org/robotics_datasets) 
    - [2013 KITTI Dataset](http://www.cvlibs.net/datasets/kitti/index.php)
    
<a name="workshops"></a>
## Workshops & Tutorials
- [2014 CVPR Workshop and Tutorials](http://frc.ri.cmu.edu/~kaess/vslam_cvpr14/)
- [2015 ICCV Imperial college Workshop](http://wp.doc.ic.ac.uk/thefutureofslam/)
- [2016 ICRA SLAM Tutorials](http://www.dis.uniroma1.it/~labrococo/tutorial_icra_2016/)
- 2017 CVPR Tutorials - pages removed
- [2018 CVPR Tutorials - First Deep SLAM Workshop](http://visualslam.ai/)

<a name="books"></a>
## Books
- [slambook-no en,kr translation](), [source](https://github.com/gaoxiang12/slambook)

<a name="slamlist"></a>
## resource pages that I refer to create this slam list pages 
- [awesome-SLAM-list](https://github.com/OpenSLAM/awesome-SLAM-list)
- [SFM-Visual-SLAM](https://github.com/marknabil/SFM-Visual-SLAM)
- [Event Vision Realted Resources - ETH Zurich](https://github.com/uzh-rpg/event-based_vision_resources)

<a name="toolkit"></a>
## Toolkits and Libraries for SLAM
### Computer Vision 
- [OpenCV-Computer Vision](https://opencv.org/)
- [MexOpenCV-Matlab mex functions for OpenCV](https://github.com/kyamagu/mexopencv) 
### Mathmatics
- [Eigen-Linear Algebra](http://eigen.tuxfamily.org/index.php?title=Main_Page)
- [Sophus-Lie Groups using Eigen](https://github.com/strasdat/Sophus)
### Optimization Solver
- [Ceres-NLLS Solver library](https://github.com/ceres-solver/ceres-solver)
- [g2o: A General Framework for Graph Optimization](https://github.com/RainerKuemmerle/g2o)
### 3D Data Processing
- [Open3D](http://www.open3d.org/)
- [Point Cloud Library](http://pointclouds.org/)

<a name="lecture"></a>
## Lectures
- [Multiple View Geometry, TUM, 2014](https://www.youtube.com/watch?v=RDkwklFGMfo&list=PLTBdjV_4f-EJn6udZ34tht9EVIW7lbeo4)
- [Roboot Mapping, University Freiburg, 2013](https://www.youtube.com/watch?v=U6vr3iNrwRA&list=PLgnQpQtFTOGQrZ4O5QzbIHgl3b1JHimN_)
## Videos
- [ARKit: Understanding ARKit Tracking and Dtection](https://developer.apple.com/videos/play/wwdc2018/610/) 

<a name="visualization"></a>
## Visualization
### Visualize GN-Optimization 
### Visualize Pose & 3D Map 
### Visualize Tracking
