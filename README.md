# SLAM_Resources
SLAM Resources to follow up current SLAM trends and papers.

Inspired by [Event-based vision resources](https://github.com/uzh-rpg/event-based_vision_resources)

Also reference pages are listed on [Pages collect resources for SLAM](#slamlist)

**What I cannot create, I do not understand.** - richard feynman

**Do the simplest thing that could possibly work** 

## Table of Contents:
- [Algorithms](#algorithms)
- [Datasets and Simulators](#datasets)
- [Evaluation](#evaluation)
- [Workshops](#workshops)
- [Tutorials](#tutorials)
- [Survey](#survey)
- [Papers](#papers)
- [Books](#books)
- [Pages collect resources for SLAM](#slamlist)
- [Toolkit](#toolkit)
- [Videos, Lectures](#lecture)
___
<br>

<a name="algorithms"></a>
## Algorithms
- [Todo Later]

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
## Papers 
2004 : Visual Odometry(Nister, CVPR 04)
2006 : Scalable monocular SLAM(Eade,Drummond,CVPR 06)
2007 : PTAM(Klein,Murray, ISMAR 07)채
2007 : MonoSLAM(Davison,Reid,Molton,Stasse, PAMI 07)
2011 : DTAM(Newcombe,Lovegrove,Davison,ICCV 11)
2013 : Semi-Dense Visual Odometry(J. Engel, J. Sturm, Davision, ICCV 13)
2014 : SVO(Forster,Pizzoli,Scarammuzza, ICRA 14)
       LSD-SLAM(Jacob Engel, T.Schoeps, Davision, ECCV 14)
       REMODE(M. Pizzoli, C. Forster, D. Scrammuza, ICRA 14)
       DTSLAM(Herrera C., D., Kim, K., Kannala, J., Pulli, K., Heikkila, J., 3DV, 14)
2015 : ORB_SLAM(Raul Mur-Artal, J. Montiel, J. Tardos, IEEE TRO 15)
       OKVIS(S. Leutenegger, S. Lynen, M. Bosse, R. Siegwart, P.Furgale, IJRR 15)
       DPPTAM(Concha, Alejo and Civera, Javier, IROS 15)
       SOFT2 : Stereo odometry based on careful feature selection and tracking(I Cvišić, I Petrović, ECCV 15)
2016 : EVO(H. Rebecq, T. Horstschaefer, G. Gallego, D. Scaramuzza, IEEE RA-L 16)
2017 : DSO(Jacob Engel, V. Kltun, Davison, PAMI 17)
       ORB_SLAM2(R Mur-Artal, JD Tardós, IEEE TRO 17)
       Real-time VIO for Event Cameras using Keyframe-based Nonlinear Optimization(H.Rebecq, T. Horstschaefer, D. Scaramuzza, BMVC 17)
       On-Manifold Preintegration for Real-Time VIO (C. Forster, L. Carlone, F. Dellaert, D. Scaramuzza, IEEE RA-L 17, Best Paper)
       SVO2 : SVO for Monocular and Multi-Camera Systems(C. Forster, Z. Zhang, M. Gassner, M. Werlberger, D. Scaramuzza, IEEE TRO 17)
2018 : VINS-Mono(T. Qin, Tong and Li, Peiliang, Shen, Shaojie, IEEE TRO 18)
       Ultimate SLAM?(T. Rosinol Vidal, H.Rebecq, T. Horstschaefer, D. Scaramuzza, IEEE RA-L 18)
       Event-based, 6-DOF Camera Tracking from Photometric Depth Maps
       (G. Gallego, Jon E. A. Lund, E. Mueggler, H.Rebecq, T. Delbruck, D. Scaramuzza, PAMI 18)
    
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
## Workshops 
- [2014 CVPR Workshop and Tutorials](http://frc.ri.cmu.edu/~kaess/vslam_cvpr14/)
- [2016 ICRA SLAM Tutorials](http://www.dis.uniroma1.it/~labrococo/tutorial_icra_2016/)

<a name="tutorials"></a>
## Tutorials
- [TODO-later]

<a name="books"></a>
## Books
- [slambook](), [source](https://github.com/gaoxiang12/slambook)

<a name="slamlist"></a>
## resource pages that I refer to create this slam list pages 
- [awesome-SLAM-list](https://github.com/OpenSLAM/awesome-SLAM-list)

<a name="toolkit"></a>
## Toolkis and Libraries for SLAM
- [Todo Later]

<a name="lecture"></a>
## Lectures
- [Todo Later]

## Videos 
- [Todo Later] 
