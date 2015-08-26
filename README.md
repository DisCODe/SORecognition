SORecognition - DisCODe Component Library
=========================================

Description
-----------

DisCODe Component Library: SIFT Object Recognition

The library contains components and tasks usefull for recognition of objects on the basis of their SIFT models.

Dependencies
------------

DCL depends on the following libraries:
OpenCV - contains algorithms for 2D vision
PCL - contains algorithms for n-D perception

DCL depends on the following DCLs:
- CameraNUI - required for acquisition from Kinect-like sensors
- CvCoreTypes - contains basic OpenCV-related types
- CvBasic - contains components responsible for "classical" (2D) computer vision
- PCLCoreTypes - contains basic PCL-related types
- PCL - contains algorithms responsible for processing of point clouds (3D vision)
- Registration- required for registration of point clouds (e.g. finding correspondences)

Tasks
------------
Registration of models of objects from stereo camera images:

Tasks for object recognition - DisCODe working as ROS node:

   * __KinectRecognizeObjectsROS__ -

   * __SequenceRecognizeObjectsROS__ -


Tasks for object recognition (scene loaded from RGB-D sequence):

   * __DisplayRecognizedObjectsOnScene__ - PCL viewer displaying scene loaded from RGB-D images along with SOMs (SIFT Object Models) loaded from JSON files, matched against the scene, grouped into subsets and validated with verification algorithm.

Tasks for object recognition (from kinect):

   * __KinectDisplayRecognizedObjectsOnScene__ - PCL viewer displaying scene acquired from Kinect along with SOMs (SIFT Object Models) loaded from JSON files, matched against the scene, grouped into subsets, projected on the scene and validated with verification algorithm.


Tasks for testing/visualization of model-scene correspondences (scene loaded from RGB-D sequence):

   * __DisplayModelsSceneCorrespondences__ - PCL viewer displaying scene loaded from RGB-D images along with SOMs (SIFT Object Models) loaded from JSON files and matched against the scene.

   * __DisplayGroupedCorrespondencesOnScene__ - PCL viewer displaying scene loaded from RGB-D images along with SOMs (SIFT Object Models) loaded from JSON files, matched against the scene, grouped into subsets. The resulting hypothesis are finally projected on the scene.


Tasks for testing/visualization of model-scene correspondences (scene from Kinect):

   * __KinectDisplayGroupedCorrespondencesOnScene__ - PCL viewer displaying scene acquired from Kinect along with SOMs (SIFT Object Models) loaded from JSON files, matched against the scene and grouped into clusters. The resulting hypotheses are finally projected on the scene.

   * __KinectDisplayModelsSceneCorrespondences__ - PCL viewer displaying scene acquired from Kinect along with SOMs (SIFT Object Models) loaded from JSON files and matched against the scene.


Tasks for testing/visualization of scene/models purposes (scene loaded from RGB-D sequence):

   * __DisplayPCDSequence__ - Displays sequence of scene clouds. (task from DCL PCL!)

   * __DisplayJSOMModelsOnScene__ - PCL viewer displaying scene loaded from RGB-D images along with SOMs (SIFT Object Models) loaded from JSON files.

   * __KinectRGBDViewer__ - Displays RGB and Depth images acquired from Kinect (task from DCL CameraNUI!)

Maintainer
----------

tkornuta
