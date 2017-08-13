# volume-camera
multisensory depth-camera experiment

build in openFrameworks in C++
portions based upon openframeworks ofxKinect examples and meshRecorder example from weLoveCode.

designed around a vision of a volumetric stills camera/apparatus for street photography.

(with a nod of the head to Vilém Flusser https://en.wikipedia.org/wiki/Vilém_Flusser)

built under http://openFrameworks.cc v0.9.8 with dependencies to standard addons ofxKinect ofxXmlSetting, and additional addons ofxImGui and ofxCv.

https://github.com/jvcleave/ofxImGui
https://github.com/kylemcdonald/ofxCv

Currently integrated with Kinect v1 (1414) running under OSX, the source includes project files for Xcode v 8.8.3 development environment.

The software reads kinect data to a variable size 3d mesh with point cloud, faces and wireframe with and without surface normals and illumination. Depth and RGB data can be captured to disk as PNG sequences with EXIF meta data and replayed and manipulated.
Basic OpenCV routines are integrated allowing gaussian blur, eroding and dilation of either RGB or Depth (mesh) data on both live and pre-recorded data.

Please refer to the master branch as the most stable version and the project page for details of work in progress, the wiki for more detailed information, controls & Gui, references etc.
Please add any issues, bugs and requests to the issues track.

<iframe src="https://player.vimeo.com/video/229040967" width="640" height="400" frameborder="0" webkitallowfullscreen mozallowfullscreen allowfullscreen></iframe>
<p><a href="https://vimeo.com/229040967">test of openframeworks Kinect RGBD Volca volume camera software with simple OpenCv routines</a> from <a href="https://vimeo.com/danielbuzzo">Daniel Buzzo</a> on <a href="https://vimeo.com">Vimeo</a>.</p>

see more at http://buzzo.com/building-a-volumetric-camera/

Dan Buzzo 2017
