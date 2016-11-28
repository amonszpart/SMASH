# SMASH
Project page: [SMASH: Physics-guided Reconstruction of Collisions from Videos](http://geometry.cs.ucl.ac.uk/projects/2016/smash/), <br />
Venue: [SIGGRAPH Asia 2016](https://sa2016.siggraph.org) <br />
Authors: [Aron Monszpart](http://geometry.cs.ucl.ac.uk/amonszpart)<sup>1</sup>, [Nils Thuerey](http://ntoken.com)<sup>2</sup>, [Niloy J. Mitra](http://geometry.cs.ucl.ac.uk)<sup>1</sup><br />
<sup>1</sup>University College London <sup>2</sup> Techincal University of Munich

---

### Dependencies
##### Eigen <small>([link](http://eigen.tuxfamily.org/index.php?title=Main_Page))</small>
##### OpenCV <small>([link](http://opencv.org/downloads.html))</small>
| Core modules: | Contrib modules: |
|:-------------:|:----------------:|
| imgproc       | cudabgsegm       |
| highgui       | cudalegacy       |
| core          | [xfeatures2d](https://github.com/opencv/opencv_contrib/tree/master/modules/xfeatures2d) |
Note: see [opencv_contrib/README.md](https://github.com/opencv/opencv_contrib/blob/master/README.md) for installation.

##### [Google] Ceres <small>([link](http://ceres-solver.org))</small>
##### VTK <small>([link](http://vtk.org))</small>
* vtkCommonMath 
* vtkCommonCore 
* vtkRenderingOpenGL 
* vtkRenderingLOD 
* vtkInteractionStyle


### Platform
The code was developed and tested on *Ubuntu 16.04 LTS* with *gcc 6.2*.

---

### Installation
```bash
git clone git@github.com:amonszpart/SMASH.git
cd SMASH
mkdir build && cd build
cmake [-DOpenCV_DIR=<path/to/opencv/share/OpenCV>]..
make -j 8

```

---
### Example
```bash
wget http://geometry.cs.ucl.ac.uk/projects/2016/smash/paper_docs/smash_data.zip
unzip smash_data.zip
cd data/duckElephant
cat run.sh
../../bin/smash --intr intrinsics.txt \
                --frames 0,101,203 \
                --init 2dparabolasInit.json \
                --cub cuboids.json \
                --img-pattern orig240/color_%05d.jpg \
                --fps 240 \
                --show-flags 100000 \
                --weight-velocity 3.16 \
                --weight-conservation-lin 3.16 \
                --weight-observed-q 1. \
                --weight-observed-x 3.16
```

---

### SMASH Blender plugin
Open the user <b>File</b> -> <b>User Preferences</b>, Select the Add-ons tab, press <b>Install from File</b> and select *smash_blender_plugin/physacqPanel.py*.
If you want it to be enabled on restart, press <b>Save User Settings</b>.