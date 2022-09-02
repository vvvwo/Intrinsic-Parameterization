# Intrinsic-Parameterization for 3D Facial Surface
This project is used to implement the intrinisc parameterization for 3D faialc surface.

![para](https://user-images.githubusercontent.com/65271555/188065294-2cfc5f91-a2e3-4b86-9a88-0a521b378045.jpg)

### Introduction

The project is based on the two papers: 

**[1] Desbrun *et al.*** <a href="https://www.researchgate.net/profile/Mohamed-Mourad-Lafifi/post/I_want_to_perform_Least_Square_Conformal_Parametrization_on_a_3D_mesh_what_are_the_tools_that_i_can_use_to_do_this/attachment/59d63eaf79197b807799b41a/AS%3A424797202063365%401478290954021/download/Intrinsic+Parameterizations+of+Surface+Meshes.pdf)" target="_blank">Intrinsic parameterizations of surface meshes</a>, 2002.

**[2] Lv *et al.*** <a href="https://aliexken.github.io/papers/2018%20Constructing3DFacialHierarchic.pdf" target="_blank">Constructing 3D facial hierarchical structure based on surface measurements</a>, 2019. 

It extracts the geodesic-based subsurface from the 3D facial surface and constructs the 2D parameterization. In this repository, we release code and executable file.

### Installation

Install <a href="https://www.tensorflow.org/get_started/os_setup" target="_blank">TensorFlow</a>. You may also need to install h5py. The code has been tested with Python 2.7, TensorFlow 1.0.1, CUDA 8.0 and cuDNN 5.1 on Ubuntu 14.04.

If you are using PyTorch, you can find a third-party pytorch implementation <a href="https://github.com/fxia22/pointnet.pytorch" target="_blank">here</a>.

To install h5py for Python:
```bash
sudo apt-get install libhdf5-dev
sudo pip install h5py
```

### Usage
You can run the .exe to generate geodesic-based subsurface and parameterization result.
```bash
Intri_Parameter.exe Data\pjanic.ply 18504 4858 1.2
Intri_Parameter.exe Data\pjanic.ply 18504 1.2
```
Parameters: 1) path of the model; 2) specified middle point for the 3D face; 3) eyebrow point to define the direction (it can be ignored); 4) geodesic searching radius.

![para2](https://user-images.githubusercontent.com/65271555/188072256-8793b42e-a0c5-4e96-aab4-6ae982ce2994.jpg)

### Citation
If you find our work useful in your research, please consider citing:

     @article{lv2019constructing,
        title={Constructing 3D facial hierarchical structure based on surface measurements},
        author={Lv, Chenlei and Wu, Zhongke and Wang, Xingce and Zhou, Mingquan},
        journal={Multimedia Tools and Applications},
        volume={78},
        number={11},
        pages={14753--14776},
        year={2019},
        publisher={Springer}
     }
  
     @inproceedings{desbrun2002intrinsic,
        title={Intrinsic parameterizations of surface meshes},
        author={Desbrun, Mathieu and Meyer, Mark and Alliez, Pierre},
        booktitle={Computer graphics forum},
        volume={21},
        number={3},
        pages={209--218},
        year={2002},
        organization={Wiley Online Library}
     }
