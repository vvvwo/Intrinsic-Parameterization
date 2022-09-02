# Intrinsic-Parameterization for 3D Surface
This project is used to implement the intrinisc parameterization for 3D surface.

![para](https://user-images.githubusercontent.com/65271555/188065294-2cfc5f91-a2e3-4b86-9a88-0a521b378045.jpg)

### Introduction
This project is based on two papers: 

our [arXiv tech report](https://arxiv.org/abs/1612.00593), which is going to appear in CVPR 2017. We proposed a novel deep net architecture for point clouds (as unordered point sets). You can also check our [project webpage](http://stanford.edu/~rqi/pointnet) for a deeper introduction.

Point cloud is an important type of geometric data structure. Due to its irregular format, most researchers transform such data to regular 3D voxel grids or collections of images. This, however, renders data unnecessarily voluminous and causes issues. In this paper, we design a novel type of neural network that directly consumes point clouds, which well respects the permutation invariance of points in the input.  Our network, named PointNet, provides a unified architecture for applications ranging from object classification, part segmentation, to scene semantic parsing. Though simple, PointNet is highly efficient and effective.

In this repository, we release code and data for training a PointNet classification network on point clouds sampled from 3D shapes, as well as for training a part segmentation network on ShapeNet Part dataset.

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
