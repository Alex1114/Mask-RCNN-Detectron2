[Tutorial-by-Alex](https://drive.google.com/drive/folders/1Te0Z4GQ5oq1zPeyxpZb-ReG48k9kvdlQ?usp=sharing)  
  
  
Detectron2 is Facebook AI Research's next generation software system
that implements state-of-the-art object detection algorithms.
It is a ground-up rewrite of the previous version,
[Detectron](https://github.com/facebookresearch/Detectron/),
and it originates from [maskrcnn-benchmark](https://github.com/facebookresearch/maskrcnn-benchmark/).

<div align="center">
  <img src="https://user-images.githubusercontent.com/1381301/66535560-d3422200-eace-11e9-9123-5535d469db19.png"/>
</div>

### What's New
* It is powered by the [PyTorch](https://pytorch.org) deep learning framework.
* Includes more features such as panoptic segmentation, densepose, Cascade R-CNN, rotated bounding boxes, etc.
* Can be used as a library to support [different projects](projects/) on top of it.
  We'll open source more research projects in this way.
* It [trains much faster](https://detectron2.readthedocs.io/notes/benchmarks.html).

See our [blog post](https://ai.facebook.com/blog/-detectron2-a-pytorch-based-modular-object-detection-library-/)
to see more demos and learn about detectron2.

## Installation

See [INSTALL.md](INSTALL.md).

## Quick Start

See [GETTING_STARTED.md](GETTING_STARTED.md),
or the [Colab Notebook](https://colab.research.google.com/drive/16jcaJoc6bCFAQ96jDe2HwtXj7BMD_-m5).

Learn more at our [documentation](https://detectron2.readthedocs.org).
And see [projects/](projects/) for some projects that are built on top of detectron2.

## Model Zoo and Baselines

We provide a large set of baseline results and trained models available for download in the [Detectron2 Model Zoo](MODEL_ZOO.md).

## Citing Detectron2

If you use Detectron2 in your research or wish to refer to the baseline results published in the [Model Zoo](MODEL_ZOO.md), please use the following BibTeX entry.

```BibTeX
@misc{wu2019detectron2,
  author =       {Yuxin Wu and Alexander Kirillov and Francisco Massa and
                  Wan-Yen Lo and Ross Girshick},
  title =        {Detectron2},
  howpublished = {\url{https://github.com/facebookresearch/detectron2}},
  year =         {2019}
}
```

# Predict in ROS (Docker)

ROS wrapper for Mask-RCNN.

## Installation

Mask-RCNN uses Python 3.  
If you use a ROS version built with Python 2, additional steps are necessary to run the node.  
  
You need to build the cv_bridge module of ROS with Python 3.   
I recommend using a workspace separate from other ROS packages. Clone the package to the workspace.   
You might need to adjust some of the following instructions depending on your Python installation.  

  ```Shell
  # Have already in this repo.
  git clone -b melodic https://github.com/ros-perception/vision_opencv.git 
  ```

- First method catkin_make (recommend),  
  
  ```Shell
  source catkin_make.sh
  ```  
  
- Second method for script,
  ```Shell
  source how_to_solve_python3_cv2.sh
  ```  
    
Check your system can use cv_bridge in python3 (import getCvType).
  ```Shell
  python3  
    
  from cv_bridge.boost.cv_bridge_boost import getCvType  
  ```

![python3_cv](figures/python3_cv.png)
  
  
## How to run
  ```Shell
  # environment
  source environment.sh  
    
  # download model
  source download_model.sh

  # download dataset
  cd datasets
  python3 download_dataset.py   
    
  # launch  
  roslaunch rcnn_pkg mask_rcnn_prediction.launch
  ```

![prtdict in rviz](figures/MaskRCNN-predict.gif)


