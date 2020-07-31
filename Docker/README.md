
# How to build docker:

```Shell
# In Docker folder 
source docker _build.sh
```  
  
# How to run container:
```Shell
# In Mask-RCNN-Detectron2 folder 
source docker_run.sh  
  
# For another terminal
source docker_join.sh  
```

#### Using a persistent cache directory

You can prevent models from being re-downloaded on every run,
by storing them in a cache directory.

To do this, add `--volume=$HOME/.torch/fvcore_cache:/tmp:rw` in the run command.
