# CRD-Fusion for Disparity Prediction
This package includes modules proposed in [CRD-Fusion](https://github.com/fanxiule/CRD_Fusion) and a script to refine raw disparity published by the camera.

## Using the Package
After starting the robot-camera simulator, run the following command to predict disparity by using the CRD-Fusion pipeline
```
roslaunch crd_fusion_perception refine_disp.launch
```

Pretrained weights are already saved in the `checkpt/` directory.