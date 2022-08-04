# CRD-Fusion for Disparity Prediction
This package includes code based on [CRD-Fusion](https://github.com/fanxiule/CRD_Fusion) to refine raw disparity

## Using the Package
Use the following command to start the RealSense camera. You can change the camera settings in the launch file
```
$ roslaunch crd_fusion_perception rs_camera.launch
```

Use the following command to start the RealSense camera AND the CRD-Fusion pipeline
```
$ roslaunch crd_fusion_perception refine_rs_disp.launch
```

Pretrained weights are already saved in the `checkpt/` directory.