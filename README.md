# vins_fusion_extra
用于vins_fusion中坐标转换。

之所以有这个包，是因为vins_fusion中输出的camera_pose的坐标系和飞机一般使用的坐标系不同，因此需要进一步转换。

这个包没有任何依赖，只需要在ros工作区间内编译即可。使用的时候按照自己需求修改`pose_transform.launch`，上面的参数非常直观。

除了坐标转换之外，该包还提供了`\RPY`话题，以方便使用者观察转换后刚体的欧拉角。
