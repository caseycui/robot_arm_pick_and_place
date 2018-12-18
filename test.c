
[ERROR] [1545102428.459410, 1408.330000]: bad callback: <function pcl_callback at 0x7f5e72622e60>
Traceback (most recent call last):
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rospy/topics.py", line 750, in _invoke_callback
    cb(msg)
  File "/home/robond/catkin_ws/src/RoboND-Perception-Project/pr2_robot/scripts/project_template.py", line 162, in pcl_callback
    prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
  File "/home/robond/.local/lib/python2.7/site-packages/sklearn/svm/base.py", line 567, in predict
    y = super(BaseSVC, self).predict(X)
  File "/home/robond/.local/lib/python2.7/site-packages/sklearn/svm/base.py", line 325, in predict
    X = self._validate_for_predict(X)
  File "/home/robond/.local/lib/python2.7/site-packages/sklearn/svm/base.py", line 458, in _validate_for_predict
    accept_large_sparse=False)
  File "/home/robond/.local/lib/python2.7/site-packages/sklearn/utils/validation.py", line 568, in check_array
    allow_nan=force_all_finite == 'allow-nan')
  File "/home/robond/.local/lib/python2.7/site-packages/sklearn/utils/validation.py", line 56, in _assert_all_finite
    raise ValueError(msg_err.format(type_err, X.dtype))
ValueError: Input contains NaN, infinity or a value too large for dtype('float64').
