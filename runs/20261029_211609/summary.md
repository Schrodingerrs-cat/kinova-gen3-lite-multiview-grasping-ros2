# Run 20261029_211609

**Result:** 2 object(s) picked and put back from one observation

```
  reached the scan pose via: joint-space move to the saved scan joints
[B] observation: predetermined viewpoints covering the table, one point cloud each (done once)
  view 0 obl_near_left: tool xyz [0.2606594074782259, 0.09278388238086374, 0.24619957228630873], oblique, looking in from 25 deg off vertical | largest joint step 59 deg
  view 1 left_45: tool xyz [0.20647518010647187, 0.13859292911256332, 0.259], swung +45 deg about the base | largest joint step 45 deg
  view 2 obl_left: tool xyz [0.3610489173914124, 0.13802229267049912, 0.2649715486559109], oblique, looking in from 25 deg off vertical | largest joint step 45 deg
  view 3 obl_near: tool xyz [0.22113748231039965, -0.006772142213574901, 0.2366012858056306], oblique, looking in from 25 deg off vertical | largest joint step 38 deg
  view 4 left_22: tool xyz [0.24379541068627827, 0.049028539936540126, 0.259], swung +22 deg about the base | largest joint step 23 deg
  view 5 center: tool xyz [0.244, -0.048, 0.259], swung +0 deg about the base | largest joint step 0 deg
  view 6 right_22: tool xyz [0.20705780117922964, -0.13772097505762365, 0.259], swung -22 deg about the base | largest joint step 22 deg
  view 7 obl_right: tool xyz [0.3608184944058099, -0.1508154797491188, 0.2590059875489409], oblique, looking in from 25 deg off vertical | largest joint step 57 deg
  view 8 obl_near_right: tool xyz [0.2611501532856921, -0.10584728922698719, 0.2420539974723641], oblique, looking in from 25 deg off vertical | largest joint step 37 deg
  view 9 right_45: tool xyz [0.13859292911256327, -0.20647518010647187, 0.259], swung -45 deg about the base | largest joint step 45 deg
view 0 (obl_near_left): 84975 points, camera at [0.281 0.126 0.341]
view 1 (left_45): 98213 points, camera at [0.245 0.18  0.345]
view 2 (obl_left): 83019 points, camera at [0.411 0.181 0.344]
view 3 (obl_near): 88330 points, camera at [ 0.228 -0.003  0.339]
view 4 (left_22): 97814 points, camera at [0.295 0.073 0.345]
view 5 (center): 96865 points, camera at [ 0.3   -0.046  0.345]
view 6 (right_22): 97831 points, camera at [ 0.26  -0.157  0.345]
view 7 (obl_right): 92864 points, camera at [ 0.411 -0.186  0.342]
view 8 (obl_near_right): 95758 points, camera at [ 0.281 -0.132  0.339]
view 9 (right_45): 96607 points, camera at [ 0.18  -0.244  0.345]
################ object 0 ################
[1.1] scans: ['scan_0.npz', 'scan_1.npz', 'scan_2.npz', 'scan_3.npz', 'scan_4.npz', 'scan_5.npz', 'scan_6.npz', 'scan_7.npz', 'scan_8.npz', 'scan_9.npz']
[1.1] table of view 0 tilted 5.60 deg, offset +22.6 mm vs the centre view: aligned
[1.1] table of view 1 tilted 2.27 deg, offset +32.8 mm vs the centre view: aligned
[1.1] table of view 2 tilted 5.82 deg, offset +17.5 mm vs the centre view: aligned
[1.1] table of view 3 tilted 1.23 deg, offset +7.0 mm vs the centre view: aligned
[1.1] table of view 4 tilted 0.81 deg, offset +32.8 mm vs the centre view: aligned
[1.1] table of view 6 tilted 1.83 deg, offset -3.5 mm vs the centre view: aligned
[1.1] table of view 7 tilted 4.65 deg, offset +12.0 mm vs the centre view: aligned
[1.1] table of view 8 tilted 4.32 deg, offset +1.5 mm vs the centre view: aligned
[1.1] table of view 9 tilted 3.21 deg, offset +4.8 mm vs the centre view: aligned
[1.1] ICP scan 0 -> scan 5: shift 4.2 mm, rot 0.63 deg, trimmed rms 4.02 mm
[1.1] ICP scan 1 -> scan 5: shift 3.5 mm, rot 0.55 deg, trimmed rms 3.52 mm
[1.1] ICP scan 2 -> scan 5: shift 13.2 mm, rot 1.85 deg, trimmed rms 4.38 mm
[1.1] ICP scan 3 -> scan 5: shift 6.0 mm, rot 1.01 deg, trimmed rms 3.53 mm
[1.1] ICP scan 4 -> scan 5: shift 8.2 mm, rot 1.36 deg, trimmed rms 4.48 mm
[1.1] ICP scan 6 -> scan 5: shift 5.7 mm, rot 0.93 deg, trimmed rms 2.83 mm
[1.1] ICP scan 7 -> scan 5: shift 1.4 mm, rot 0.24 deg, trimmed rms 3.98 mm
[1.1] ICP scan 8 -> scan 5: shift 9.8 mm, rot 1.46 deg, trimmed rms 3.61 mm
[1.1] ICP scan 9 -> scan 5: shift 4.8 mm, rot 0.69 deg, trimmed rms 3.00 mm
[1.0] 10 viewpoints: obl_near_left (camera z 0.341 m), left_45 (camera z 0.345 m), obl_left (camera z 0.344 m), obl_near (camera z 0.339 m), left_22 (camera z 0.345 m), center (camera z 0.345 m), right_22 (camera z 0.345 m), obl_right (camera z 0.342 m), obl_near_right (camera z 0.339 m), right_45 (camera z 0.345 m)
[1.1] merged 932276 -> 205859 points (voxel 0.004 m)
[1.2] workspace: 108722 points
[1.3] plane normal=(-0.065,-0.028,0.998) -> 76534 inliers, z at x,y=0: -0.0519 m
[1.4] object points: 14276 (> 12 mm above plane; 4971 noise points and 1276 single-view points removed)
[1.5] 2 cluster(s):
      cluster 0: 6393 pts, extent [0.146 0.127 0.077], top 88 mm above table, centroid [ 0.366 -0.12   0.011]
      cluster 1: 4760 pts, extent [0.127 0.093 0.122], top 130 mm above table, centroid [0.368 0.178 0.051]
[2.1] centroids of all objects: object 0: (0.366, -0.120, 0.011); object 1: (0.368, 0.178, 0.051)
      target = object 0 of 2 on the table (nearest-first order)
[2.1] centroid = [ 0.3655 -0.1198  0.0107], object height 72 mm above the table
[2.2] 6393 normals; mean angle to the centroid direction = 50.4 deg
[2.3] antipodal pair: p1=[ 0.3753 -0.1087  0.0268] p2=[ 0.3512 -0.0933  0.0224] width=29.0 mm, mean grasp-point distance to the centroid=27.1 mm, cone quality=0.914
[3] PCA eigenvalues [0.00028  0.000781 0.001337], major axis [-0.992  0.128  0.008]; approach [-0.052 -0.346 -0.937] via cross(PCA major axis, grasp axis)
pre_grasp: tool xyz [ 0.37  -0.054  0.124] theta [161.4  -8.7 -32.8] | bounds ok | IK max joint step 39 deg -> OK
grasp    : tool xyz [ 0.365 -0.089  0.03 ] theta [161.4  -8.7 -32.8] | bounds ok | IK max joint step 19 deg -> OK
lift     : tool xyz [ 0.365 -0.089  0.15 ] theta [161.4  -8.7 -32.8] | bounds ok | IK max joint step 21 deg -> OK
PLAN OK
-> pre-grasp: tool xyz [ 0.37  -0.054  0.124], theta [161.4  -8.7 -32.8]
   reached, position error 0.8 mm
   close-range object: 751 points, centroid (0.3690, -0.1096), 10.8 mm from the far-range centroid
[2.1] centroid = [ 0.369  -0.1096  0.0067], object height 61 mm above the table
[2.2] 751 normals; mean angle to the centroid direction = 66.9 deg
[2.3] no usable antipodal pair; grasping across the body near its middle at (0.370, -0.115), local body direction [-1.   -0.01], width 55 mm
[3] PCA eigenvalues [4.30e-05 3.90e-04 9.82e-04], major axis [-1.    -0.018 -0.023]; approach [ 0.  0. -1.] via vertical (grasping across the body)
   close-range grasp (across_body): grasp point (0.3704, -0.1147) (far-range grasp point was (0.3632, -0.1010))
   close-range re-synthesis: approach tilt +0 deg, reachable
   refined grasp: approach tilt +0 deg (re-searched with IK), wrist rotation 58 deg
   refined: table z -0.0426, fingertip ends will stop at z -0.0181 (24.5 mm above the table)
-> pre-grasp (refined): tool xyz [ 0.37  -0.115  0.117], theta [180.    0.  -89.3]
   reached, position error 1.1 mm
-> descend (stage 1): tool xyz [ 0.37  -0.115  0.057], theta [180.    0.  -89.3]
   reached, position error 1.6 mm
-> descend to grasp (slow): tool xyz [ 0.37  -0.115  0.017], theta [180.    0.  -89.3]
   reached, position error 0.8 mm
   gripper closed to 0.74, contact=True
-> lift: tool xyz [ 0.37  -0.115  0.137], theta [180.    0.  -89.3]
   reached, position error 1.0 mm
   after lift gripper at 0.72 (was 0.74) -> HOLDING
-> lower to the table (place back): tool xyz [ 0.37  -0.115  0.02 ], theta [180.    0.  -89.3]
   reached, position error 1.1 mm
   gripper opened smoothly -> 0.00
-> retreat up: tool xyz [ 0.37  -0.115  0.137], theta [180.    0.  -89.3]
   reached, position error 1.0 mm
PICK + LIFT + RELEASE COMPLETE (place_back)
--- object 0: PICK + LIFT + RELEASE COMPLETE (place_back)
################ object 1 ################
[1.1] scans: ['scan_0.npz', 'scan_1.npz', 'scan_2.npz', 'scan_3.npz', 'scan_4.npz', 'scan_5.npz', 'scan_6.npz', 'scan_7.npz', 'scan_8.npz', 'scan_9.npz']
[1.1] table of view 0 tilted 5.60 deg, offset +22.6 mm vs the centre view: aligned
[1.1] table of view 1 tilted 2.27 deg, offset +32.8 mm vs the centre view: aligned
[1.1] table of view 2 tilted 5.82 deg, offset +17.5 mm vs the centre view: aligned
[1.1] table of view 3 tilted 1.23 deg, offset +7.0 mm vs the centre view: aligned
[1.1] table of view 4 tilted 0.81 deg, offset +32.8 mm vs the centre view: aligned
[1.1] table of view 6 tilted 1.81 deg, offset -3.5 mm vs the centre view: aligned
[1.1] table of view 7 tilted 4.66 deg, offset +11.3 mm vs the centre view: aligned
[1.1] table of view 8 tilted 4.06 deg, offset +2.5 mm vs the centre view: aligned
[1.1] table of view 9 tilted 3.21 deg, offset +4.8 mm vs the centre view: aligned
[1.1] ICP scan 0 -> scan 5: shift 4.2 mm, rot 0.63 deg, trimmed rms 4.02 mm
[1.1] ICP scan 1 -> scan 5: shift 3.5 mm, rot 0.55 deg, trimmed rms 3.52 mm
[1.1] ICP scan 2 -> scan 5: shift 13.2 mm, rot 1.85 deg, trimmed rms 4.38 mm
[1.1] ICP scan 3 -> scan 5: shift 6.0 mm, rot 1.01 deg, trimmed rms 3.53 mm
[1.1] ICP scan 4 -> scan 5: shift 8.2 mm, rot 1.36 deg, trimmed rms 4.48 mm
[1.1] ICP scan 6 -> scan 5: shift 5.7 mm, rot 0.94 deg, trimmed rms 2.83 mm
[1.1] ICP scan 7 -> scan 5: shift 1.3 mm, rot 0.19 deg, trimmed rms 3.98 mm
[1.1] ICP scan 8 -> scan 5: shift 8.7 mm, rot 1.10 deg, trimmed rms 3.61 mm
[1.1] ICP scan 9 -> scan 5: shift 4.8 mm, rot 0.69 deg, trimmed rms 3.00 mm
[1.0] 10 viewpoints: obl_near_left (camera z 0.341 m), left_45 (camera z 0.345 m), obl_left (camera z 0.344 m), obl_near (camera z 0.339 m), left_22 (camera z 0.345 m), center (camera z 0.345 m), right_22 (camera z 0.345 m), obl_right (camera z 0.342 m), obl_near_right (camera z 0.339 m), right_45 (camera z 0.345 m)
[1.1] merged 932276 -> 206047 points (voxel 0.004 m)
[1.2] workspace: 108812 points
[1.3] plane normal=(-0.066,-0.030,0.997) -> 76804 inliers, z at x,y=0: -0.0523 m
[1.4] object points: 14016 (> 12 mm above plane; 4730 noise points and 1229 single-view points removed)
[1.5] 2 cluster(s):
      cluster 0: 6410 pts, extent [0.146 0.127 0.077], top 88 mm above table, centroid [ 0.366 -0.12   0.011]
      cluster 1: 4750 pts, extent [0.128 0.096 0.121], top 129 mm above table, centroid [0.369 0.178 0.051]
[2.1] centroids of all objects: object 0: (0.366, -0.120, 0.011); object 1: (0.369, 0.178, 0.051)
      target = object 1 of 2 on the table (nearest-first order)
[2.1] centroid = [0.3686 0.1784 0.0514], object height 115 mm above the table
[2.2] 4750 normals; mean angle to the centroid direction = 51.7 deg
[2.3] antipodal pair: p1=[0.3546 0.1707 0.055 ] p2=[0.3575 0.2209 0.0637] width=51.0 mm, mean grasp-point distance to the centroid=31.0 mm, cone quality=0.953
[3] PCA eigenvalues [0.00024  0.000833 0.001125], major axis [-0.361  0.42   0.833]; approach [ 0.01   0.17  -0.985] via vertical made orthogonal to the grasp axis (PCA major axis ill-defined: near-isotropic object)
pre_grasp: tool xyz [0.355 0.173 0.14 ] theta [-180.     9.8  -93.2] | bounds ok | IK max joint step 38 deg -> OK
grasp    : tool xyz [0.356 0.19  0.042] theta [-180.     9.8  -93.2] | bounds ok | IK max joint step 17 deg -> OK
lift     : tool xyz [0.356 0.19  0.162] theta [-180.     9.8  -93.2] | bounds ok | IK max joint step 21 deg -> OK
PLAN OK
-> pre-grasp: tool xyz [0.355 0.173 0.14 ], theta [-180.     9.8  -93.2]
   reached, position error 1.5 mm
   close-range object: 483 points, centroid (0.3694, 0.1968), 18.4 mm from the far-range centroid
[2.1] centroid = [0.3694 0.1968 0.06  ], object height 95 mm above the table
[2.2] 483 normals; mean angle to the centroid direction = 83.0 deg
[2.3] no usable antipodal pair; grasping across the body near its middle at (0.371, 0.195), local body direction [-1.    0.08], width 61 mm
[3] PCA eigenvalues [3.00e-06 3.19e-04 7.53e-04], major axis [-0.985  0.164 -0.047]; approach [ 0.  0. -1.] via vertical (grasping across the body)
   close-range grasp (across_body): grasp point (0.3712, 0.1954) (far-range grasp point was (0.3560, 0.1958))
   close-range re-synthesis: approach tilt +0 deg, reachable
   refined grasp: approach tilt +0 deg (re-searched with IK), wrist rotation 10 deg
   refined: table z -0.0347, fingertip ends will stop at z -0.0047 (30.0 mm above the table)
-> pre-grasp (refined): tool xyz [0.371 0.195 0.13 ], theta [-180.     0.   -94.6]
   reached, position error 0.8 mm
-> descend (stage 1): tool xyz [0.371 0.195 0.07 ], theta [-180.     0.   -94.6]
   reached, position error 1.4 mm
-> descend to grasp (slow): tool xyz [0.371 0.195 0.03 ], theta [-180.     0.   -94.6]
   reached, position error 0.8 mm
   gripper closed to 0.82, contact=True
-> lift: tool xyz [0.371 0.195 0.15 ], theta [-180.     0.   -94.6]
   reached, position error 1.7 mm
   after lift gripper at 0.82 (was 0.82) -> HOLDING
-> lower to the table (place back): tool xyz [0.371 0.195 0.033], theta [-180.     0.   -94.6]
   reached, position error 0.8 mm
   gripper opened smoothly -> 0.00
-> retreat up: tool xyz [0.371 0.195 0.15 ], theta [-180.     0.   -94.6]
   reached, position error 1.4 mm
PICK + LIFT + RELEASE COMPLETE (place_back)
--- object 1: PICK + LIFT + RELEASE COMPLETE (place_back)
################ object 2 ################
[1.1] scans: ['scan_0.npz', 'scan_1.npz', 'scan_2.npz', 'scan_3.npz', 'scan_4.npz', 'scan_5.npz', 'scan_6.npz', 'scan_7.npz', 'scan_8.npz', 'scan_9.npz']
[1.1] table of view 0 tilted 5.60 deg, offset +22.6 mm vs the centre view: aligned
[1.1] table of view 1 tilted 2.27 deg, offset +32.8 mm vs the centre view: aligned
[1.1] table of view 2 tilted 5.82 deg, offset +17.5 mm vs the centre view: aligned
[1.1] table of view 3 tilted 1.23 deg, offset +7.0 mm vs the centre view: aligned
[1.1] table of view 4 tilted 0.81 deg, offset +32.8 mm vs the centre view: aligned
[1.1] table of view 6 tilted 1.81 deg, offset -3.5 mm vs the centre view: aligned
[1.1] table of view 7 tilted 4.65 deg, offset +12.0 mm vs the centre view: aligned
[1.1] table of view 8 tilted 4.06 deg, offset +2.5 mm vs the centre view: aligned
[1.1] table of view 9 tilted 3.21 deg, offset +4.8 mm vs the centre view: aligned
[1.1] ICP scan 0 -> scan 5: shift 4.2 mm, rot 0.63 deg, trimmed rms 4.02 mm
[1.1] ICP scan 1 -> scan 5: shift 3.5 mm, rot 0.55 deg, trimmed rms 3.52 mm
[1.1] ICP scan 2 -> scan 5: shift 13.2 mm, rot 1.85 deg, trimmed rms 4.38 mm
[1.1] ICP scan 3 -> scan 5: shift 6.0 mm, rot 1.01 deg, trimmed rms 3.53 mm
[1.1] ICP scan 4 -> scan 5: shift 8.2 mm, rot 1.36 deg, trimmed rms 4.48 mm
[1.1] ICP scan 6 -> scan 5: shift 5.7 mm, rot 0.94 deg, trimmed rms 2.83 mm
[1.1] ICP scan 7 -> scan 5: shift 1.4 mm, rot 0.24 deg, trimmed rms 3.98 mm
[1.1] ICP scan 8 -> scan 5: shift 8.7 mm, rot 1.10 deg, trimmed rms 3.61 mm
[1.1] ICP scan 9 -> scan 5: shift 4.8 mm, rot 0.69 deg, trimmed rms 3.00 mm
[1.0] 10 viewpoints: obl_near_left (camera z 0.341 m), left_45 (camera z 0.345 m), obl_left (camera z 0.344 m), obl_near (camera z 0.339 m), left_22 (camera z 0.345 m), center (camera z 0.345 m), right_22 (camera z 0.345 m), obl_right (camera z 0.342 m), obl_near_right (camera z 0.339 m), right_45 (camera z 0.345 m)
[1.1] merged 932276 -> 205963 points (voxel 0.004 m)
[1.2] workspace: 108767 points
[1.3] plane normal=(-0.061,-0.028,0.998) -> 76354 inliers, z at x,y=0: -0.0505 m
[1.4] object points: 14612 (> 12 mm above plane; 5261 noise points and 1306 single-view points removed)
[1.5] 2 cluster(s):
      cluster 0: 6441 pts, extent [0.147 0.127 0.077], top 88 mm above table, centroid [ 0.366 -0.12   0.01 ]
      cluster 1: 4771 pts, extent [0.128 0.097 0.122], top 130 mm above table, centroid [0.369 0.178 0.051]
[2.1] centroids of all objects: object 0: (0.366, -0.120, 0.010); object 1: (0.369, 0.178, 0.051)
--- object 2: NO_MORE_OBJECTS
  reached the scan pose via: joint-space move to the saved scan joints
```
